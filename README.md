# RS02PrivateCAN README

M5Stack CoreS3 + MCP2515（1Mbps）で RS02/RS05 系モータを **PP / Velocity / Current / CSP** の各モードで動かし、**エンコーダ角（無限角度）や各種パラメータ**を画面に常時表示するための最小ライブラリとサンプルです。
**Type17（0x11）パラメータ読出し**で `mechPos(0x7019)` / `mechVel(0x701B)` をポーリングして角度をアンラップし、**Type2**が出ない個体でも角度が読めます。

---

## 1) 構成

```
your-project/
├─ src/
│   └─ main.cpp                // 画面表示・デモ・Angle∞
└─ lib/
   └─ rs02/
      ├─ library.json
      └─ src/
         ├─ RS02PrivateCAN.h   // ライブラリ（公開API）
         └─ RS02PrivateCAN.cpp // ライブラリ実装
```

**lib/rs02/library.json**

```json
{
  "name": "rs02",
  "version": "1.0.0",
  "build": { "srcFilter": "+<*.cpp>" }
}
```

> 既存の古い `RS02PrivateCAN.*` が別の場所に残っているとリンク競合します。**古いファイルは削除**してください。

---

## 2) ハードウェア接続（例）

* **MCP2515**（SPI）↔ **CoreS3**

  * CS: `GPIO6`（例。`main.cpp` の `CAN_CS_PIN` を合わせる）
  * SCK/MOSI/MISO は CoreS3 の SPI ピンへ
  * 1Mbps / 8MHz or 16MHz の **水晶設定**に合わせて `MCP_CLOCK` を `MCP_8MHZ` or `MCP_16MHZ` に変更
* CANバスは**終端120Ω**、電源・GND共通

---

## 3) ビルド環境（PlatformIO 例）

**platformio.ini（抜粋）**

```ini
[env:m5stack-cores3]
platform = espressif32
board = m5stack-cores3
framework = arduino
lib_deps =
  m5stack/M5Unified
  autowp/mcp_can
build_flags =
  -DCORE_DEBUG_LEVEL=0
```

---

## 4) 起動と操作（サンプル `main.cpp`）

* **起動時**：CAN初期化 → RS02PrivateCAN 初期化 → `masterId=0xFD` 設定
* **画面操作**

  * **A**：Monitor ON/OFF（200ms周期で各値を更新）
  * **B**：現在モードの **Bring-up + デモ**
  * **C**：モード切替（Velocity → PP → Current → CSP → …）

### 表示される主な項目

* **Angle∞**：`mechPos(0x7019)` を ±π 閾値でアンラップして累積角を表示（turns / rad / deg）
* **Vel**：`mechVel(0x701B)` \[rad/s]
* **Refs**：`loc_ref(0x7016)` / `spd_ref(0x700A)` / `iq_ref(0x7006)`
* **Limit**：`limit_spd(0x7017)` / `limit_cur(0x7018,0x2019)` / `limit_torque(0x700B)` / `acc_rad(0x7022)`
* **Gain**：`spd_kp(0x701C)` / `spd_ki(0x701D)` / `cur_kp(0x7010)` / `cur_ki(0x7011)` / `loc_kp(0x701E)`
* **Mode**：`run_mode(0x7005)`

---

## 5) モード別の基本手順（内部で実行される流れ）

* **Velocity (mode=2)**

  1. `run_mode=2` → **Enable**
  2. `limit_cur`,`acc_rad` 設定
  3. `spd_ref` を 50–100ms 間隔で継続送信
* **PP (mode=1)**

  1. `run_mode=1` → **Enable**
  2. `limit_spd` 設定
  3. `loc_ref` に位置 \[rad] を必要に応じ再送
* **Current (mode=3)**

  1. `run_mode=3` → **Enable**
  2. 必要なら `limit_torque` と `cur_kp/cur_ki` 設定
  3. **`iq_ref(0x7006)`** に電流\[A]（正/負でトルク方向）

  > `iq_ref=0` のままでは**回りません**（ゼロトルクが仕様）
* **CSP (mode=5)**

  1. `run_mode=5`（必要に応じて2回）
  2. `limit_spd`/`limit_cur`（旧/新両系）/`loc_kp` 設定 → **Enable**
  3. `loc_ref` に位置 \[rad]

---

## 6) ライブラリの主なAPI

```cpp
// 低レベル・基本
bool begin();
void setMasterId(uint8_t mid);   // 既定 0xFD
bool sendExt(unsigned long id, const uint8_t* payload, uint8_t len);
bool readAny(RS02PrivFrame& out);
bool parseFeedback(const RS02PrivFrame& f, RS02Feedback& out);

bool ping(uint8_t id);               // Type0
bool enable(uint8_t id);             // Type3
bool stop(uint8_t id, bool clear);   // Type4

// Param R/W（index=LE）
bool writeParamLE(uint8_t id, uint16_t idx, const uint8_t le[4]);
bool writeFloatParam(uint8_t id, uint16_t idx, float v);
bool readParamRaw(uint8_t id, uint16_t idx, uint8_t out4LE[4]);
bool readFloatParam(uint8_t id, uint16_t idx, float& out);

// レポート/プロトコル
bool setActiveReport(uint8_t id, bool enable);  // Type24
bool setReportIntervalTicks(uint8_t id, uint16_t ticks);
bool switchProtocol(uint8_t id, uint8_t fcmd);  // Type25

// Operation Control (Type1)
bool opControl(uint8_t id, float torqueNm, float posRad, float velRadS, float kp, float kd);

// ランモードユーティリティ
bool setRunMode(uint8_t id, uint8_t mode);
bool readRunMode(uint8_t id, uint8_t& out);

// Velocity / PP / Current / CSP
bool enterVelocity(...);  bool velocityRef(...);
bool bringUpVelocityPerSpec(...);

bool enterPP(...);        bool ppLocRef(...);   bool bringUpPPPerSpec(...);

bool enterCurrent(...);   bool currentIqRef(...); bool bringUpCurrentPerSpec(...);

bool enterCSP(...);       bool cspLocRef(...);
bool enterCSP_simple(...);
bool enterCSP_robust(...);

// 参考：無限回転（パラメータ合成; FWによって未更新の個体あり）
bool getInfiniteByParams(uint8_t id, double& turns, double& angleRad);
```

---

## 7) 使用するインデックス（抜粋）

| 名称              |           Index | 型     | 備考                                              |
| --------------- | --------------: | ----- | ----------------------------------------------- |
| `RUN_MODE`      |        `0x7005` | `u8`  | 0=Operation, 1=PP, 2=Velocity, 3=Current, 5=CSP |
| `SPD_REF`       |        `0x700A` | `f32` | rad/s                                           |
| `IQ_REF`        |        `0x7006` | `f32` | A（Current 指令）                                   |
| `LIMIT_TORQUE`  |        `0x700B` | `f32` | Nm                                              |
| `LOC_REF`       |        `0x7016` | `f32` | rad                                             |
| `LIMIT_SPD`     |        `0x7017` | `f32` | rad/s                                           |
| `LIMIT_CUR`     |        `0x7018` | `f32` | A（新系）                                           |
| `MECH_POS`      |        `0x7019` | `f32` | 機械角 \[rad]（**Angle∞ の基点**）                      |
| `MECH_VEL`      |        `0x701B` | `f32` | 角速度 \[rad/s]                                    |
| `SPD_KP/KI`     | `0x701C/0x701D` | `f32` | 速度ループ                                           |
| `LOC_KP`        |        `0x701E` | `f32` | 位置ループ                                           |
| `CUR_KP/KI`     | `0x7010/0x7011` | `f32` | 電流ループ                                           |
| `ACC_RAD`       |        `0x7022` | `f32` | rad/s²                                          |
| `LIMIT_CUR_OLD` |        `0x2019` | `f32` | 旧系                                              |

---

## 8) IDとマスターID

* **モータID**：既定 `0x7F`（必要に応じて `main.cpp` の `MOTOR_ID` を変更）
* **マスターID（DA2上位）**：既定 `0xFD`（`RS.setMasterId(0xFD)`）
* **Type17 応答の宛先（dst）**は**`hostId`/`0x00/0xFF/0xFE/targetId`** をすべて許容
  → 実機の応答が `dst=targetId` でも正しく拾えます

---

## 9) トラブルシュート

* **リンクエラー（undefined reference）**
  → ライブラリ配置（`lib/rs02/src/`）と **.h/.cpp の宣言一致**を確認。古い複製を削除。
* **角度が 0 のまま**
  → 本版は `0x7019/0x701B` を **Type17 読み**で取得。**Monitor ON** の状態でモータを少し動かす/任意の指令を出すと更新されます。
* **Current で回らない**
  → 仕様通り。`iq_ref(0x7006)` を与えないと回りません（ゼロトルク）。`limit_torque` や `cur_kp/cur_ki` の設定も確認。
* **速度/位置が鈍い**
  → `limit_cur`/`limit_spd`/`acc_rad`/`loc_kp` を調整。
* **MCP\_CLOCK ミスマッチ**
  → 8MHz/16MHz の設定を基板に合わせて切替。
* **終端不良や配線**
  → CANH/CANL 終端 120Ω、電源/GND 共通、配線長・ノイズ対策を確認。

---

## 10) 安全上の注意

* 実機にトルクがかかります。**負荷を外し**、\*\*低い上限（電流/トルク）\*\*から試し、少しずつ上げてください。
* 緊急停止は `stop(id, true)`（フォルトクリア付き）を用意。
* ギア・機構の**ストッパ**に注意（PP/CSP の位置指令で過大移動を防ぐ）。

---

## 11) よくある質問（FAQ）

* **Q: Type2 が出ないけど大丈夫？**
  A: はい。本実装は `0x7019/0x701B` を **Type17 読み**で角度/速度を取得します。Type2 は任意で `setActiveReport(true)` を送っておけば、出る個体では併用可能です。

* **Q: 以前 Type25 で MIT に切り替えた気がする**
  A: `switchProtocol(id, 0)`（Private）で戻せますが、応答が来ない場合は電源再投入や別経路（ブート設定）を試してください。

---

## 12) 変更したいとき

* **ID変更**：`main.cpp` の `MOTOR_ID`
* **マスターID変更**：`setup()` 内 `RS.setMasterId(0xFD);`
* **CANクロック/CSピン**：`MCP_CLOCK` と `CAN_CS_PIN`
* **ポーリング周期**：`monitorTick()` の更新間隔（既定 200ms）

---

以上です。
この README の通りに配置＆ビルドすれば、**画面に Angle∞ / Vel / Refs / Limit / Gain / Mode** が出て、**B ボタン**で各モードのデモが動きます。カスタム要望（表示追加・ログCSV保存・PC連携など）もいつでも言ってください！
