#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>
#include <M5Unified.h>
#include "Robstride.h" // 作成したライブラリをインクルード

// --- ユーザー設定項目 ---
// お使いのESP32-S3とMCP2515の接続に合わせてピン番号を変更してください
const int SPI_CS_PIN = 6; // 例: GPIO 5
const int INT_PIN = 4;    // MCP2515の割り込みピン (メッセージ受信時にLOWになる)

// モーター側で設定したCAN ID (1~15)
const byte MOTOR_ID = 1;

// ホスト(このESP32)のCAN ID。モーターはこのID宛に返信します。
// 他のCANデバイスと重複しない値を設定してください。
const byte HOST_ID = 0x0A;

// --- グローバル変数 ---
MCP_CAN CAN0(SPI_CS_PIN);
Robstride motor(CAN0, MOTOR_ID, HOST_ID); // ホストIDを渡して初期化

unsigned long prev_command_time = 0;
unsigned long prev_display_update = 0;
int communication_count = 0;
unsigned long last_communication_time = 0;
bool motor_connected = false;

// モータースキャン用の変数
bool scan_mode = true;
byte current_scan_id = 1;
unsigned long scan_start_time = 0;
unsigned long scan_timeout = 2000; // 各IDのスキャンタイムアウト（ms）
bool found_motors[16] = {false};   // 見つかったモーターID（1-15）
byte active_motor_id = 0;

// モータースキャン関数
void scanMotor(byte motor_id)
{
  // モーターに有効化コマンドを送信
  byte data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
  CAN0.sendMsgBuf(motor_id, 0, 8, data);
  scan_start_time = millis();
}

// スキャン結果をチェック
bool checkScanResponse()
{
  if (!digitalRead(INT_PIN))
  {
    long unsigned int rxId;
    byte len = 0;
    byte rxBuf[8];

    CAN0.readMsgBuf(&rxId, &len, rxBuf);

    // ホストID宛の返信で、現在スキャン中のIDと一致する場合
    if (rxId == HOST_ID && len >= 6 && rxBuf[0] == current_scan_id)
    {
      found_motors[current_scan_id] = true;
      if (active_motor_id == 0)
      {
        active_motor_id = current_scan_id;
      }
      return true;
    }
  }
  return false;
}

// スキャンディスプレイ更新関数
void updateScanDisplay()
{
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(1);

  M5.Display.setCursor(10, 10);
  M5.Display.println("Motor ID Scanner");
  M5.Display.println("================");

  M5.Display.setCursor(10, 40);
  M5.Display.print("Scanning ID: ");
  M5.Display.println(current_scan_id);

  M5.Display.setCursor(10, 60);
  M5.Display.print("Timeout: ");
  M5.Display.print((scan_timeout - (millis() - scan_start_time)) / 1000);
  M5.Display.println("s");

  M5.Display.setCursor(10, 80);
  M5.Display.println("Found Motors:");

  int y_pos = 100;
  for (int i = 1; i <= 15; i++)
  {
    if (found_motors[i])
    {
      M5.Display.setCursor(10, y_pos);
      M5.Display.setTextColor(GREEN);
      M5.Display.print("ID ");
      M5.Display.print(i);
      M5.Display.println(": FOUND");
      y_pos += 15;
    }
  }

  if (active_motor_id > 0)
  {
    M5.Display.setTextColor(YELLOW);
    M5.Display.setCursor(10, 200);
    M5.Display.print("Active Motor: ID ");
    M5.Display.println(active_motor_id);
  }
}

// ディスプレイ更新関数
void updateDisplay()
{
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(1);

  // 通信状況の表示
  M5.Display.setCursor(10, 10);
  M5.Display.print("Comm Count: ");
  M5.Display.println(communication_count);

  M5.Display.setCursor(10, 30);
  if (motor_connected)
  {
    M5.Display.setTextColor(GREEN);
    M5.Display.println("Motor: CONNECTED");
  }
  else
  {
    M5.Display.setTextColor(RED);
    M5.Display.println("Motor: DISCONNECTED");
  }

  // 最後の通信時刻
  M5.Display.setTextColor(WHITE);
  M5.Display.setCursor(10, 50);
  M5.Display.print("Last Comm: ");
  M5.Display.print((millis() - last_communication_time) / 1000);
  M5.Display.println("s ago");

  // モーターの状態
  M5.Display.setCursor(10, 70);
  M5.Display.print("Pos: ");
  M5.Display.print(motor.getPosition(), 3);
  M5.Display.println(" rad");

  M5.Display.setCursor(10, 90);
  M5.Display.print("Vel: ");
  M5.Display.print(motor.getVelocity(), 3);
  M5.Display.println(" rad/s");

  M5.Display.setCursor(10, 110);
  M5.Display.print("Torque: ");
  M5.Display.print(motor.getTorque(), 3);
  M5.Display.println(" Nm");
}

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // シリアルモニタが開くまで待機

  // M5Stackの初期化
  M5.begin();
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("Robstride Motor Control");

  Serial.println("Robstride Motor Control Example (Verified with Datasheet)");

  // MCP2515の初期化
  // CANレート: 1Mbps, 水晶発振子: 8MHz (データシート推奨)
  if (CAN0.begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ) == CAN_OK)
  {
    Serial.println("MCP2515 Initialized Successfully!");
    // CAN初期化成功時はディスプレイを緑色に
    M5.Display.fillScreen(GREEN);
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 10);
    M5.Display.println("CAN Init: SUCCESS");
    M5.Display.setCursor(10, 40);
    M5.Display.println("Motor Control Ready");
  }
  else
  {
    Serial.println("Error Initializing MCP2515. Check wiring.");
    // CAN初期化失敗時はディスプレイを赤色に
    M5.Display.fillScreen(RED);
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 10);
    M5.Display.println("CAN Init: FAILED");
    M5.Display.setCursor(10, 40);
    M5.Display.println("Check wiring!");
    while (1)
      ;
  }
  CAN0.setMode(MCP_NORMAL);

  // 割り込みピンを入力に設定
  pinMode(INT_PIN, INPUT);

  Serial.println("Starting motor ID scan...");

  // --- デバイスIDの取得デモ ---
  Serial.println("Requesting MCU Unique ID...");

  // M5ディスプレイにデバイスID取得中を表示
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("Getting Device ID...");
  M5.Display.setTextSize(1);
  M5.Display.setCursor(10, 50);
  M5.Display.println("Please wait...");

  motor.requestDeviceID();

  unsigned long startTime = millis();
  bool idReceived = false;
  while (millis() - startTime < 500)
  { // 500ms待機
    if (!digitalRead(INT_PIN))
    {
      long unsigned int rxId;
      byte len = 0;
      byte rxBuf[8];
      CAN0.readMsgBuf(&rxId, &len, rxBuf);
      if (motor.parseReply(rxId, len, rxBuf))
      {
        if (motor.getMCUUniqueID() != 0)
        {
          idReceived = true;
          break;
        }
      }
    }
  }

  // M5ディスプレイに結果を表示
  M5.Display.fillScreen(BLACK);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("Device ID Result");
  M5.Display.println("================");

  if (idReceived)
  {
    M5.Display.setTextColor(GREEN);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 50);
    M5.Display.println("MCU Unique ID Received:");

    uint32_t high = motor.getMCUUniqueID() >> 32;
    uint32_t low = motor.getMCUUniqueID() & 0xFFFFFFFF;

    M5.Display.setCursor(10, 70);
    M5.Display.print("0x");
    if (high > 0)
    {
      M5.Display.print(high, HEX);
    }
    M5.Display.println(low, HEX);

    // シリアルにも出力
    Serial.print("MCU Unique ID Received: 0x");
    if (high > 0)
      Serial.print(high, HEX);
    Serial.println(low, HEX);
  }
  else
  {
    M5.Display.setTextColor(RED);
    M5.Display.setTextSize(1);
    M5.Display.setCursor(10, 50);
    M5.Display.println("Failed to receive");
    M5.Display.setCursor(10, 70);
    M5.Display.println("MCU Unique ID");

    Serial.println("Failed to receive MCU Unique ID.");
  }

  // 3秒間表示してからスキャンに移行
  delay(3000);
  // --- デモ終了 ---

  // スキャンモード開始
  scanMotor(current_scan_id);
  updateScanDisplay();
}

void loop()
{
  if (scan_mode)
  {
    // スキャンモード
    // 1. スキャン応答をチェック
    if (checkScanResponse())
    {
      Serial.print("Found motor with ID: ");
      Serial.println(current_scan_id);
    }

    // 2. タイムアウトチェック
    if (millis() - scan_start_time > scan_timeout)
    {
      // 次のIDに進む
      current_scan_id++;
      if (current_scan_id > 15)
      {
        // スキャン完了
        scan_mode = false;
        if (active_motor_id > 0)
        {
          // 見つかったモーターで通常モードに移行
          motor = Robstride(CAN0, active_motor_id, HOST_ID);
          motor.enableMotor();
          delay(1000);
          Serial.print("Switching to normal mode with motor ID: ");
          Serial.println(active_motor_id);
        }
        else
        {
          Serial.println("No motors found!");
        }
      }
      else
      {
        // 次のIDをスキャン
        scanMotor(current_scan_id);
      }
    }

    // 3. スキャンディスプレイ更新
    if (millis() - prev_display_update >= 200)
    {
      prev_display_update = millis();
      updateScanDisplay();
    }
  }
  else
  {
    // 通常モード
    // 1. CANメッセージの受信と解析
    // INT_PINがLOWなら、MCP2515に受信メッセージあり
    if (!digitalRead(INT_PIN))
    {
      long unsigned int rxId;
      byte len = 0;
      byte rxBuf[8];

      CAN0.readMsgBuf(&rxId, &len, rxBuf);

      // 受信したデータをライブラリに渡して解析させる
      if (motor.parseReply(rxId, len, rxBuf))
      {
        // 正常にデータを解析できた場合
        communication_count++;
        last_communication_time = millis();
        motor_connected = true;
      }
    }

    // 2. 20ms周期 (50Hz) でモーターに指令を送信
    if (millis() - prev_command_time >= 20)
    {
      prev_command_time = millis();

      // 目標位置をサイン波で生成 (振幅 ±1.5 rad, 周期 5秒)
      float time_sec = millis() / 1000.0f;
      float target_pos = 1.5f * sin(2.0f * PI / 5.0f * time_sec);

      // モーターへ位置指令を送信
      // setPosition(目標位置[rad], 速度FF[rad/s], トルクFF[Nm], Kp, Kd)
      // Kp, Kdはモーターや負荷に合わせて調整してください
      motor.setPosition(target_pos, 0.0, 0.0, 8.0, 1.0);

      // モーターの状態をシリアルモニタに出力
      Serial.print("Target: ");
      Serial.print(target_pos, 3);
      Serial.print("\t Actual: ");
      Serial.print(motor.getPosition(), 3);
      Serial.print("\t Vel: ");
      Serial.print(motor.getVelocity(), 3);
      Serial.print("\t Torque: ");
      Serial.println(motor.getTorque(), 3);
    }

    // 3. 500ms周期でディスプレイを更新
    if (millis() - prev_display_update >= 500)
    {
      prev_display_update = millis();

      // 5秒以上通信がない場合は切断とみなす
      if (millis() - last_communication_time > 5000)
      {
        motor_connected = false;
      }

      updateDisplay();
    }
  }
}
