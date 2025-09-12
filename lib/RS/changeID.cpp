// main.cpp — M5Unified 画面UIでモーターID変更（Type7 / Param(0x200A)）
// ・A/C：値変更（-1 / +1, 切替）
// ・B短押し：カーソル移動
// ・B長押し：適用（Apply）→ 新IDで 0x200A を読戻し（Verify）
//
// ハード: M5Stack CoreS3 + MCP2515(1Mbps), mcp_can(4引数 readMsgBuf 版)
// 前提: RS02PrivateCAN に setMotorId / setMotorIdViaParam / saveParams を実装済み

#include <M5Unified.h>
#include <SPI.h>
#include <mcp_can.h>
#include "RS02PrivateCAN.h"

// ===== MCP2515 設定 =====
#define CAN_CS_PIN 6
#define CAN_BAUD CAN_1000KBPS
#define MCP_CLOCK MCP_8MHZ // 基板に合わせ 16MHzなら MCP_16MHZ

// ===== ID/ホスト設定 =====
static constexpr uint8_t HOST_ID = 0x00;   // 自ホスト
static constexpr uint8_t MASTER_ID = 0xFD; // 実機に合わせる

// ===== CAN / ライブラリ =====
MCP_CAN CAN(CAN_CS_PIN);
RS02PrivateCAN RS(CAN, HOST_ID);

// ===== 画面 =====
M5Canvas spr(&M5.Display);
static const int W = 320, H = 240, PAD = 6;

// ===== UI State =====
enum class Method : uint8_t
{
    Type7_Immediate = 0,
    Param_200A = 1
};
static uint8_t curId = 0x7F; // 現在ID（対象）
static uint8_t newId = 0x01; // 新ID
static Method method = Method::Type7_Immediate;
static bool saveFlag = true; // Param方式時の保存(Type22)
static int cursor = 0;       // 0..3（Current / New / Method / Save）
static char statusLine[120] = "Ready: B short=Next, B long=Apply";

// ===== ヘルパ =====
static void drawUI()
{
    spr.fillScreen(BLACK);
    spr.setTextSize(1);
    spr.setTextColor(WHITE, BLACK);

    // タイトル
    spr.setCursor(PAD, PAD);
    spr.printf("RS02 ID Changer  HOST=0x%02X  MASTER=0x%02X", HOST_ID, MASTER_ID);

    // ヘッダライン
    spr.setCursor(PAD, PAD + 16);
    spr.printf("A:-  C:+  |  B short: Next  /  B long: Apply");

    // 枠
    spr.drawRect(2, 42, W - 4, 140, 0x7BEF);

    // 項目行（固定位置 4行）
    auto rowY = [&](int i)
    { return 50 + i * 26; };
    auto drawItem = [&](int i, const char *key, const char *val, bool selected)
    {
        int y = rowY(i);
        // 行背景クリア
        spr.fillRect(6, y - 2, W - 12, 24, selected ? 0x0821 /*濃紺*/ : BLACK);
        spr.setCursor(10, y);
        spr.setTextColor(selected ? 0xFFFF : 0xEFFF, selected ? 0x0821 : BLACK);
        spr.printf("%s", key);
        spr.setCursor(160, y);
        spr.printf("%s", val);
    };

    // 各フィールド文字列
    char v0[32], v1[32], v2[32], v3[32];
    snprintf(v0, sizeof(v0), "0x%02X", curId);
    snprintf(v1, sizeof(v1), "0x%02X", newId);
    snprintf(v2, sizeof(v2), "%s", (method == Method::Type7_Immediate) ? "Type7 (Immediate)" : "Param (0x200A)");
    snprintf(v3, sizeof(v3), "%s", saveFlag ? "SAVE: ON (Type22)" : "SAVE: OFF");

    drawItem(0, "Current ID", v0, cursor == 0);
    drawItem(1, "New ID    ", v1, cursor == 1);
    drawItem(2, "Method    ", v2, cursor == 2);
    drawItem(3, "Save Param", v3, cursor == 3);

    // ステータス
    spr.drawRect(2, 186, W - 4, 48, 0x7BEF);
    spr.setCursor(PAD, 192);
    spr.setTextColor(WHITE, BLACK);
    spr.printf("%s", statusLine);

    spr.pushSprite(0, 0);
}

static void clampIds()
{
    // 有効範囲 0..127 を想定（必要に応じて変更）
    if (curId > 127)
        curId = 127;
    if (newId > 127)
        newId = 127;
}

static void setStatus(const char *fmt, ...)
{
    va_list ap;
    va_start(ap, fmt);
    vsnprintf(statusLine, sizeof(statusLine), fmt, ap);
    va_end(ap);
}

static bool readBackCanId(uint8_t id, uint8_t &out)
{
    uint8_t le[4] = {0};
    if (!RS.readParamRaw(id, 0x200A, le))
        return false; // CAN_ID param
    out = le[0];
    return true;
}

static void applyChange()
{
    // 実行
    bool ok = false;
    if (method == Method::Type7_Immediate)
    {
        setStatus("Type7 apply: 0x%02X -> 0x%02X ...", curId, newId);
        drawUI();
        ok = RS.setMotorId(curId, newId);
    }
    else
    {
        setStatus("Param apply: id=0x%02X -> 0x%02X (save=%d) ...", curId, newId, saveFlag ? 1 : 0);
        drawUI();
        ok = RS.setMotorIdViaParam(curId, newId, saveFlag);
    }

    if (!ok)
    {
        setStatus("SEND NG");
        drawUI();
        return;
    }

    // 検証：新IDで 0x200A を読戻し
    uint8_t val = 0;
    bool okRead = false;
    uint32_t t0 = millis();
    while (millis() - t0 < 800)
    {
        if (readBackCanId(newId, val))
        {
            okRead = true;
            break;
        }
        delay(20);
    }

    if (okRead)
        setStatus("OK: 0x200A @ newId=0x%02X -> 0x%02X", newId, val);
    else
        setStatus("Applied. Verify failed (try again)");
    drawUI();
}

// ===== Arduino Lifecycle =====
void setup()
{
    auto cfg = M5.config();
    M5.begin(cfg);
    M5.Display.setTextWrap(false, false);

    spr.setColorDepth(8);
    spr.createSprite(W, H);
    spr.setTextWrap(false, false);
    spr.setTextSize(1);

    // CAN初期化
    SPI.begin();
    if (CAN.begin(MCP_ANY, CAN_BAUD, MCP_CLOCK) != CAN_OK)
    {
        spr.fillScreen(BLACK);
        spr.setCursor(PAD, PAD);
        spr.setTextColor(RED, BLACK);
        spr.println("CAN.begin FAIL");
        spr.pushSprite(0, 0);
        while (1)
            delay(1000);
    }
    CAN.setMode(MCP_NORMAL);
    RS.begin();
    RS.setMasterId(MASTER_ID);

    drawUI();
}

void loop()
{
    M5.update();

    // A/C: 値変更
    if (M5.BtnA.wasPressed())
    {
        switch (cursor)
        {
        case 0:
            if (curId > 0)
                curId--;
            break;
        case 1:
            if (newId > 0)
                newId--;
            break;
        case 2:
            method = (method == Method::Type7_Immediate) ? Method::Param_200A : Method::Type7_Immediate;
            break;
        case 3:
            saveFlag = !saveFlag;
            break;
        }
        clampIds();
        drawUI();
    }
    if (M5.BtnC.wasPressed())
    {
        switch (cursor)
        {
        case 0:
            if (curId < 127)
                curId++;
            break;
        case 1:
            if (newId < 127)
                newId++;
            break;
        case 2:
            method = (method == Method::Type7_Immediate) ? Method::Param_200A : Method::Type7_Immediate;
            break;
        case 3:
            saveFlag = !saveFlag;
            break;
        }
        clampIds();
        drawUI();
    }

    // B：短押しでカーソル移動、長押しで適用
    static bool bPressed = false;
    static uint32_t bDownAt = 0;

    if (M5.BtnB.wasPressed())
    {
        bPressed = true;
        bDownAt = millis();
    }
    if (M5.BtnB.wasReleased() && bPressed)
    {
        uint32_t dur = millis() - bDownAt;
        bPressed = false;
        if (dur >= 600)
        {
            applyChange();
        }
        else
        {
            cursor = (cursor + 1) % 4;
            setStatus("Select next. B long=Apply");
            drawUI();
        }
    }

    delay(5);
}
