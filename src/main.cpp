#include <SPI.h>
#include <mcp_can.h>
#include <M5Unified.h>
#include "RS02PrivateCAN.h"

// ====== ハード設定 ======
#define CAN_CS_PIN 6
// MCP2515 の水晶に合わせて切り替え（多くは 16MHz です）
#define MCP_CLOCK MCP_8MHZ // 16MHz の場合は MCP_16MHZ
#define CAN_BAUD CAN_1000KBPS

MCP_CAN CAN(CAN_CS_PIN);
RS02PrivateCAN RS(CAN, /*hostId*/ 0xFD);

// ====== 動作用の設定 ======
constexpr uint8_t MOTOR_ID = 0x7F; // まずは1から試す
bool activeReportOn = false;

// ====== 操作用の状態 ======
uint8_t modeState = 0; // 0:pos, 1:velocity, 2:current, 3:CSP
uint8_t posState = 0;  // 0:90度, 1:0度, 2:-90度
uint32_t lastOpMs = 0;
const char *modeNames[] = {"Position", "Velocity", "Current", "CSP"};

// ====== 画面ログユーティリティ ======
int lineHeight = 14;

void drawHeader(const char *status, uint16_t color = CYAN)
{
  auto &D = M5.Display;
  D.fillRect(0, 0, D.width(), 28, BLACK);
  D.setTextColor(color, BLACK);
  D.setCursor(0, 0);
  D.printf("RS02 Private Debug  |  Mode: %s  |  %s\n", modeNames[modeState], status);
  D.setTextColor(WHITE, BLACK);
  D.drawLine(0, 28, D.width(), 28, 0x7BEF);
  D.setCursor(0, 30);
}
void logLine(const String &s, uint16_t color = WHITE)
{
  Serial.println(s);
  auto &D = M5.Display;
  int y = D.getCursorY();
  if (y > D.height() - lineHeight)
  {
    D.fillRect(0, 30, D.width(), D.height() - 30, BLACK); // 下部ログエリアだけ消す
    D.setCursor(0, 30);
  }
  D.setTextColor(color, BLACK);
  D.println(s);
}

// ====== 受信フレームの表示 ======
void showRawFrame(const RS02PrivFrame &f)
{
  String s = String(f.isExt ? "[EXT]" : "[STD]") + " ID=0x" + String(f.id, HEX) + " DLC=" + String(f.dlc) + " Data:";
  for (int i = 0; i < f.dlc; i++)
    s += " " + String(f.data[i], HEX);
  logLine(s, 0xCFFF);
}

// ====== 初期化 ======
void setup()
{
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setTextSize(1);
  M5.Display.setFont(&fonts::Font0);
  M5.Display.fillScreen(BLACK);
  drawHeader("Booting...");

  Serial.begin(115200);
  while (!Serial)
  {
  }

  // MCP2515 初期化
  while (CAN.begin(MCP_ANY, CAN_BAUD, MCP_CLOCK) != CAN_OK)
  {
    drawHeader("MCP2515 init FAIL (retry)", RED);
    delay(300);
  }
  CAN.setMode(MCP_NORMAL);
  drawHeader("MCP2515 init OK", GREEN);

  RS.begin();

  // Private Ping（応答があればPrivateで生存）
  bool ok = RS.ping(MOTOR_ID);
  logLine(ok ? "Ping TX OK" : "Ping TX FAIL", ok ? GREEN : RED);
}

void loop()
{
  M5.update();

  // --- ボタン操作 ---
  if (M5.BtnA.wasPressed())
  {
    bool ok = RS.enable(MOTOR_ID);
    drawHeader(ok ? "Enable sent" : "Enable FAIL", ok ? GREEN : RED);
    logLine(ok ? "Enable TX OK" : "Enable TX FAIL", ok ? GREEN : RED);
  }
  if (M5.BtnB.wasPressed())
  {
    bool ok = false;
    modeState = (modeState + 1) & 0x03; // 0-3を循環（処理前に更新）
    String modeStr = modeNames[modeState];

    switch (modeState)
    {
    case 0: // Position mode (Operation Control)
    {
      float pos;
      switch (posState)
      {
      case 0:
        pos = 0.5f * PI; // +90度
        break;
      case 1:
        pos = 0.0f; // 0度
        break;
      case 2:
        pos = -0.5f * PI; // -90度
        break;
      }
      float vmax = 6.0f, kp = 4.0f, kd = 1.0f;
      ok = RS.opControl(MOTOR_ID, 0.0f, pos, vmax, kp, kd);
      logLine(String("Position mode: pos=") + String(pos, 3) + " rad", ok ? CYAN : RED);
      posState = (posState + 1) % 3; // 0-2を循環
    }
    break;

    case 1: // Velocity mode
    {
      float velocity;
      switch (posState)
      {
      case 0:
        velocity = 5.0f; // 5 rad/s
        break;
      case 1:
        velocity = 0.0f; // 0 rad/s (停止)
        break;
      case 2:
        velocity = -5.0f; // -5 rad/s (逆回転)
        break;
      }
      ok = RS.enterVelocityStrict(MOTOR_ID, 8.0f, 10.0f, 20.0f); // 8Nm制限, 10A制限, 20rad/s^2加速度
      logLine(String("enterVelocityStrict: ") + (ok ? "OK" : "FAIL"), ok ? GREEN : RED);
      if (ok)
      {
        delay(100); // 設定完了を待機
        ok = RS.velocityRef(MOTOR_ID, velocity);
        logLine(String("velocityRef: ") + (ok ? "OK" : "FAIL"), ok ? GREEN : RED);
      }
      logLine(String("Velocity mode: ") + String(velocity, 1) + " rad/s", ok ? CYAN : RED);
      posState = (posState + 1) % 3; // 0-2を循環
    }
    break;

    case 2: // Current mode
    {
      ok = RS.enterCurrent(MOTOR_ID, 5.0f); // 5Nm制限
      if (ok)
      {
        ok = RS.currentIqRef(MOTOR_ID, 2.0f); // 2A
      }
      logLine("Current mode: 2.0A", ok ? CYAN : RED);
    }
    break;

    case 3: // CSP mode
    {
      ok = RS.enterCSP(MOTOR_ID, 5.0f, 10.0f); // 5rad/s制限, 10A制限
      if (ok)
      {
        ok = RS.cspLocRef(MOTOR_ID, 1.0f); // 1 rad位置
      }
      logLine("CSP mode: 1.0 rad position", ok ? CYAN : RED);
    }
    break;
    }

    drawHeader(ok ? String("Mode: " + modeStr).c_str() : "Mode FAIL", ok ? CYAN : RED);
    lastOpMs = millis();
  }
  if (M5.BtnC.wasPressed())
  {
    bool ok = RS.stop(MOTOR_ID, /*clearFault*/ true);
    drawHeader(ok ? "Stop + FaultClear" : "Stop FAIL", ok ? 0xFFE0 : RED);
    logLine(ok ? "Stop&Clear TX OK" : "Stop TX FAIL", ok ? 0xFFE0 : RED);
  }

  // --- 受信処理 ---
  RS02PrivFrame f;
  while (RS.readAny(f))
  {
    // フィードバック(Type 2)ならパースして見やすく表示
    RS02PrivateCAN::Feedback fb;
    if (RS.parseFeedback(f, fb))
    {
      char line[128];
      snprintf(line, sizeof(line),
               "[FB] id=%u mode=%u fault=0x%02X  pos=%.3f rad  vel=%.3f  tq=%.3f  T=%.1fC",
               fb.motorId, fb.mode, fb.faultBits, fb.angleRad, fb.velRadS, fb.torqueNm, fb.tempC);
      logLine(line, 0x7FFF);
      // ヘッダにも簡易表示
      char hdr[64];
      snprintf(hdr, sizeof(hdr), "FB: pos %.2f rad | vel %.2f | T %.1fC", fb.angleRad, fb.velRadS, fb.tempC);
      drawHeader(hdr, (fb.faultBits ? RED : GREEN));
    }
    else
    {
      // その他のフレームは生で表示
      showRawFrame(f);
    }
  }

  // --- 各モードでの継続制御コマンド送信 ---
  if (millis() - lastOpMs >= 50)
  { // 50ms間隔で継続送信
    lastOpMs = millis();
    bool ok = false;

    switch (modeState)
    {
    case 0: // Position mode - 継続送信は不要（一度送信すればOK）
      break;

    case 1: // Velocity mode - 継続送信が必要
    {
      float velocity;
      switch (posState)
      {
      case 0:
        velocity = 5.0f; // 5 rad/s
        break;
      case 1:
        velocity = 0.0f; // 0 rad/s (停止)
        break;
      case 2:
        velocity = -5.0f; // -5 rad/s (逆回転)
        break;
      }
      ok = RS.velocityRef(MOTOR_ID, velocity);
      if (!ok)
      {
        logLine("Continuous velocityRef FAIL", RED);
      }
      else
      {
        // デバッグ用：時々継続制御の成功を表示
        static int debugCounter = 0;
        if (++debugCounter >= 20)
        { // 1秒に1回表示
          logLine(String("Continuous velocity: ") + String(velocity, 1) + " rad/s", GREEN);
          debugCounter = 0;
        }
      }
    }
    break;

    case 2: // Current mode - 継続送信が必要
      ok = RS.currentIqRef(MOTOR_ID, 2.0f);
      break;

    case 3: // CSP mode - 継続送信が必要
      ok = RS.cspLocRef(MOTOR_ID, 1.0f);
      break;
    }

    if (modeState > 0 && !ok)
    {
      logLine("Continuous control FAIL", RED);
    }
  }

  delay(5);
}
