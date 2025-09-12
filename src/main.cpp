// main.cpp — Velocity / PP / Current / CSP（robust）検証UI
// M5Unified + MCP2515(1Mbps) / mcp_can 4引数 readMsgBuf 版
//
// 操作：
//  左(A)  … ステータス取得（CAN_MASTER / RUN_MODE）
//  中(B)  … 選択モードの Bring-up & デモ指令
//  右(C)  … Modeを Velocity→PP→Current→CSP→… とローテーション
//
// 表示は毎回スプライト全描画（重なり/見切れなし）

#include <M5Unified.h>
#include <SPI.h>
#include <mcp_can.h>
#include <stdarg.h>
#include "RS02PrivateCAN.h"

// ===== CAN配線/設定 =====
#define CAN_CS_PIN 6
#define CAN_BAUD CAN_1000KBPS
#define MCP_CLOCK MCP_8MHZ // 16MHz基板なら MCP_16MHZ

// ===== ノード設定 =====
constexpr uint8_t HOST_ID = 0x00;
constexpr uint8_t MOTOR_ID = 0x7F;

MCP_CAN CAN(CAN_CS_PIN);
RS02PrivateCAN RS(CAN, HOST_ID);

// ===== UI =====
M5Canvas spr(&M5.Display);
const int W = 320, H = 240, PAD = 6;

enum class Mode : uint8_t
{
  Velocity = 0,
  PP = 1,
  Current = 2,
  CSP = 3
};
Mode curMode = Mode::Velocity;

static const char *modeName(Mode m)
{
  switch (m)
  {
  case Mode::Velocity:
    return "Velocity";
  case Mode::PP:
    return "PP";
  case Mode::Current:
    return "Current";
  case Mode::CSP:
    return "CSP";
  }
  return "?";
}

static void drawLayout()
{
  spr.fillScreen(BLACK);
  spr.setTextSize(1);
  spr.setTextColor(WHITE, BLACK);
  spr.setCursor(PAD, PAD);
  spr.printf("RS02 Multi-Mode Tester  HOST=0x%02X  MOTOR=0x%02X  MASTER=0x%02X\n",
             HOST_ID, MOTOR_ID, RS.masterId());
  spr.setCursor(PAD, PAD + 16);
  spr.printf("[A] Status  [B] Bring-up+Demo  [C] Mode=%s\n", modeName(curMode));
  spr.drawRect(2, 40, W - 4, 192, 0x7BEF);
}

static void printLine(int row, const char *fmt, ...)
{
  char buf[230];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  int y = 46 + row * 18;
  spr.fillRect(6, y - 2, W - 12, 18, BLACK);
  spr.setCursor(6, y);
  spr.setTextColor(WHITE, BLACK);
  spr.print(buf);
}

// ===== 小ヘルパ =====
static bool readU16(uint8_t node, uint16_t index, uint16_t &out)
{
  uint8_t le[4] = {0};
  if (!RS.readParamRaw(node, index, le))
    return false;
  out = (uint16_t)le[0] | ((uint16_t)le[1] << 8);
  return true;
}
static bool readU8(uint8_t node, uint16_t index, uint8_t &out)
{
  uint8_t le[4] = {0};
  if (!RS.readParamRaw(node, index, le))
    return false;
  out = le[0];
  return true;
}
static bool readF32(uint8_t node, uint16_t index, float &out)
{
  uint8_t le[4] = {0};
  if (!RS.readParamRaw(node, index, le))
    return false;
  memcpy(&out, le, 4);
  return true;
}
static bool writeU8(uint8_t node, uint16_t index, uint8_t v)
{
  uint8_t le[4] = {v, 0, 0, 0};
  return RS.writeParamLE(node, index, le);
}

// ===== A: Status =====
static void doStatus()
{
  drawLayout();
  uint16_t canMaster = 0xFFFF;
  uint8_t run = 0xFF;
  bool ok1 = readU16(MOTOR_ID, RS02Idx::CAN_MASTER, canMaster);
  bool ok2 = readU8(MOTOR_ID, RS02Idx::RUN_MODE, run);
  printLine(0, "CAN_MASTER(0x200B): %s 0x%04X", ok1 ? "OK" : "--", ok1 ? canMaster : 0);
  printLine(1, "RUN_MODE (0x7005) : %s %u", ok2 ? "OK" : "--", ok2 ? run : 255);
  spr.pushSprite(0, 0);
}

// ===== B: Bring-up + Demo =====
static void doVelocityDemo()
{
  drawLayout();
  int row = 0;
  const float LIMIT_CUR_A = 5.0f;
  const float ACC_RAD_S2 = 20.0f;
  const float SPD_REF = 2.0f;

  bool ok = RS.stop(MOTOR_ID, true);
  printLine(row++, "Stop+Clear: %s", ok ? "OK" : "NG");
  delay(100);
  ok &= writeU8(MOTOR_ID, RS02Idx::RUN_MODE, 2);
  printLine(row++, "RUN_MODE=2: %s", ok ? "OK" : "NG");
  delay(50);
  ok &= RS.enable(MOTOR_ID);
  printLine(row++, "Enable    : %s", ok ? "OK" : "NG");
  delay(50);
  ok &= RS.writeFloatParam(MOTOR_ID, RS02Idx::LIMIT_CUR, LIMIT_CUR_A);
  ok &= RS.writeFloatParam(MOTOR_ID, RS02Idx::LIMIT_CUR_OLD, LIMIT_CUR_A);
  ok &= RS.writeFloatParam(MOTOR_ID, RS02Idx::ACC_RAD, ACC_RAD_S2);
  printLine(row++, "Params limit_cur(7018/2019)+acc(7022): %s", ok ? "OK" : "NG");

  bool okSpd = true;
  for (int i = 0; i < 10; i++)
  {
    okSpd &= RS.writeFloatParam(MOTOR_ID, RS02Idx::SPD_REF, SPD_REF);
    delay(20);
  }
  printLine(row++, "spd_ref(700A)=%.1f x10 : %s", SPD_REF, okSpd ? "OK" : "NG");

  uint8_t rm = 0xFF;
  float vel = 0;
  readU8(MOTOR_ID, RS02Idx::RUN_MODE, rm);
  readF32(MOTOR_ID, RS02Idx::MECH_VEL, vel);
  printLine(row++, "Read-back RUN_MODE=%u  mechVel=%.2f", rm, vel);
  spr.pushSprite(0, 0);
}

static void doPPDemo()
{
  drawLayout();
  int row = 0;
  const float LIMIT_SPD = 3.0f; // rad/s
  const float LOC_STEP = 1.57f; // 90deg

  bool ok = RS.stop(MOTOR_ID, true);
  printLine(row++, "Stop+Clear: %s", ok ? "OK" : "NG");
  delay(100);
  ok &= writeU8(MOTOR_ID, RS02Idx::RUN_MODE, 1);
  printLine(row++, "RUN_MODE=1(PP): %s", ok ? "OK" : "NG");
  delay(50);
  ok &= RS.enable(MOTOR_ID);
  printLine(row++, "Enable         : %s", ok ? "OK" : "NG");
  delay(50);
  ok &= RS.writeFloatParam(MOTOR_ID, RS02Idx::LIMIT_SPD, LIMIT_SPD);
  printLine(row++, "limit_spd(7017)=%.1f : %s", LIMIT_SPD, ok ? "OK" : "NG");
  ok &= RS.writeFloatParam(MOTOR_ID, RS02Idx::LOC_REF, LOC_STEP);
  delay(150);
  ok &= RS.writeFloatParam(MOTOR_ID, RS02Idx::LOC_REF, 0.0f);
  printLine(row++, "loc_ref step: ->%.2f ->0.00 : %s", LOC_STEP, ok ? "OK" : "NG");

  uint8_t rm = 0xFF;
  readU8(MOTOR_ID, RS02Idx::RUN_MODE, rm);
  printLine(row++, "Read-back RUN_MODE=%u", rm);
  spr.pushSprite(0, 0);
}

static void doCurrentDemo()
{
  drawLayout();
  int row = 0;
  const float LIMIT_TORQUE = 3.0f; // Nm（環境に応じて）
  const float IQ_REF = 1.0f;       // A  （FWによりIndex差あり）

  bool ok = RS.stop(MOTOR_ID, true);
  printLine(row++, "Stop+Clear: %s", ok ? "OK" : "NG");
  delay(100);
  ok &= writeU8(MOTOR_ID, RS02Idx::RUN_MODE, 3);
  printLine(row++, "RUN_MODE=3(Current): %s", ok ? "OK" : "NG");
  delay(50);
  ok &= RS.enable(MOTOR_ID);
  printLine(row++, "Enable             : %s", ok ? "OK" : "NG");
  delay(50);
  ok &= RS.writeFloatParam(MOTOR_ID, RS02Idx::LIMIT_TORQUE, LIMIT_TORQUE);
  printLine(row++, "limit_torque(700B)=%.1f : %s", LIMIT_TORQUE, ok ? "OK" : "NG");
  ok &= RS.writeFloatParam(MOTOR_ID, RS02Idx::IQ_REF, IQ_REF);
  printLine(row++, "iq_ref(700C)=%.2f : %s", IQ_REF, ok ? "OK" : "NG");

  uint8_t rm = 0xFF;
  readU8(MOTOR_ID, RS02Idx::RUN_MODE, rm);
  printLine(row++, "Read-back RUN_MODE=%u", rm);
  spr.pushSprite(0, 0);
}

static void doCSPDemo()
{
  drawLayout();
  int row = 0;
  const float LIMIT_SPD = 6.0f;      // rad/s
  const float LIMIT_CUR = 5.0f;      // A
  const float KP_LOC = 5.0f;         // 任意（必要な個体向け）
  const float P1 = 1.57f, P2 = 0.0f; // 90deg -> 0

  // ★ robust版：Enable前後でrun_mode=5を書き・前提パラメータを両系で適用
  bool ok = RS.enterCSP_robust(MOTOR_ID, LIMIT_SPD, LIMIT_CUR, KP_LOC);
  printLine(row++, "CSP robust (RM=5 twice + limits + KP): %s", ok ? "OK" : "NG");

  // 読み戻し
  uint8_t rm = 0xFF;
  RS.readRunMode(MOTOR_ID, rm);
  printLine(row++, "Read-back RUN_MODE=%u", rm);

  // 位置指令
  bool okRef = true;
  okRef &= RS.cspLocRef(MOTOR_ID, P1);
  delay(180);
  okRef &= RS.cspLocRef(MOTOR_ID, P2);
  printLine(row++, "loc_ref(7016): 1.57 -> 0.00 : %s", okRef ? "OK" : "NG");

  spr.pushSprite(0, 0);
}

// ===== Arduino lifecycle =====
void setup()
{
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setTextWrap(false, false);

  spr.setColorDepth(8);
  spr.createSprite(W, H);
  spr.setTextWrap(false, false);
  spr.setTextSize(1);

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
  CAN.setMode(MCP_NORMAL); // 念のためNORMAL
  RS.begin();
  RS.setMasterId(0xFD); // ★ 参考実機に合う既定

  drawLayout();
  printLine(0, "READY. Mode=%s  (A:Status / B:Demo / C:Next)", modeName(curMode));
  spr.pushSprite(0, 0);
}

void loop()
{
  M5.update();

  if (M5.BtnA.wasPressed())
  {
    doStatus();
  }
  if (M5.BtnB.wasPressed())
  {
    switch (curMode)
    {
    case Mode::Velocity:
      doVelocityDemo();
      break;
    case Mode::PP:
      doPPDemo();
      break;
    case Mode::Current:
      doCurrentDemo();
      break;
    case Mode::CSP:
      doCSPDemo();
      break;
    }
  }
  if (M5.BtnC.wasPressed())
  {
    curMode = static_cast<Mode>((static_cast<uint8_t>(curMode) + 1) % 4);
    drawLayout();
    printLine(0, "Mode changed -> %s", modeName(curMode));
    spr.pushSprite(0, 0);
  }

  delay(5);
}
