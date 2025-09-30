// main.cpp — mechPos(0x7019)/mechVel(0x701B) をType17で周期読出し→Angle∞をアンラップ表示
// M5Unified + MCP2515(1Mbps) / mcp_can 4引数 readMsgBuf 版

#include <M5Unified.h>
#include <SPI.h>
#ifdef USE_TWAI
#include "RS02PrivateTWAI.h"
#else
#include <mcp_can.h>
#include "RS02PrivateCAN.h"
#endif
#include <stdarg.h>
#include <math.h>

#ifndef USE_TWAI
#define CAN_CS_PIN 6
#define CAN_BAUD CAN_1000KBPS
#define MCP_CLOCK MCP_8MHZ // 基板に合わせて 16MHzなら MCP_16MHZ
#endif

constexpr uint8_t HOST_ID = 0x00;
constexpr uint8_t MOTOR_ID = 0x7E;

#ifdef USE_TWAI
// M5AtomS3 default TWAI pins: GPIO2=TX, GPIO1=RX（ビルドフラグで上書き可）
#ifndef TWAI_TX_GPIO
#define TWAI_TX_GPIO 2
#endif
#ifndef TWAI_RX_GPIO
#define TWAI_RX_GPIO 1
#endif
RS02PrivateTWAI RS(HOST_ID, TWAI_TX_GPIO, TWAI_RX_GPIO);
#else
MCP_CAN CAN(CAN_CS_PIN);
RS02PrivateCAN RS(CAN, HOST_ID);
#endif

// ===== Minimal (no UI) =====

// No mode switching; IMU-CSP only

// ===== Angle∞ アンラップ =====
struct AngleTracker
{
  bool has = false;
  float prev = NAN;    // 直近の 0x7019 値（ラップ角）
  double accRad = 0.0; // 累積角 [rad]
} gAngle;

static inline void angleTrackUpdateFromMechPos(float nowRad)
{
  if (!isfinite(nowRad))
    return;
  const float WRAP = 2.0f * (float)M_PI; // mechPos は 2π周期想定
  if (!gAngle.has)
  {
    gAngle.has = true;
    gAngle.prev = nowRad;
    gAngle.accRad = nowRad;
    return;
  }
  float d = nowRad - gAngle.prev;
  // 最小差分へ折り返し（πが境目）
  if (d > (float)M_PI)
    d -= WRAP;
  if (d < -(float)M_PI)
    d += WRAP;
  gAngle.accRad += (double)d;
  gAngle.prev = nowRad;
}

// ===== 表示ユーティリティ =====
// (UI utilities removed)
static bool readU8(uint8_t node, uint16_t idx, uint8_t &out)
{
  uint8_t le[4] = {0};
  if (!RS.readParamRaw(node, idx, le))
    return false;
  out = le[0];
  return true;
}
static bool readF32(uint8_t node, uint16_t idx, float &out)
{
  uint8_t le[4] = {0};
  if (!RS.readParamRaw(node, idx, le))
    return false;
  memcpy(&out, le, 4);
  return true;
}
static bool writeU8(uint8_t node, uint16_t idx, uint8_t v)
{
  uint8_t le[4] = {v, 0, 0, 0};
  return RS.writeParamLE(node, idx, le);
}

// ===== Monitor（200ms周期で 0x7019/0x701B をType17読出し）=====
// monitor removed

// ===== Demo（B）=====
// (Demo functions removed)

// ===== IMU-based CSP Control (Yaw stabilization) =====
static bool gImuReady = false;
static bool gImuCspActive = false;
static uint32_t gImuLastTickMs = 0;
static double gYawRad = 0.0;          // integrated yaw [rad]
static double gYawRateRadS = 0.0;     // gyro Z [rad/s]
static double gCspTargetPosRad = 0.0; // motor absolute target [rad]

static void imuInit()
{
  // Try initialize IMU; M5Unified exposes M5.Imu
  // begin() が無い環境でも多くは M5.begin で初期化済みのため、存在チェックのみ
  // ここでは getGyro が成功するかで判定する
  float gx = 0, gy = 0, gz = 0;
  M5.Imu.getGyro(&gx, &gy, &gz); // 多くの環境でvoid
  gImuReady = true;
}

static bool startImuCsp()
{
  int row = 0;
  const float LIMIT_SPD = 6.0f; // rad/s
  const float LIMIT_CUR = 5.0f; // A
  const float KP_LOC = 50.0f;

  if (!gImuReady)
    imuInit();

  bool ok = RS.enterCSP_robust(MOTOR_ID, LIMIT_SPD, LIMIT_CUR, KP_LOC);
  Serial.printf("IMU-CSP bring-up: %s\n", ok ? "OK" : "NG");

  // 初期目標を現在位置へ
  float posNow = 0.0f;
  if (ok && RS.readFloatParam(MOTOR_ID, RS02Idx::MECH_POS, posNow))
  {
    gCspTargetPosRad = posNow;
  }
  else
  {
    gCspTargetPosRad = 0.0;
  }
  gYawRad = 0.0;
  gYawRateRadS = 0.0;
  gImuLastTickMs = millis();
  gImuCspActive = ok;
  Serial.printf("Init pos=%.2f rad  IMU=%s\n", (float)gCspTargetPosRad, gImuReady ? "OK" : "NG");
  return gImuCspActive;
}

static void stopImuCsp()
{
  gImuCspActive = false;
}

static void imuCspTick()
{
  if (!gImuCspActive)
    return;

  uint32_t now = millis();
  float dt = (now - gImuLastTickMs) * 0.001f;
  if (dt <= 0.0f || dt > 0.1f)
  {
    gImuLastTickMs = now;
    return;
  }
  gImuLastTickMs = now;

  // 1) センサ取得（Z軸ジャイロをYaw速度とみなす）
  float gx = 0, gy = 0, gz = 0;
  if (gImuReady)
  {
    M5.Imu.getGyro(&gx, &gy, &gz);
    // M5Unifiedはdeg/sのことが多い
    gYawRateRadS = (double)gz * ((double)M_PI / 180.0);
    gYawRad += gYawRateRadS * (double)dt;
  }

  // 2) 制御（PD: u = -Kp*yaw - Kd*dyaw）→ 速度 [rad/s]
  const double Kp = 2.0; // 調整用
  const double Kd = 0.05;
  double uVel = -(Kp * gYawRad + Kd * gYawRateRadS);

  // 3) 速度上限 → 位置へ積分
  const double VEL_LIMIT = 5.0; // rad/s（motor LIMIT_SPD より少し低め）
  if (uVel > VEL_LIMIT)
    uVel = VEL_LIMIT;
  if (uVel < -VEL_LIMIT)
    uVel = -VEL_LIMIT;
  gCspTargetPosRad += uVel * (double)dt;

  // 4) 位置コマンド送出
  (void)RS.cspLocRef(MOTOR_ID, (float)gCspTargetPosRad);

  // 5) ログ（任意）
  // Serial.printf("yaw=%.3f rad  dy=%.3f rad/s  tgt=%.2f\n", (float)gYawRad, (float)gYawRateRadS, (float)gCspTargetPosRad);
}

// ===== Minimal display (small font) =====
static uint32_t gNextUiMs = 0;

static void updateDisplayTick()
{
  if ((int32_t)(millis() - gNextUiMs) < 0)
    return;
  gNextUiMs = millis() + 100; // 10Hz

  M5.Display.fillScreen(BLACK);
  M5.Display.setCursor(2, 2);
  M5.Display.setTextColor(WHITE, BLACK);
  M5.Display.setTextSize(1); // smallest

  float yawRad = (float)gYawRad;
  float yawDeg = yawRad * (180.0f / (float)M_PI);
  float dy = (float)gYawRateRadS;
  float tgt = (float)gCspTargetPosRad;

  M5.Display.println("IMU-CSP");
  M5.Display.printf("yaw=%.2f rad (%.0f°)\n", yawRad, yawDeg);
  M5.Display.printf("dy=%.2f rs\n", dy);
  // M5.Display.printf("tgt=%.2f v=%.2f\n", tgt, gLastVelCmd);
}

// ===== Arduino lifecycle =====
void setup()
{
  auto cfg = M5.config();
  M5.begin(cfg);
  M5.Display.setBrightness(128);
  M5.Display.setTextWrap(false, false);
  M5.Display.setTextSize(1);
  Serial.begin(115200);
  delay(50);
  Serial.println("[BOOT] IMU-CSP pendulum");

  // SPIはMCP2515使用時のみ必要
#ifndef USE_TWAI
  SPI.begin();
#endif

#ifdef USE_TWAI
  Serial.printf("[TWAI] init: TX=%d RX=%d\n", (int)TWAI_TX_GPIO, (int)TWAI_RX_GPIO);
  if (!RS.begin())
  {
    Serial.println("[TWAI] begin FAIL");
    while (1)
      delay(1000);
  }
  Serial.println("[TWAI] begin OK");
#else
  if (CAN.begin(MCP_ANY, CAN_BAUD, MCP_CLOCK) != CAN_OK)
  {
    Serial.println("CAN.begin FAIL");
    while (1)
      delay(1000);
  }
  CAN.setMode(MCP_NORMAL);
  RS.begin();
#endif
  RS.setMasterId(0xFD);

  // IMUとCSPを起動
  imuInit();
  startImuCsp();
}

void loop()
{
  M5.update();
  imuCspTick();
  updateDisplayTick();
  delay(1);
}
