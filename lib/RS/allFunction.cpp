// main.cpp — mechPos(0x7019)/mechVel(0x701B) をType17で周期読出し→Angle∞をアンラップ表示
// M5Unified + MCP2515(1Mbps) / mcp_can 4引数 readMsgBuf 版

#include <M5Unified.h>
#include <SPI.h>
#include <mcp_can.h>
#include <stdarg.h>
#include <math.h>
#include "RS02PrivateCAN.h"

#define CAN_CS_PIN 6
#define CAN_BAUD CAN_1000KBPS
#define MCP_CLOCK MCP_8MHZ // 基板に合わせて 16MHzなら MCP_16MHZ

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
bool monitorOn = true;
uint32_t nextMonUpdate = 0;

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
    spr.printf("RS02 Monitor  HOST=0x%02X  MOTOR=0x%02X  MASTER=0x%02X\n",
               HOST_ID, MOTOR_ID, RS.masterId());
    spr.setCursor(PAD, PAD + 16);
    spr.printf("[A] Monitor %s  [B] Demo  [C] Mode=%s\n", monitorOn ? "ON" : "OFF", modeName(curMode));
    spr.drawRect(2, 40, W - 4, 192, 0x7BEF);
}
static void printLine(int row, const char *fmt, ...)
{
    char buf[240];
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
static void drawMonitorFrame()
{
    drawLayout();
    printLine(0, "MONITOR — mechPos(0x7019)/mechVel(0x701B) polling");
    spr.pushSprite(0, 0);
}
static void monitorTick()
{
    if (!monitorOn)
        return;

    uint8_t run = 255;
    readU8(MOTOR_ID, RS02Idx::RUN_MODE, run);

    float pos = 0.0f, vel = 0.0f, spdRef = 0.0f, locRef = 0.0f, iqRef = 0.0f;
    bool okPos = readF32(MOTOR_ID, RS02Idx::MECH_POS, pos);
    bool okVel = readF32(MOTOR_ID, RS02Idx::MECH_VEL, vel);
    (void)readF32(MOTOR_ID, RS02Idx::SPD_REF, spdRef);
    (void)readF32(MOTOR_ID, RS02Idx::LOC_REF, locRef);
    (void)readF32(MOTOR_ID, RS02Idx::IQ_REF, iqRef);

    if (okPos)
        angleTrackUpdateFromMechPos(pos);

    float limSpd = 0, limCur = 0, limCurOld = 0, limTq = 0, acc = 0;
    (void)readF32(MOTOR_ID, RS02Idx::LIMIT_SPD, limSpd);
    (void)readF32(MOTOR_ID, RS02Idx::LIMIT_CUR, limCur);
    (void)readF32(MOTOR_ID, RS02Idx::LIMIT_CUR_OLD, limCurOld);
    (void)readF32(MOTOR_ID, RS02Idx::LIMIT_TORQUE, limTq);
    (void)readF32(MOTOR_ID, RS02Idx::ACC_RAD, acc);

    printLine(1, "Mode=%u(%s)  Vel=%.3f%s rad/s",
              run, modeName(curMode), vel, okVel ? "" : "?");

    if (gAngle.has)
    {
        double turns = gAngle.accRad / (2.0 * M_PI);
        double deg = gAngle.accRad * (180.0 / M_PI);
        printLine(2, "Angle∞: pos=%.3f%s  ->  turns=%.1f  rad=%.3f  deg=%.1f",
                  pos, okPos ? "" : "?", turns, gAngle.accRad, deg);
    }
    else
    {
        printLine(2, "Angle∞: -- (waiting mechPos 0x7019)");
    }

    printLine(3, "Refs : loc=%.3f  spd=%.3f  iq=%.3f", locRef, spdRef, iqRef);
    printLine(4, "Limit: I=%.1f IO=%.1f T=%.1f S=%.1f  acc=%.1f", limCur, limCurOld, limTq, limSpd, acc);

    spr.pushSprite(0, 0);
}

// ===== Demo（B）=====
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
    printLine(row++, "limit_cur(7018/2019)+acc(7022): %s", ok ? "OK" : "NG");

    bool okSpd = true;
    for (int i = 0; i < 10; i++)
    {
        okSpd &= RS.writeFloatParam(MOTOR_ID, RS02Idx::SPD_REF, SPD_REF);
        delay(20);
    }
    printLine(row++, "spd_ref(700A)=%.1f x10 : %s", SPD_REF, okSpd ? "OK" : "NG");
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
    spr.pushSprite(0, 0);
}
static void doCurrentDemo()
{
    drawLayout();
    int row = 0;
    const float LIMIT_TORQUE = 3.0f; // Nm
    const float IQ = 0.5f;           // A

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
    ok &= RS.writeFloatParam(MOTOR_ID, RS02Idx::CUR_KP, 0.17f);
    ok &= RS.writeFloatParam(MOTOR_ID, RS02Idx::CUR_KI, 0.012f);
    printLine(row++, "limits/gains: tq=%.1f, cKp=0.17, cKi=0.012 -> %s", LIMIT_TORQUE, ok ? "OK" : "NG");

    ok &= RS.currentIqRef(MOTOR_ID, +IQ);
    delay(300);
    ok &= RS.currentIqRef(MOTOR_ID, 0.0f);
    delay(150);
    ok &= RS.currentIqRef(MOTOR_ID, -IQ);
    delay(300);
    ok &= RS.currentIqRef(MOTOR_ID, 0.0f);
    printLine(row++, "iq_ref sweep ±%.2f A : %s", IQ, ok ? "OK" : "NG");
    spr.pushSprite(0, 0);
}
static void doCSPDemo()
{
    drawLayout();
    int row = 0;
    const float LIMIT_SPD = 6.0f; // rad/s
    const float LIMIT_CUR = 5.0f; // A
    const float KP_LOC = 5.0f;
    const float P1 = 1.57f, P2 = 0.0f;

    bool ok = RS.enterCSP_robust(MOTOR_ID, LIMIT_SPD, LIMIT_CUR, KP_LOC);
    printLine(row++, "CSP robust (RM=5 twice + limits + KP): %s", ok ? "OK" : "NG");
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
    CAN.setMode(MCP_NORMAL);
    RS.begin();
    RS.setMasterId(0xFD);

    // 任意：Type2を有効化（出ない個体もあるが無害）
    RS.setActiveReport(MOTOR_ID, true);
    RS.setReportIntervalTicks(MOTOR_ID, 1);

    drawLayout();
    printLine(0, "READY. Mode=%s  (A:Monitor / B:Demo / C:Next)", modeName(curMode));
    spr.pushSprite(0, 0);
}

void loop()
{
    M5.update();

    if (M5.BtnA.wasPressed())
    {
        monitorOn = !monitorOn;
        drawMonitorFrame();
        if (monitorOn)
            nextMonUpdate = 0;
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

    // Monitor: 200ms周期で Type17 読み
    if (monitorOn && (int32_t)(millis() - nextMonUpdate) >= 0)
    {
        monitorTick();
        nextMonUpdate = millis() + 200;
    }

    delay(3);
}
