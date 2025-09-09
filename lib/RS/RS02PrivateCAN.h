#pragma once
#include <Arduino.h>
#include <mcp_can.h>

struct RS02PrivFrame
{
    unsigned long id = 0; // 29bit拡張ID
    byte dlc = 0;
    byte data[8] = {0};
    bool isExt = true;
};

// ===== パラメータインデックスの定数（PDF準拠） =====
struct RS02Idx
{
    static constexpr uint16_t RUN_MODE = 0x7005;     // 0:Operation,1:PP,2:Velocity,3:Current,5:CSP
    static constexpr uint16_t IQ_REF = 0x7006;       // Current mode Iq command [A]
    static constexpr uint16_t SPD_REF = 0x700A;      // Velocity mode speed ref [rad/s]
    static constexpr uint16_t LIMIT_TORQUE = 0x700B; // Torque limit [Nm]
    static constexpr uint16_t LOC_REF = 0x7016;      // CSP position ref [rad]
    static constexpr uint16_t LIMIT_SPD = 0x7017;    // CSP speed limit [rad/s]
    static constexpr uint16_t LIMIT_CUR = 0x7018;    // Position/Velocity current limit [A]
    static constexpr uint16_t CUR_KP = 0x7010;       // Current loop Kp
    static constexpr uint16_t CUR_KI = 0x7011;       // Current loop Ki
    static constexpr uint16_t LOC_KP = 0x701E;       // Position loop Kp
    static constexpr uint16_t SPD_KP = 0x701F;       // Speed loop Kp
    static constexpr uint16_t SPD_KI = 0x7020;       // Speed loop Ki
    static constexpr uint16_t ACC_RAD = 0x7022;      // Velocity mode acceleration [rad/s^2]
};

class RS02PrivateCAN
{
public:
    explicit RS02PrivateCAN(MCP_CAN &can, uint8_t hostId = 0xFD)
        : _can(can), _hostId(hostId) {}

    bool begin();

    // 29bitID: Bit[28:24]=通信タイプ, Bit[23:8]=DataArea2(用途可変), Bit[7:0]=Destination(motor id)
    static inline unsigned long buildExId(uint8_t type, uint16_t data16, uint8_t targetId)
    {
        return ((unsigned long)(type & 0x1F) << 24) | ((unsigned long)data16 << 8) | (unsigned long)targetId;
    }

    bool sendExt(unsigned long id, const byte *payload, byte len);

    bool readAny(RS02PrivFrame &out);

    // 便利ヘルパ
    static inline void packF32LE(float v, byte *d)
    {
        uint32_t u;
        memcpy(&u, &v, 4);
        d[0] = u & 0xFF;
        d[1] = (u >> 8) & 0xFF;
        d[2] = (u >> 16) & 0xFF;
        d[3] = (u >> 24) & 0xFF;
    }
    static inline uint16_t float_to_uint(float x, float x_min, float x_max)
    {
        float span = x_max - x_min;
        if (x > x_max)
            x = x_max;
        else if (x < x_min)
            x = x_min;
        return (uint16_t)((x - x_min) * 65535.0f / span + 0.5f);
    }
    static inline float uint_to_float(uint16_t u, float x_min, float x_max)
    {
        float span = x_max - x_min;
        return (float)u * span / 65535.0f + x_min;
    }
    static inline void packU16BE(uint16_t v, byte *d)
    {
        d[0] = (byte)(v >> 8);
        d[1] = (byte)(v & 0xFF);
    }
    static inline uint16_t unpackU16BE(const byte *d) { return (uint16_t)((d[0] << 8) | d[1]); }

    // ====== Privateプロトコルの主な操作 ======

    // 通信タイプ0: Get device ID（Ping）
    bool ping(uint8_t targetId);

    // 通信タイプ3: Enable
    bool enable(uint8_t targetId);

    // 通信タイプ4: Stop / Fault Clear（Byte0..1=1でクリア）
    bool stop(uint8_t targetId, bool clearFault = false);

    // 通信タイプ18(0x12): Write single parameter（index(2B BE), value(4B LE)）
    bool writeParamLE(uint8_t targetId, uint16_t index, const byte valueLE[4]);
    bool writeFloatParam(uint8_t targetId, uint16_t index, float value);

    // 通信タイプ25(0x19): Protocol switch（0:Private,1:CANopen,2:MIT）※参考
    bool switchProtocol(uint8_t targetId, uint8_t fcmd);

    // 通信タイプ1: Operation Control（pos/vel/Kp/Kd は data部 16bitBE、torque はID側DataArea2）
    bool opControl(uint8_t targetId, float torqueNm, float posRad, float velRadS, float kp, float kd);

    // フィードバック（通信タイプ2）の簡易デコード
    struct Feedback
    {
        uint8_t motorId = 0;
        uint16_t faultBits = 0;
        uint8_t mode = 0;
        float angleRad = 0, velRadS = 0, torqueNm = 0, tempC = 0;
    };
    bool parseFeedback(const RS02PrivFrame &f, Feedback &out);

    // 走行モード（0=Operation,1=PP,2:Velocity,3:Current,5:CSP）: パラメータ 0x7005
    bool setRunMode(uint8_t targetId, uint8_t runMode);

    // ===== Velocity / Current / CSP 用ユーティリティ =====

    // --- Velocity mode ---
    // 推奨フロー: stop(clear) → enable() → enterVelocity() → velocityRef() を周期送信(50〜100ms)
    bool enterVelocity(uint8_t targetId,
                       float limitCurA = 10.0f, // 0..23 A
                       float accRadS2 = 20.0f,  // 既定 20 rad/s^2
                       float spdKp = NAN,       // 任意: 既定 6
                       float spdKi = NAN);      // 任意: 既定 0.02

    // 速度指令（rad/s）※50〜100ms 間隔で連続送信推奨
    bool velocityRef(uint8_t targetId, float spdRadS);

    // Velocityモードの"決め打ちBring-up"手順（PDF順序に厳密）
    bool enterVelocityStrict(uint8_t targetId,
                             float limitTorqueNm, // 例: 6〜10
                             float limitCurA,     // 例: 8〜12
                             float accRadS2,      // 例: 10〜30
                             float spdKp = NAN,   // 任意
                             float spdKi = NAN);  // 任意

    // --- Current mode ---
    // 推奨フロー: stop(clear) → enable() → enterCurrent() → currentIqRef() を周期送信(20〜50ms)
    bool enterCurrent(uint8_t targetId,
                      float limitTorqueNm = 10.0f, // 0..17 Nm
                      float curKp = NAN,           // 任意: 既定 0.17
                      float curKi = NAN);          // 任意: 既定 0.012

    // 電流指令（Iq [A]）。モータ仕様の定格内で。
    bool currentIqRef(uint8_t targetId, float iqA);

    // --- CSP (Cyclic Synchronous Position) ---
    // 推奨フロー: stop(clear) → enable() → enterCSP() → cspLocRef() を必要に応じて送信
    bool enterCSP(uint8_t targetId,
                  float limitSpdRadS = 10.0f, // 0..44 rad/s
                  float limitCurA = 10.0f,    // 0..23 A
                  float locKp = NAN);         // 任意: 既定 40

    // 位置指令（rad）。必要なら一定周期で再発行（例: 50〜100ms）
    bool cspLocRef(uint8_t targetId, float posRad);

private:
    MCP_CAN &_can;
    uint8_t _hostId;
};
