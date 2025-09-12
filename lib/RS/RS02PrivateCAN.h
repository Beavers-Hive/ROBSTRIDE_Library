#pragma once
// RS02PrivateCAN.h — FD00/LE/応答dst拡張（host,0x00,0xFF,0xFE, targetId も許可）
// 依存: Arduino, mcp_can (Cory Fowler系 / 4引数 readMsgBuf)

#include <Arduino.h>
#include <mcp_can.h>
#include <stdint.h>

namespace RS02Idx
{
    // ランモード/制御
    static constexpr uint16_t RUN_MODE = 0x7005; // u8: 0=Operation,1=PP,2=Velocity,3=Current,5=CSP
    // 指令/上限
    static constexpr uint16_t SPD_REF = 0x700A;      // f32: rad/s
    static constexpr uint16_t LIMIT_TORQUE = 0x700B; // f32: Nm
    static constexpr uint16_t IQ_REF = 0x7006;       // f32: A（Currentのq軸電流）
    static constexpr uint16_t LOC_REF = 0x7016;      // f32: rad
    static constexpr uint16_t LIMIT_SPD = 0x7017;    // f32: rad/s
    static constexpr uint16_t LIMIT_CUR = 0x7018;    // f32: A（新系）
    // センサ実測（TWAI動作実績に合わせる）
    static constexpr uint16_t MECH_POS = 0x7019; // f32: 機械角 [rad]
    static constexpr uint16_t MECH_VEL = 0x701B; // f32: 角速度 [rad/s]
    // ゲイン
    static constexpr uint16_t SPD_KP = 0x701C;  // f32
    static constexpr uint16_t SPD_KI = 0x701D;  // f32
    static constexpr uint16_t LOC_KP = 0x701E;  // f32
    static constexpr uint16_t ACC_RAD = 0x7022; // f32: rad/s^2
    static constexpr uint16_t CUR_KP = 0x7010;  // f32
    static constexpr uint16_t CUR_KI = 0x7011;  // f32
    // 旧系（個体差対策）
    static constexpr uint16_t LIMIT_CUR_OLD = 0x2019; // f32: A
    // 診断
    static constexpr uint16_t CAN_MASTER = 0x200B; // u16
    // （備考）一部FWで死んでいることがある:
    static constexpr uint16_t IDX_ROTATION = 0x3014;       // f32
    static constexpr uint16_t IDX_MODPOS = 0x3015;         // f32
    static constexpr uint16_t IDX_MECH_ANGLE_ROT = 0x3036; // f32
    static constexpr uint16_t IDX_EPSCAN_TIME = 0x5001;    // u16 (仮)
}

struct RS02PrivFrame
{
    unsigned long id = 0;
    uint8_t dlc = 0;
    uint8_t data[8] = {0};
    bool isExt = false;
};

struct RS02Feedback
{
    uint8_t motorId = 0;
    uint16_t faultBits = 0;
    uint8_t mode = 0;
    float angleRad = 0.0f;
    float velRadS = 0.0f;
    float torqueNm = 0.0f;
    float tempC = 0.0f;
};

class RS02PrivateCAN
{
public:
    RS02PrivateCAN(MCP_CAN &can, uint8_t hostId) : _can(can), _hostId(hostId) {}

    bool begin();
    void setMasterId(uint8_t mid) { _masterId = mid; } // 既定=0xFD
    uint8_t masterId() const { return _masterId; }
    uint8_t hostId() const { return _hostId; }

    // Motor CAN ID change (Type7: immediate)
    bool setMotorId(uint8_t currentId, uint8_t newId);

    // Motor CAN ID change via param 0x200A (Type18) + optional save (Type22)
    bool setMotorIdViaParam(uint8_t targetId, uint8_t newId, bool save);

    // Save all parameters (Type22)
    bool saveParams(uint8_t targetId);

    // 低レベル
    bool sendExt(unsigned long id, const uint8_t *payload, uint8_t len);
    bool readAny(RS02PrivFrame &out);

    // 基本コマンド
    bool ping(uint8_t targetId);                  // Type0
    bool enable(uint8_t targetId);                // Type3
    bool stop(uint8_t targetId, bool clearFault); // Type4

    // パラメータR/W（index=LE）
    bool writeParamLE(uint8_t targetId, uint16_t index, const uint8_t valueLE[4]);
    bool writeFloatParam(uint8_t targetId, uint16_t index, float value);
    bool readParamRaw(uint8_t targetId, uint16_t index, uint8_t out4LE[4]);
    bool readFloatParam(uint8_t targetId, uint16_t index, float &out);

    // プロトコル/レポート
    bool switchProtocol(uint8_t targetId, uint8_t fcmd); // Type25
    bool setActiveReport(uint8_t targetId, bool enable); // Type24
    bool setReportIntervalTicks(uint8_t targetId, uint16_t ticks);

    // Operation Control（Type1のみDA2=トルク）
    bool opControl(uint8_t targetId, float torqueNm, float posRad, float velRadS, float kp, float kd);

    // 受信解析 Type2
    bool parseFeedback(const RS02PrivFrame &f, RS02Feedback &out);

    // ランモード
    bool setRunMode(uint8_t targetId, uint8_t runMode);
    bool readRunMode(uint8_t targetId, uint8_t &outMode);

    // ===== Velocity / PP / Current / CSP =====
    bool enterVelocity(uint8_t targetId, float limitCurA, float accRadS2, float spdKp = NAN, float spdKi = NAN);
    bool velocityRef(uint8_t targetId, float spdRadS);
    bool enterVelocityStrict(uint8_t targetId, float limitTorqueNm, float limitCurA, float accRadS2, float spdKp = NAN, float spdKi = NAN);
    bool bringUpVelocityPerSpec(uint8_t targetId, float limitCurA, float accRadS2, float spdRadS);

    bool enterPP(uint8_t targetId, float limitSpdRadS, float locKp = NAN);
    bool ppLocRef(uint8_t targetId, float posRad);
    bool bringUpPPPerSpec(uint8_t targetId, float limitSpdRadS, float posRad);

    bool enterCurrent(uint8_t targetId, float limitTorqueNm, float curKp = NAN, float curKi = NAN);
    bool currentIqRef(uint8_t targetId, float iqA);
    bool bringUpCurrentPerSpec(uint8_t targetId, float iqA);

    bool enterCSP(uint8_t targetId, float limitSpdRadS, float limitCurA, float locKp = NAN);
    bool cspLocRef(uint8_t targetId, float posRad);
    bool bringUpCSPPerSpec(uint8_t targetId, float limitSpdRadS, float posRad);
    bool enterCSP_simple(uint8_t targetId, float limitSpdRadS);
    bool enterCSP_robust(uint8_t targetId, float limitSpdRadS, float limitCurA, float locKp = NAN);

    // 無限回転（パラメータ合成: FWにより未更新の個体もある）
    bool getInfiniteByParams(uint8_t targetId, double &turns, double &angleRad);

private:
    MCP_CAN &_can;
    uint8_t _hostId = 0x00;
    uint8_t _masterId = 0xFD;

    inline uint16_t da2_master() const { return ((uint16_t)_masterId << 8) | 0x00; }
    static inline uint32_t buildExId(uint8_t type5, uint16_t da2, uint8_t dst)
    {
        return ((uint32_t)(type5 & 0x1F) << 24) | ((uint32_t)da2 << 8) | (uint32_t)dst;
    }

    // パック/演算
    static inline void packU16BE(uint16_t v, uint8_t *p)
    {
        p[0] = (uint8_t)(v >> 8);
        p[1] = (uint8_t)(v);
    }
    static inline uint16_t unpackU16BE(const uint8_t *p) { return ((uint16_t)p[0] << 8) | p[1]; }
    static inline void packF32LE(float f, uint8_t *p)
    {
        uint32_t u;
        memcpy(&u, &f, 4);
        p[0] = u;
        p[1] = u >> 8;
        p[2] = u >> 16;
        p[3] = u >> 24;
    }

    static inline uint16_t float_to_uint(float x, float x_min, float x_max)
    {
        float cl = (x < x_min) ? x_min : (x > x_max ? x_max : x);
        return (uint16_t)((cl - x_min) * 65535.0f / (x_max - x_min));
    }
    static inline float uint_to_float(uint16_t x, float x_min, float x_max)
    {
        return ((float)x) * (x_max - x_min) / 65535.0f + x_min;
    }
};
