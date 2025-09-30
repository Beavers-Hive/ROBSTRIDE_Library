// RS02PrivateTWAI.cpp — ESP32 TWAI(内蔵CAN)向け 実装
#include "RS02PrivateTWAI.h"
#include <math.h>

bool RS02PrivateTWAI::begin()
{
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)_txPin, (gpio_num_t)_rxPin, TWAI_MODE_NORMAL);
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();
    if (twai_driver_install(&g_config, &_timing, &f_config) != ESP_OK)
        return false;
    if (twai_start() != ESP_OK)
        return false;
    return true;
}

bool RS02PrivateTWAI::sendExt(unsigned long id, const uint8_t *payload, uint8_t len)
{
    if (len > 8)
        len = 8;
    twai_message_t msg = {};
    msg.identifier = id & 0x1FFFFFFF; // 29bit
    msg.flags = TWAI_MSG_FLAG_EXTD;
    msg.data_length_code = len;
    memcpy(msg.data, payload, len);
    return twai_transmit(&msg, pdMS_TO_TICKS(50)) == ESP_OK;
}

bool RS02PrivateTWAI::readAny(RS02PrivFrame &out)
{
    twai_message_t msg = {};
    esp_err_t r = twai_receive(&msg, 0);
    if (r != ESP_OK)
        return false;
    out.id = msg.identifier;
    out.dlc = (uint8_t)msg.data_length_code;
    memcpy(out.data, msg.data, out.dlc);
    out.isExt = (msg.flags & TWAI_MSG_FLAG_EXTD) != 0;
    return true;
}

// ===== Type0/3/4 =====
bool RS02PrivateTWAI::ping(uint8_t targetId)
{
    uint8_t d[8] = {0};
    auto id = buildExId(0x00, da2_master(), targetId);
    return sendExt(id, d, 8);
}
bool RS02PrivateTWAI::enable(uint8_t targetId)
{
    uint8_t d[8] = {0};
    auto id = buildExId(0x03, da2_master(), targetId);
    return sendExt(id, d, 8);
}
bool RS02PrivateTWAI::stop(uint8_t targetId, bool clearFault)
{
    uint8_t d[8] = {0};
    if (clearFault)
    {
        d[0] = 0x00;
        d[1] = 0x01;
    }
    auto id = buildExId(0x04, da2_master(), targetId);
    return sendExt(id, d, 8);
}

bool RS02PrivateTWAI::setMotorId(uint8_t currentId, uint8_t newId)
{
    // Type7: mode=0x07, DataArea2 = [newId:high][hostId:low], dst=currentId
    uint8_t d[8] = {0};
    auto id = buildExId(0x07, ((uint16_t)newId << 8) | _hostId, currentId);
    return sendExt(id, d, 8);
}

bool RS02PrivateTWAI::saveParams(uint8_t targetId)
{
    // Type22: 保存。データ内容は仕様上ダミーでOK（01..08を送る例）
    uint8_t d[8] = {1, 2, 3, 4, 5, 6, 7, 8};
    auto id = buildExId(0x16, ((uint16_t)_hostId << 8) | targetId, targetId);
    return sendExt(id, d, 8);
}

bool RS02PrivateTWAI::setMotorIdViaParam(uint8_t targetId, uint8_t newId, bool save)
{
    // 0x200A = CAN_ID (uint8)
    uint8_t v[4] = {newId, 0, 0, 0};
    bool ok = writeParamLE(targetId, 0x200A, v); // Type18
    if (ok && save)
        ok &= saveParams(targetId); // Type22（必要に応じて）
    return ok;
}

// ===== Param Write/Read (index=LE) =====
bool RS02PrivateTWAI::writeParamLE(uint8_t targetId, uint16_t index, const uint8_t valueLE[4])
{
    uint8_t d[8] = {0};
    d[0] = (uint8_t)(index & 0xFF);
    d[1] = (uint8_t)(index >> 8);
    d[4] = valueLE[0];
    d[5] = valueLE[1];
    d[6] = valueLE[2];
    d[7] = valueLE[3];
    auto id = buildExId(0x12, da2_master(), targetId);
    return sendExt(id, d, 8);
}
bool RS02PrivateTWAI::writeFloatParam(uint8_t targetId, uint16_t index, float value)
{
    uint8_t v[4];
    packF32LE(value, v);
    return writeParamLE(targetId, index, v);
}
bool RS02PrivateTWAI::readParamRaw(uint8_t targetId, uint16_t index, uint8_t out4LE[4])
{
    // 要求送信
    uint8_t d[8] = {0};
    d[0] = (uint8_t)(index & 0xFF);
    d[1] = (uint8_t)(index >> 8);
    auto rid = buildExId(0x11, da2_master(), targetId);
    if (!sendExt(rid, d, 8))
        return false;

    // 応答待ち
    RS02PrivFrame f;
    uint32_t t0 = millis();
    while ((millis() - t0) < 300)
    {
        if (!readAny(f))
        {
            delay(1);
            continue;
        }
        uint8_t type = (uint8_t)((f.id >> 24) & 0x1F);
        if (type != 0x11)
            continue;

        // ★ 応答dstが targetId（モータID）で来る個体も許可
        uint8_t dst = (uint8_t)(f.id & 0xFF);
        if (!(dst == _hostId || dst == 0x00 || dst == 0xFF || dst == 0xFE || dst == targetId))
            continue;

        // indexエコー（LE/BEどちらでも一致でOK）
        uint16_t idxLE = (uint16_t)f.data[0] | ((uint16_t)f.data[1] << 8);
        uint16_t idxBE = (uint16_t)f.data[1] | ((uint16_t)f.data[0] << 8);
        if (idxLE != index && idxBE != index)
            continue;

        out4LE[0] = f.data[4];
        out4LE[1] = f.data[5];
        out4LE[2] = f.data[6];
        out4LE[3] = f.data[7];
        return true;
    }
    return false;
}
bool RS02PrivateTWAI::readFloatParam(uint8_t targetId, uint16_t index, float &out)
{
    uint8_t le[4];
    if (!readParamRaw(targetId, index, le))
        return false;
    memcpy(&out, le, 4);
    return true;
}

// ===== Protocol / Report =====
bool RS02PrivateTWAI::switchProtocol(uint8_t targetId, uint8_t fcmd)
{
    uint8_t d[8] = {1, 2, 3, 4, 5, 6, fcmd, 0};
    auto id = buildExId(0x19, da2_master(), targetId);
    return sendExt(id, d, 8);
}
bool RS02PrivateTWAI::setActiveReport(uint8_t targetId, bool enableFlag)
{
    uint8_t d[8] = {1, 2, 3, 4, 5, 6, (uint8_t)(enableFlag ? 1 : 0), 0};
    auto id = buildExId(0x18, da2_master(), targetId);
    return sendExt(id, d, 8);
}
bool RS02PrivateTWAI::setReportIntervalTicks(uint8_t targetId, uint16_t ticks)
{
    uint8_t v[4] = {(uint8_t)(ticks & 0xFF), (uint8_t)(ticks >> 8), 0, 0};
    return writeParamLE(targetId, RS02Idx::IDX_EPSCAN_TIME, v);
}

// ===== Operation Control (Type1 only DA2=torque) =====
bool RS02PrivateTWAI::opControl(uint8_t targetId, float torqueNm, float posRad, float velRadS, float kp, float kd)
{
    const float P_MIN = -12.57f, P_MAX = 12.57f, V_MIN = -44.0f, V_MAX = 44.0f, KP_MIN = 0.0f, KP_MAX = 500.0f, KD_MIN = 0.0f, KD_MAX = 5.0f, T_MIN = -17.0f, T_MAX = 17.0f;
    uint16_t uP = float_to_uint(posRad, P_MIN, P_MAX);
    uint16_t uV = float_to_uint(velRadS, V_MIN, V_MAX);
    uint16_t uKP = float_to_uint(kp, KP_MIN, KP_MAX);
    uint16_t uKD = float_to_uint(kd, KD_MIN, KD_MAX);
    uint16_t uT = float_to_uint(torqueNm, T_MIN, T_MAX);
    uint8_t d[8];
    packU16BE(uP, &d[0]);
    packU16BE(uV, &d[2]);
    packU16BE(uKP, &d[4]);
    packU16BE(uKD, &d[6]);
    auto id = buildExId(0x01, uT, targetId); // Type1のみDA2=トルク
    return sendExt(id, d, 8);
}

// ===== Type2 parse =====
bool RS02PrivateTWAI::parseFeedback(const RS02PrivFrame &f, RS02Feedback &out)
{
    if (((f.id >> 24) & 0x1F) != 0x02 || f.dlc < 8)
        return false;
    out.motorId = (uint8_t)(f.id & 0xFF);
    out.faultBits = (uint16_t)((f.id >> 16) & 0x3F);
    out.mode = (uint8_t)((f.id >> 22) & 0x03);
    uint16_t uP = ((uint16_t)f.data[0] << 8) | f.data[1];
    uint16_t uV = ((uint16_t)f.data[2] << 8) | f.data[3];
    uint16_t uT = ((uint16_t)f.data[4] << 8) | f.data[5];
    uint16_t uC = ((uint16_t)f.data[6] << 8) | f.data[7];
    out.angleRad = uint_to_float(uP, -12.57f, 12.57f);
    out.velRadS = uint_to_float(uV, -44.0f, 44.0f);
    out.torqueNm = uint_to_float(uT, -17.0f, 17.0f);
    out.tempC = (float)uC * 0.1f;
    return true;
}

// ===== Run mode =====
bool RS02PrivateTWAI::setRunMode(uint8_t targetId, uint8_t runMode)
{
    uint8_t v[4] = {runMode, 0, 0, 0};
    return writeParamLE(targetId, RS02Idx::RUN_MODE, v);
}
bool RS02PrivateTWAI::readRunMode(uint8_t targetId, uint8_t &outMode)
{
    uint8_t le[4] = {0};
    if (!readParamRaw(targetId, RS02Idx::RUN_MODE, le))
        return false;
    outMode = le[0];
    return true;
}

// ===== Velocity =====
bool RS02PrivateTWAI::enterVelocity(uint8_t targetId, float limitCurA, float accRadS2, float spdKp, float spdKi)
{
    uint8_t rm[4] = {2, 0, 0, 0};
    bool ok = writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
    ok &= writeFloatParam(targetId, RS02Idx::LIMIT_CUR, limitCurA);
    ok &= writeFloatParam(targetId, RS02Idx::ACC_RAD, accRadS2);
    if (!isnan(spdKp))
        ok &= writeFloatParam(targetId, RS02Idx::SPD_KP, spdKp);
    if (!isnan(spdKi))
        ok &= writeFloatParam(targetId, RS02Idx::SPD_KI, spdKi);
    return ok;
}
bool RS02PrivateTWAI::velocityRef(uint8_t targetId, float spdRadS)
{
    return writeFloatParam(targetId, RS02Idx::SPD_REF, spdRadS);
}
bool RS02PrivateTWAI::enterVelocityStrict(uint8_t targetId, float limitTorqueNm, float limitCurA, float accRadS2, float spdKp, float spdKi)
{
    bool ok = stop(targetId, true);
    delay(100);
    uint8_t rm[4] = {2, 0, 0, 0};
    ok &= writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
    delay(50);
    ok &= enable(targetId);
    delay(50);
    ok &= writeFloatParam(targetId, RS02Idx::LIMIT_TORQUE, limitTorqueNm);
    ok &= writeFloatParam(targetId, RS02Idx::LIMIT_CUR, limitCurA);
    ok &= writeFloatParam(targetId, RS02Idx::ACC_RAD, accRadS2);
    if (!isnan(spdKp))
        ok &= writeFloatParam(targetId, RS02Idx::SPD_KP, spdKp);
    if (!isnan(spdKi))
        ok &= writeFloatParam(targetId, RS02Idx::SPD_KI, spdKi);
    return ok;
}
bool RS02PrivateTWAI::bringUpVelocityPerSpec(uint8_t targetId, float limitCurA, float accRadS2, float spdRadS)
{
    bool ok = true;
    uint8_t rm[4] = {2, 0, 0, 0};
    ok &= writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
    delay(50);
    ok &= enable(targetId);
    delay(50);
    ok &= writeFloatParam(targetId, RS02Idx::LIMIT_CUR, limitCurA);
    ok &= writeFloatParam(targetId, RS02Idx::ACC_RAD, accRadS2);
    ok &= writeFloatParam(targetId, RS02Idx::SPD_REF, spdRadS);
    return ok;
}

// ===== PP =====
bool RS02PrivateTWAI::enterPP(uint8_t targetId, float limitSpdRadS, float locKp)
{
    uint8_t rm[4] = {1, 0, 0, 0};
    bool ok = writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
    ok &= writeFloatParam(targetId, RS02Idx::LIMIT_SPD, limitSpdRadS);
    if (!isnan(locKp))
        ok &= writeFloatParam(targetId, RS02Idx::LOC_KP, locKp);
    return ok;
}
bool RS02PrivateTWAI::ppLocRef(uint8_t targetId, float posRad)
{
    return writeFloatParam(targetId, RS02Idx::LOC_REF, posRad);
}
bool RS02PrivateTWAI::bringUpPPPerSpec(uint8_t targetId, float limitSpdRadS, float posRad)
{
    bool ok = true;
    uint8_t rm[4] = {1, 0, 0, 0};
    ok &= writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
    delay(50);
    ok &= enable(targetId);
    delay(50);
    ok &= writeFloatParam(targetId, RS02Idx::LIMIT_SPD, limitSpdRadS);
    ok &= writeFloatParam(targetId, RS02Idx::LOC_REF, posRad);
    return ok;
}

// ===== Current =====
bool RS02PrivateTWAI::enterCurrent(uint8_t targetId, float limitTorqueNm, float curKp, float curKi)
{
    uint8_t rm[4] = {3, 0, 0, 0};
    bool ok = writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
    ok &= writeFloatParam(targetId, RS02Idx::LIMIT_TORQUE, limitTorqueNm);
    if (!isnan(curKp))
        ok &= writeFloatParam(targetId, RS02Idx::CUR_KP, curKp);
    if (!isnan(curKi))
        ok &= writeFloatParam(targetId, RS02Idx::CUR_KI, curKi);
    return ok;
}
bool RS02PrivateTWAI::currentIqRef(uint8_t targetId, float iqA)
{
    return writeFloatParam(targetId, RS02Idx::IQ_REF, iqA); // 0x7006
}
bool RS02PrivateTWAI::bringUpCurrentPerSpec(uint8_t targetId, float iqA)
{
    bool ok = true;
    uint8_t rm[4] = {3, 0, 0, 0};
    ok &= writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
    delay(50);
    ok &= enable(targetId);
    delay(50);
    ok &= writeFloatParam(targetId, RS02Idx::IQ_REF, iqA);
    return ok;
}

// ===== CSP =====
bool RS02PrivateTWAI::enterCSP(uint8_t targetId, float limitSpdRadS, float limitCurA, float locKp)
{
    uint8_t rm[4] = {5, 0, 0, 0};
    bool ok = writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
    ok &= writeFloatParam(targetId, RS02Idx::LIMIT_SPD, limitSpdRadS);
    ok &= writeFloatParam(targetId, RS02Idx::LIMIT_CUR, limitCurA);
    if (!isnan(locKp))
        ok &= writeFloatParam(targetId, RS02Idx::LOC_KP, locKp);
    return ok;
}
bool RS02PrivateTWAI::cspLocRef(uint8_t targetId, float posRad)
{
    return writeFloatParam(targetId, RS02Idx::LOC_REF, posRad);
}
bool RS02PrivateTWAI::bringUpCSPPerSpec(uint8_t targetId, float limitSpdRadS, float posRad)
{
    bool ok = true;
    uint8_t rm[4] = {5, 0, 0, 0};
    ok &= writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
    delay(50);
    ok &= enable(targetId);
    delay(50);
    ok &= writeFloatParam(targetId, RS02Idx::LIMIT_SPD, limitSpdRadS);
    ok &= writeFloatParam(targetId, RS02Idx::LOC_REF, posRad);
    return ok;
}
bool RS02PrivateTWAI::enterCSP_simple(uint8_t targetId, float limitSpdRadS)
{
    uint8_t rm[4] = {5, 0, 0, 0};
    if (!writeParamLE(targetId, RS02Idx::RUN_MODE, rm))
        return false;
    if (!writeFloatParam(targetId, RS02Idx::LIMIT_SPD, limitSpdRadS))
        return false;
    delay(5);
    return enable(targetId);
}
bool RS02PrivateTWAI::enterCSP_robust(uint8_t targetId, float limitSpdRadS, float limitCurA, float locKp)
{
    bool ok = stop(targetId, true);
    delay(100);
    uint8_t rm5[4] = {5, 0, 0, 0};
    ok &= writeParamLE(targetId, RS02Idx::RUN_MODE, rm5);
    delay(30);
    ok &= writeFloatParam(targetId, RS02Idx::LIMIT_SPD, limitSpdRadS);
    ok &= writeFloatParam(targetId, RS02Idx::LIMIT_CUR, limitCurA);
    ok &= writeFloatParam(targetId, RS02Idx::LIMIT_CUR_OLD, limitCurA);
    if (!isnan(locKp))
        ok &= writeFloatParam(targetId, RS02Idx::LOC_KP, locKp);
    ok &= enable(targetId);
    delay(50);
    ok &= writeParamLE(targetId, RS02Idx::RUN_MODE, rm5);
    delay(30);
    uint8_t cur = 0xFF;
    if (readRunMode(targetId, cur) && cur != 5)
        return false;
    return ok;
}

// ===== 無限回転（パラメータ合成; 使えないFWあり）=====
bool RS02PrivateTWAI::getInfiniteByParams(uint8_t targetId, double &turns, double &angleRad)
{
    float rotA = 0.0f, rotB = 0.0f, mod = 0.0f;
    bool okA = readFloatParam(targetId, RS02Idx::IDX_ROTATION, rotA);
    bool okB = readFloatParam(targetId, RS02Idx::IDX_MECH_ANGLE_ROT, rotB);
    bool okM = readFloatParam(targetId, RS02Idx::IDX_MODPOS, mod);
    if (!okM)
        return false;
    float rot = okB ? rotB : (okA ? rotA : 0.0f);
    turns = (double)llround((double)rot);
    angleRad = turns * TWO_PI + (double)mod;
    return (okA || okB);
}
