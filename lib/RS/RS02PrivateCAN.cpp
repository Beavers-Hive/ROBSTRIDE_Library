// RS02PrivateCAN.cpp — Type17応答dstを緩和（targetIdもOK）/ mechPos=0x7019, mechVel=0x701B
#include "RS02PrivateCAN.h"
#include <math.h>

bool RS02PrivateCAN::begin() { return true; }

bool RS02PrivateCAN::sendExt(unsigned long id, const uint8_t *payload, uint8_t len)
{
  return _can.sendMsgBuf(id, 1 /*ext*/, len, const_cast<uint8_t *>(payload)) == CAN_OK;
}

bool RS02PrivateCAN::readAny(RS02PrivFrame &out)
{
  if (_can.checkReceive() != CAN_MSGAVAIL)
    return false;
  unsigned long cid = 0;
  byte ext = 0, len = 0;
  if (_can.readMsgBuf(&cid, &ext, &len, out.data) != CAN_OK)
    return false; // 4引数版
  out.id = cid;
  out.dlc = (uint8_t)len;
  out.isExt = (ext != 0) || (out.id > 0x7FF);
  return true;
}

// ===== Type0/3/4 =====
bool RS02PrivateCAN::ping(uint8_t targetId)
{
  uint8_t d[8] = {0};
  auto id = buildExId(0x00, da2_master(), targetId);
  return sendExt(id, d, 8);
}
bool RS02PrivateCAN::enable(uint8_t targetId)
{
  uint8_t d[8] = {0};
  auto id = buildExId(0x03, da2_master(), targetId);
  return sendExt(id, d, 8);
}
bool RS02PrivateCAN::stop(uint8_t targetId, bool clearFault)
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

bool RS02PrivateCAN::setMotorId(uint8_t currentId, uint8_t newId)
{
  // Type7: mode=0x07, DataArea2 = [newId:high][hostId:low], dst=currentId
  byte d[8] = {0};
  auto id = buildExId(0x07, ((uint16_t)newId << 8) | _hostId, currentId);
  return sendExt(id, d, 8);
}

bool RS02PrivateCAN::saveParams(uint8_t targetId)
{
  // Type22: 保存。データ内容は仕様上ダミーでOK（01..08を送る例）
  byte d[8] = {1, 2, 3, 4, 5, 6, 7, 8};
  auto id = buildExId(0x16, ((uint16_t)_hostId << 8) | targetId, targetId);
  return sendExt(id, d, 8);
}

bool RS02PrivateCAN::setMotorIdViaParam(uint8_t targetId, uint8_t newId, bool save)
{
  // 0x200A = CAN_ID (uint8)
  byte v[4] = {newId, 0, 0, 0};
  bool ok = writeParamLE(targetId, 0x200A, v); // Type18
  if (ok && save)
    ok &= saveParams(targetId); // Type22（必要に応じて）
  return ok;
}

// ===== Param Write/Read (index=LE) =====
bool RS02PrivateCAN::writeParamLE(uint8_t targetId, uint16_t index, const uint8_t valueLE[4])
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
bool RS02PrivateCAN::writeFloatParam(uint8_t targetId, uint16_t index, float value)
{
  uint8_t v[4];
  packF32LE(value, v);
  return writeParamLE(targetId, index, v);
}
bool RS02PrivateCAN::readParamRaw(uint8_t targetId, uint16_t index, uint8_t out4LE[4])
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
bool RS02PrivateCAN::readFloatParam(uint8_t targetId, uint16_t index, float &out)
{
  uint8_t le[4];
  if (!readParamRaw(targetId, index, le))
    return false;
  memcpy(&out, le, 4);
  return true;
}

// ===== Protocol / Report =====
bool RS02PrivateCAN::switchProtocol(uint8_t targetId, uint8_t fcmd)
{
  uint8_t d[8] = {1, 2, 3, 4, 5, 6, fcmd, 0};
  auto id = buildExId(0x19, da2_master(), targetId);
  return sendExt(id, d, 8);
}
bool RS02PrivateCAN::setActiveReport(uint8_t targetId, bool enableFlag)
{
  uint8_t d[8] = {1, 2, 3, 4, 5, 6, (uint8_t)(enableFlag ? 1 : 0), 0};
  auto id = buildExId(0x18, da2_master(), targetId);
  return sendExt(id, d, 8);
}
bool RS02PrivateCAN::setReportIntervalTicks(uint8_t targetId, uint16_t ticks)
{
  uint8_t v[4] = {(uint8_t)(ticks & 0xFF), (uint8_t)(ticks >> 8), 0, 0};
  return writeParamLE(targetId, RS02Idx::IDX_EPSCAN_TIME, v);
}

// ===== Operation Control (Type1 only DA2=torque) =====
bool RS02PrivateCAN::opControl(uint8_t targetId, float torqueNm, float posRad, float velRadS, float kp, float kd)
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
bool RS02PrivateCAN::parseFeedback(const RS02PrivFrame &f, RS02Feedback &out)
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
bool RS02PrivateCAN::setRunMode(uint8_t targetId, uint8_t runMode)
{
  uint8_t v[4] = {runMode, 0, 0, 0};
  return writeParamLE(targetId, RS02Idx::RUN_MODE, v);
}
bool RS02PrivateCAN::readRunMode(uint8_t targetId, uint8_t &outMode)
{
  uint8_t le[4] = {0};
  if (!readParamRaw(targetId, RS02Idx::RUN_MODE, le))
    return false;
  outMode = le[0];
  return true;
}

// ===== Velocity =====
bool RS02PrivateCAN::enterVelocity(uint8_t targetId, float limitCurA, float accRadS2, float spdKp, float spdKi)
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
bool RS02PrivateCAN::velocityRef(uint8_t targetId, float spdRadS)
{
  return writeFloatParam(targetId, RS02Idx::SPD_REF, spdRadS);
}
bool RS02PrivateCAN::enterVelocityStrict(uint8_t targetId, float limitTorqueNm, float limitCurA, float accRadS2, float spdKp, float spdKi)
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
bool RS02PrivateCAN::bringUpVelocityPerSpec(uint8_t targetId, float limitCurA, float accRadS2, float spdRadS)
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
bool RS02PrivateCAN::enterPP(uint8_t targetId, float limitSpdRadS, float locKp)
{
  uint8_t rm[4] = {1, 0, 0, 0};
  bool ok = writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
  ok &= writeFloatParam(targetId, RS02Idx::LIMIT_SPD, limitSpdRadS);
  if (!isnan(locKp))
    ok &= writeFloatParam(targetId, RS02Idx::LOC_KP, locKp);
  return ok;
}
bool RS02PrivateCAN::ppLocRef(uint8_t targetId, float posRad)
{
  return writeFloatParam(targetId, RS02Idx::LOC_REF, posRad);
}
bool RS02PrivateCAN::bringUpPPPerSpec(uint8_t targetId, float limitSpdRadS, float posRad)
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
bool RS02PrivateCAN::enterCurrent(uint8_t targetId, float limitTorqueNm, float curKp, float curKi)
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
bool RS02PrivateCAN::currentIqRef(uint8_t targetId, float iqA)
{
  return writeFloatParam(targetId, RS02Idx::IQ_REF, iqA); // 0x7006
}
bool RS02PrivateCAN::bringUpCurrentPerSpec(uint8_t targetId, float iqA)
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
bool RS02PrivateCAN::enterCSP(uint8_t targetId, float limitSpdRadS, float limitCurA, float locKp)
{
  uint8_t rm[4] = {5, 0, 0, 0};
  bool ok = writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
  ok &= writeFloatParam(targetId, RS02Idx::LIMIT_SPD, limitSpdRadS);
  ok &= writeFloatParam(targetId, RS02Idx::LIMIT_CUR, limitCurA);
  if (!isnan(locKp))
    ok &= writeFloatParam(targetId, RS02Idx::LOC_KP, locKp);
  return ok;
}
bool RS02PrivateCAN::cspLocRef(uint8_t targetId, float posRad)
{
  return writeFloatParam(targetId, RS02Idx::LOC_REF, posRad);
}
bool RS02PrivateCAN::bringUpCSPPerSpec(uint8_t targetId, float limitSpdRadS, float posRad)
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
bool RS02PrivateCAN::enterCSP_simple(uint8_t targetId, float limitSpdRadS)
{
  uint8_t rm[4] = {5, 0, 0, 0};
  if (!writeParamLE(targetId, RS02Idx::RUN_MODE, rm))
    return false;
  if (!writeFloatParam(targetId, RS02Idx::LIMIT_SPD, limitSpdRadS))
    return false;
  delay(5);
  return enable(targetId);
}
bool RS02PrivateCAN::enterCSP_robust(uint8_t targetId, float limitSpdRadS, float limitCurA, float locKp)
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
bool RS02PrivateCAN::getInfiniteByParams(uint8_t targetId, double &turns, double &angleRad)
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
