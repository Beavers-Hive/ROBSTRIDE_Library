#include "RS02PrivateCAN.h"

bool RS02PrivateCAN::begin()
{
  return true;
}

bool RS02PrivateCAN::sendExt(unsigned long id, const byte *payload, byte len)
{
  return _can.sendMsgBuf(id, 1 /*ext*/, len, const_cast<byte *>(payload)) == CAN_OK;
}

bool RS02PrivateCAN::readAny(RS02PrivFrame &out)
{
  if (_can.checkReceive() != CAN_MSGAVAIL)
    return false;
  byte ext = 0;
  if (_can.readMsgBuf(&out.id, &ext, &out.dlc, out.data) == CAN_OK)
  {
    out.isExt = (ext != 0);
    return true;
  }
  if (_can.readMsgBuf(&out.id, &out.dlc, out.data) == CAN_OK)
  {
    out.isExt = true;
    return true;
  }
  return false;
}

// 通信タイプ0: Get device ID（Ping）
bool RS02PrivateCAN::ping(uint8_t targetId)
{
  byte zeros[8] = {0};
  auto id = buildExId(0x00, ((uint16_t)_hostId << 8) | targetId, targetId);
  return sendExt(id, zeros, 8);
}

// 通信タイプ3: Enable
bool RS02PrivateCAN::enable(uint8_t targetId)
{
  byte d[8] = {0};
  auto id = buildExId(0x03, _hostId, targetId);
  return sendExt(id, d, 8);
}

// 通信タイプ4: Stop / Fault Clear（Byte0..1=1でクリア）
bool RS02PrivateCAN::stop(uint8_t targetId, bool clearFault)
{
  byte d[8] = {0};
  if (clearFault)
  {
    d[0] = 0x00;
    d[1] = 0x01;
  }
  auto id = buildExId(0x04, _hostId, targetId);
  return sendExt(id, d, 8);
}

// 通信タイプ18(0x12): Write single parameter（index(2B BE), value(4B LE)）
bool RS02PrivateCAN::writeParamLE(uint8_t targetId, uint16_t index, const byte valueLE[4])
{
  byte d[8] = {0};
  packU16BE(index, &d[0]);
  d[4] = valueLE[0];
  d[5] = valueLE[1];
  d[6] = valueLE[2];
  d[7] = valueLE[3];
  auto id = buildExId(0x12, _hostId, targetId);
  return sendExt(id, d, 8);
}

bool RS02PrivateCAN::writeFloatParam(uint8_t targetId, uint16_t index, float value)
{
  byte v[4];
  packF32LE(value, v);
  return writeParamLE(targetId, index, v);
}

// 通信タイプ25(0x19): Protocol switch（0:Private,1:CANopen,2:MIT）※参考
bool RS02PrivateCAN::switchProtocol(uint8_t targetId, uint8_t fcmd)
{
  byte d[8] = {1, 2, 3, 4, 5, 6, fcmd, 0};
  auto id = buildExId(0x19, _hostId, targetId);
  return sendExt(id, d, 8);
}

// 通信タイプ1: Operation Control（pos/vel/Kp/Kd は data部 16bitBE、torque はID側DataArea2）
bool RS02PrivateCAN::opControl(uint8_t targetId, float torqueNm, float posRad, float velRadS, float kp, float kd)
{
  const float P_MIN = -12.57f, P_MAX = 12.57f, V_MIN = -44.0f, V_MAX = 44.0f, KP_MIN = 0.0f, KP_MAX = 500.0f, KD_MIN = 0.0f, KD_MAX = 5.0f, T_MIN = -17.0f, T_MAX = 17.0f;
  uint16_t uP = float_to_uint(posRad, P_MIN, P_MAX), uV = float_to_uint(velRadS, V_MIN, V_MAX), uKP = float_to_uint(kp, KP_MIN, KP_MAX), uKD = float_to_uint(kd, KD_MIN, KD_MAX), uT = float_to_uint(torqueNm, T_MIN, T_MAX);
  byte d[8];
  packU16BE(uP, &d[0]);
  packU16BE(uV, &d[2]);
  packU16BE(uKP, &d[4]);
  packU16BE(uKD, &d[6]);
  auto id = buildExId(0x01, uT, targetId);
  return sendExt(id, d, 8);
}

bool RS02PrivateCAN::parseFeedback(const RS02PrivFrame &f, Feedback &out)
{
  if (((f.id >> 24) & 0x1F) != 0x02 || f.dlc < 8)
    return false;
  out.motorId = (uint8_t)(f.id & 0xFF);
  out.faultBits = (uint16_t)((f.id >> 16) & 0x3F);
  out.mode = (uint8_t)((f.id >> 22) & 0x03);
  uint16_t uP = unpackU16BE(&f.data[0]), uV = unpackU16BE(&f.data[2]), uT = unpackU16BE(&f.data[4]), uC = unpackU16BE(&f.data[6]);
  out.angleRad = uint_to_float(uP, -12.57f, 12.57f);
  out.velRadS = uint_to_float(uV, -44.0f, 44.0f);
  out.torqueNm = uint_to_float(uT, -17.0f, 17.0f);
  out.tempC = (float)uC * 0.1f;
  return true;
}

// 走行モード（0=Operation,1=PP,2=Velocity,3=Current,5=CSP）: パラメータ 0x7005
bool RS02PrivateCAN::setRunMode(uint8_t targetId, uint8_t runMode)
{
  byte v[4] = {runMode, 0, 0, 0};
  return writeParamLE(targetId, RS02Idx::RUN_MODE, v);
}

// ===== Velocity / Current / CSP 用ユーティリティ =====

// --- Velocity mode ---
// 推奨フロー: stop(clear) → enable() → enterVelocity() → velocityRef() を周期送信(50〜100ms)
bool RS02PrivateCAN::enterVelocity(uint8_t targetId,
                                   float limitCurA,
                                   float accRadS2,
                                   float spdKp,
                                   float spdKi)
{
  // run_mode = 2 (Velocity)
  byte rm[4] = {2, 0, 0, 0};
  bool ok = writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
  ok &= writeFloatParam(targetId, RS02Idx::LIMIT_CUR, limitCurA);
  ok &= writeFloatParam(targetId, RS02Idx::ACC_RAD, accRadS2);
  if (!isnan(spdKp))
    ok &= writeFloatParam(targetId, RS02Idx::SPD_KP, spdKp);
  if (!isnan(spdKi))
    ok &= writeFloatParam(targetId, RS02Idx::SPD_KI, spdKi);
  return ok;
}

// 速度指令（rad/s）※50〜100ms 間隔で連続送信推奨
bool RS02PrivateCAN::velocityRef(uint8_t targetId, float spdRadS)
{
  return writeFloatParam(targetId, RS02Idx::SPD_REF, spdRadS);
}

// Velocityモードの"決め打ちBring-up"手順（PDF順序に厳密）
bool RS02PrivateCAN::enterVelocityStrict(uint8_t targetId,
                                         float limitTorqueNm,
                                         float limitCurA,
                                         float accRadS2,
                                         float spdKp,
                                         float spdKi)
{
  // 0) まず停止してフォルトクリア
  bool ok = stop(targetId, true); // Stop + Fault Clear
  delay(100);                     // 少し待機

  // 1) run_mode=2（Velocity） ※まずモードを切り替える
  byte rm[4] = {2, 0, 0, 0};
  ok &= writeParamLE(targetId, RS02Idx::RUN_MODE, rm); // run_mode=2
  delay(50);                                           // モード切り替え待機

  // 2) 上限&加速を"先に"セット（トルク上限 + 電流上限 + 加速度）
  ok &= writeFloatParam(targetId, RS02Idx::LIMIT_TORQUE, limitTorqueNm); // 必須相当
  ok &= writeFloatParam(targetId, RS02Idx::LIMIT_CUR, limitCurA);
  ok &= writeFloatParam(targetId, RS02Idx::ACC_RAD, accRadS2);

  // 3) 必要なら速度ループゲイン
  if (!isnan(spdKp))
    ok &= writeFloatParam(targetId, RS02Idx::SPD_KP, spdKp);
  if (!isnan(spdKi))
    ok &= writeFloatParam(targetId, RS02Idx::SPD_KI, spdKi);

  // 4) Enable（通信タイプ3）
  ok &= enable(targetId); // Enable

  return ok;
}

// --- Current mode ---
// 推奨フロー: stop(clear) → enable() → enterCurrent() → currentIqRef() を周期送信(20〜50ms)
bool RS02PrivateCAN::enterCurrent(uint8_t targetId,
                                  float limitTorqueNm,
                                  float curKp,
                                  float curKi)
{
  // run_mode = 3 (Current)
  byte rm[4] = {3, 0, 0, 0};
  bool ok = writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
  ok &= writeFloatParam(targetId, RS02Idx::LIMIT_TORQUE, limitTorqueNm);
  if (!isnan(curKp))
    ok &= writeFloatParam(targetId, RS02Idx::CUR_KP, curKp);
  if (!isnan(curKi))
    ok &= writeFloatParam(targetId, RS02Idx::CUR_KI, curKi);
  return ok;
}

// 電流指令（Iq [A]）。モータ仕様の定格内で。
bool RS02PrivateCAN::currentIqRef(uint8_t targetId, float iqA)
{
  return writeFloatParam(targetId, RS02Idx::IQ_REF, iqA);
}

// --- CSP (Cyclic Synchronous Position) ---
// 推奨フロー: stop(clear) → enable() → enterCSP() → cspLocRef() を必要に応じて送信
bool RS02PrivateCAN::enterCSP(uint8_t targetId,
                              float limitSpdRadS,
                              float limitCurA,
                              float locKp)
{
  // run_mode = 5 (CSP)
  byte rm[4] = {5, 0, 0, 0};
  bool ok = writeParamLE(targetId, RS02Idx::RUN_MODE, rm);
  ok &= writeFloatParam(targetId, RS02Idx::LIMIT_SPD, limitSpdRadS);
  ok &= writeFloatParam(targetId, RS02Idx::LIMIT_CUR, limitCurA);
  if (!isnan(locKp))
    ok &= writeFloatParam(targetId, RS02Idx::LOC_KP, locKp);
  return ok;
}

// 位置指令（rad）。必要なら一定周期で再発行（例: 50〜100ms）
bool RS02PrivateCAN::cspLocRef(uint8_t targetId, float posRad)
{
  return writeFloatParam(targetId, RS02Idx::LOC_REF, posRad);
}
