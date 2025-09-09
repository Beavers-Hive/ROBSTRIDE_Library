#include "Robstride.h"

// 定数の定義 (データシート Page 30, "MIT Dynamic Parameters" に基づく)
const float Robstride::P_MIN = -12.57f; // -4pi
const float Robstride::P_MAX = 12.57f;  // +4pi
const float Robstride::V_MIN = -44.0f;
const float Robstride::V_MAX = 44.0f;
const float Robstride::T_MIN = -17.0f;
const float Robstride::T_MAX = 17.0f;
const float Robstride::KP_MIN = 0.0f;
const float Robstride::KP_MAX = 500.0f;
const float Robstride::KD_MIN = 0.0f;
const float Robstride::KD_MAX = 5.0f;

Robstride::Robstride(MCP_CAN &can, byte motor_id, byte host_id) : _p_act(0.0), _v_act(0.0), _t_act(0.0), _mcu_unique_id(0)
{
    _can = &can;
    _motor_id = motor_id;
    _host_id = host_id;
}

void Robstride::enableMotor()
{
    // MIT Protocol, Command 1: Enable Motor Operation
    byte data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    sendCommand(_motor_id, data);
}

void Robstride::disableMotor()
{
    // MIT Protocol, Command 2: Stop Motor Operation
    byte data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    sendCommand(_motor_id, data);
}

void Robstride::setPosition(float p_des, float v_des, float t_ff, float kp, float kd)
{
    // MIT Protocol, Command 3: MIT Dynamic Parameters
    unsigned int p_int = floatToUint(p_des, P_MIN, P_MAX, 16);
    unsigned int v_int = floatToUint(v_des, V_MIN, V_MAX, 12);
    unsigned int kp_int = floatToUint(kp, KP_MIN, KP_MAX, 12);
    unsigned int kd_int = floatToUint(kd, KD_MIN, KD_MAX, 12);
    unsigned int t_int = floatToUint(t_ff, T_MIN, T_MAX, 12);

    byte data[8];
    data[0] = p_int >> 8;
    data[1] = p_int & 0xFF;
    data[2] = v_int >> 4;
    data[3] = ((v_int & 0xF) << 4) | (kp_int >> 8);
    data[4] = kp_int & 0xFF;
    data[5] = kd_int >> 4;
    data[6] = ((kd_int & 0xF) << 4) | (t_int >> 8);
    data[7] = t_int & 0xFF;

    sendCommand(_motor_id, data);
}

void Robstride::requestDeviceID()
{
    // Communication Type 0: Get device ID (拡張フレーム)
    // ID構造: type=0, data=0, dest=_motor_id
    unsigned long can_id = (0x0UL << 24) | (0x0UL << 8) | _motor_id;
    byte data[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    sendExtendedCommand(can_id, 8, data);
}

bool Robstride::parseReply(unsigned long can_id, byte len, byte buf[])
{
    // 標準フレーム(11bit ID)か拡張フレーム(29bit ID)かをIDの大きさで判定
    if (can_id <= 0x7FF)
    {
        // --- MITプロトコルからの返信 (標準フレーム) ---
        if (can_id == _host_id)
        {
            if (len >= 6 && buf[0] == _motor_id)
            {
                unsigned int p_int = (buf[1] << 8) | buf[2];
                unsigned int v_int = (buf[3] << 4) | (buf[4] >> 4);
                unsigned int t_int = ((buf[4] & 0x0F) << 8) | buf[5];
                _p_act = uintToFloat(p_int, P_MIN, P_MAX, 16);
                _v_act = uintToFloat(v_int, V_MIN, V_MAX, 12);
                _t_act = uintToFloat(t_int, T_MIN, T_MAX, 12);
                return true;
            }
        }
    }
    else
    {
        // --- プライベートプロトコルからの返信 (拡張フレーム) ---
        // Device ID取得コマンドへの返信か確認
        // 返信ID構造: type=0, data=_motor_id, dest=0xFE
        unsigned long expected_reply_id = (0x0UL << 24) | ((unsigned long)_motor_id << 8) | 0xFE;
        if (can_id == expected_reply_id)
        {
            if (len == 8)
            {
                _mcu_unique_id = 0;
                for (int i = 0; i < 8; i++)
                {
                    _mcu_unique_id = (_mcu_unique_id << 8) | buf[i];
                }
                return true; // 固有IDの解析成功
            }
        }
    }
    return false; // 解釈できないメッセージ
}

float Robstride::getPosition() { return _p_act; }
float Robstride::getVelocity() { return _v_act; }
float Robstride::getTorque() { return _t_act; }
byte Robstride::getMotorID() { return _motor_id; }
uint64_t Robstride::getMCUUniqueID() { return _mcu_unique_id; }

void Robstride::sendCommand(byte can_id, byte data[])
{
    // 標準フレーム(11-bit ID)で送信
    _can->sendMsgBuf(can_id, 0, 8, data);
}

void Robstride::sendExtendedCommand(unsigned long can_id, byte len, byte data[])
{
    // 拡張フレーム(29-bit ID)で送信
    _can->sendMsgBuf(can_id, 1, len, data);
}

unsigned int Robstride::floatToUint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    if (x > x_max)
        x = x_max;
    if (x < x_min)
        x = x_min;
    return (unsigned int)((x - x_min) * ((float)((1 << bits) - 1) / span));
}

float Robstride::uintToFloat(unsigned int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    return ((float)x_int * span / ((float)((1 << bits) - 1))) + x_min;
}
