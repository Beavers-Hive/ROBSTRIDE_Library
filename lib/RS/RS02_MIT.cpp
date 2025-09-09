#include "RS02_MIT.h"

RS02_MIT::RS02_MIT(uint8_t cs_pin, uint8_t motor_id, uint8_t host_id)
    : mcp2515(cs_pin), _motor_id(motor_id), _host_id(host_id)
{
    _feedback.has_data = false;
}

bool RS02_MIT::begin(byte can_speed)
{
    SPI.begin();
    if (mcp2515.begin(MCP_ANY, can_speed, MCP_8MHZ) == CAN_OK)
    {
        mcp2515.setMode(MCP_NORMAL);
        return true;
    }
    return false;
}

void RS02_MIT::enableMotor()
{
    byte data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    mcp2515.sendMsgBuf(_motor_id, 0, 8, data);
}

void RS02_MIT::disableMotor()
{
    byte data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    mcp2515.sendMsgBuf(_motor_id, 0, 8, data);
}

void RS02_MIT::setZeroPosition()
{
    byte data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    mcp2515.sendMsgBuf(_motor_id, 0, 8, data);
}

void RS02_MIT::setMITParams(float position, float velocity, float kp, float kd, float torque)
{
    uint16_t pos_int = floatToUint(position, P_MIN, P_MAX, 16);
    uint16_t vel_int = floatToUint(velocity, V_MIN, V_MAX, 12);
    uint16_t kp_int = floatToUint(kp, KP_MIN, KP_MAX, 12);
    uint16_t kd_int = floatToUint(kd, KD_MIN, KD_MAX, 12);
    uint16_t tor_int = floatToUint(torque, T_MIN, T_MAX, 12);

    byte data[8];
    data[0] = pos_int >> 8;
    data[1] = pos_int & 0xFF;
    data[2] = vel_int >> 4;
    data[3] = ((vel_int & 0x0F) << 4) | (kp_int >> 8);
    data[4] = kp_int & 0xFF;
    data[5] = kd_int >> 4;
    data[6] = ((kd_int & 0x0F) << 4) | (tor_int >> 8);
    data[7] = tor_int & 0xFF;

    mcp2515.sendMsgBuf(_motor_id, 0, 8, data);
}

bool RS02_MIT::readFeedback()
{
    if (mcp2515.checkReceive() != CAN_MSGAVAIL)
    {
        return false;
    }

    long unsigned int rxId;
    byte len = 0;
    byte rxBuf[8];
    mcp2515.readMsgBuf(&rxId, &len, rxBuf);

    // Feedback in MIT protocol is sent to the host ID
    // MITプロトコルのフィードバックはホストID宛に送られてくる
    if (rxId == _host_id && len == 8)
    {
        uint8_t motor_id = rxBuf[0];

        // Check if the feedback is from the motor this instance is responsible for
        // このインスタンスが担当するモーターからのフィードバックか確認
        if (motor_id == _motor_id)
        {
            _feedback.id = motor_id;

            uint16_t pos_raw = (rxBuf[1] << 8) | rxBuf[2];
            uint16_t vel_raw = (rxBuf[3] << 4) | (rxBuf[4] >> 4);
            uint16_t tor_raw = ((rxBuf[4] & 0x0F) << 8) | rxBuf[5];

            _feedback.position = uintToFloat(pos_raw, P_MIN, P_MAX, 16);
            _feedback.velocity = uintToFloat(vel_raw, V_MIN, V_MAX, 12);
            _feedback.torque = uintToFloat(tor_raw, T_MIN, T_MAX, 12);
            // --- 修正点 ---
            // マニュアル(p.30)に基づき、温度データを2バイト(Byte 6-7)から読み取るように修正
            _feedback.temperature = (float)((rxBuf[6] << 8) | rxBuf[7]);
            _feedback.has_data = true;
            return true;
        }
    }
    return false;
}

MotorFeedback RS02_MIT::getFeedback()
{
    return _feedback;
}

// --- Private Helper Functions ---

int RS02_MIT::floatToUint(float x, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    x = constrain(x, x_min, x_max);
    return (int)((x - offset) * ((float)((1 << bits) - 1)) / span);
}

float RS02_MIT::uintToFloat(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int * span / ((float)((1 << bits) - 1))) + offset;
}
