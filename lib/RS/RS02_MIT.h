#ifndef RS02_MIT_H
#define RS02_MIT_H

#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>

// Constants based on the manual (p.21, p.30)
// マニュアル(p.21, p.30)に基づいた定数
#define P_MIN -12.57f // Position min rad
#define P_MAX 12.57f  // Position max rad
#define V_MIN -44.0f  // Velocity min rad/s
#define V_MAX 44.0f   // Velocity max rad/s
#define KP_MIN 0.0f   // Kp min
#define KP_MAX 500.0f // Kp max
#define KD_MIN 0.0f   // Kd min
#define KD_MAX 5.0f   // Kd max
#define T_MIN -17.0f  // Torque min Nm
#define T_MAX 17.0f   // Torque max Nm

// Structure to hold feedback data from the motor
// モーターからのフィードバックデータを格納する構造体
struct MotorFeedback
{
    uint8_t id;
    float position;
    float velocity;
    float torque;
    float temperature;
    bool has_data;
};

class RS02_MIT
{
public:
    RS02_MIT(uint8_t cs_pin, uint8_t motor_id, uint8_t host_id = 0xFD);
    bool begin(byte can_speed);

    // --- Main Control Commands ---
    // --- 主要な制御コマンド ---
    void enableMotor();
    void disableMotor();
    void setZeroPosition();

    // --- MIT Mode Control ---
    // --- MITモード制御 ---
    void setMITParams(float position, float velocity, float kp, float kd, float torque);

    // --- Feedback Handling ---
    // --- フィードバック処理 ---
    bool readFeedback();
    MotorFeedback getFeedback();

private:
    MCP_CAN mcp2515;
    uint8_t _motor_id;
    uint8_t _host_id;
    MotorFeedback _feedback;

    // --- Data Conversion Helper Functions ---
    // --- データ変換ヘルパー関数 ---
    int floatToUint(float x, float x_min, float x_max, int bits);
    float uintToFloat(int x_int, float x_min, float x_max, int bits);
};

#endif // RS02_MIT_H
