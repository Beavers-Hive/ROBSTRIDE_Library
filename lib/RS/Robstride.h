#ifndef ROBSTRIDE_H
#define ROBSTRIDE_H

#include <mcp_can.h>

class Robstride
{
public:
    // コンストラクタ: MCP_CANオブジェクト、モーターID、ホスト(ESP32)側のCAN IDを渡します
    Robstride(MCP_CAN &can, byte motor_id, byte host_id);

    // モーターを有効にします
    void enableMotor();

    // モーターを無効にします
    void disableMotor();

    /**
     * @brief モーターに位置指令を送信します (MITモード)
     * @param p_des 目標位置 (ラジアン)
     * @param v_des 速度フィードフォワード (rad/s)
     * @param t_ff トルクフィードフォワード (N-m)
     * @param kp 位置ゲイン (Pゲイン)
     * @param kd 速度ゲイン (Dゲイン)
     */
    void setPosition(float p_des, float v_des, float t_ff, float kp, float kd);

    // デバイスID(MCU固有ID)を要求します (拡張フレーム)
    void requestDeviceID();

    // 受信したCANメッセージを解釈し、モーターの状態を更新します
    bool parseReply(unsigned long can_id, byte len, byte buf[]);

    // 最新のモーター位置 [rad] を取得します
    float getPosition();
    // 最新のモーター速度 [rad/s] を取得します
    float getVelocity();
    // 最新のモータートルク [N-m] を取得します
    float getTorque();
    // 設定されているモーターのCAN IDを取得します
    byte getMotorID();
    // 取得した64bitのMCU固有IDを取得します
    uint64_t getMCUUniqueID();

private:
    MCP_CAN *_can;  // MCP_CANオブジェクトへのポインタ
    byte _motor_id; // モーターのCAN ID
    byte _host_id;  // ホスト(ESP32)側のCAN ID

    // モーターからのフィードバック値
    float _p_act, _v_act, _t_act;
    uint64_t _mcu_unique_id; // MCU固有IDを格納する変数

    // 内部で使用するヘルパー関数
    void sendCommand(byte can_id, byte data[]);
    void sendExtendedCommand(unsigned long can_id, byte len, byte data[]);
    unsigned int floatToUint(float x, float x_min, float x_max, int bits);
    float uintToFloat(unsigned int x_int, float x_min, float x_max, int bits);

    // モーターの物理的・電気的制限の定数
    static const float P_MIN, P_MAX;
    static const float V_MIN, V_MAX;
    static const float T_MIN, T_MAX;
    static const float KP_MIN, KP_MAX;
    static const float KD_MIN, KD_MAX;
};

#endif // ROBSTRIDE_H
