#include <Arduino.h>
#include <M5Unified.h>
#include "RS02_MIT.h"

// --- Configuration ---
// --- 設定 ---
const uint8_t CS_PIN = 6;     // CS pin for MCP2515
const uint8_t MOTOR_ID = 1;   // CAN ID of the motor to control
const uint8_t HOST_ID = 0xFE; // ID of this controller (for receiving feedback)

// Create an instance of the RS02 motor controller
// RS02モーターのインスタンスを作成
RS02_MIT motor(CS_PIN, MOTOR_ID, HOST_ID);

unsigned long last_command_time = 0;
unsigned long last_print_time = 0;
unsigned long last_motor_response = 0;    // Track last motor response time
bool motor_communication_success = false; // Track motor communication status

void setup()
{
  Serial.begin(115200);
  while (!Serial)
    ; // Wait for Serial Monitor to open

  // Initialize M5Stack
  // M5Stackを初期化
  M5.begin();

  Serial.println("RS02 MIT Protocol Example");

  // Start CAN communication (e.g., CAN_500KBPS, CAN_1000KBPS)
  // CAN通信を開始 (CAN_500KBPS, CAN_1000KBPSなど)
  if (motor.begin(CAN_500KBPS) == false)
  {
    Serial.println("CAN Init Failed. Halting.");
    // Set display to red for CAN initialization failure
    // CAN初期化失敗時にディスプレイを赤色に設定
    M5.Display.fillScreen(RED);
    M5.Display.setTextColor(WHITE);
    M5.Display.setTextSize(2);
    M5.Display.setCursor(10, 10);
    M5.Display.println("CAN Init Failed");
    while (1)
      ;
  }
  Serial.println("CAN Init Success.");
  // Set display to green for CAN initialization success
  // CAN初期化成功時にディスプレイを緑色に設定
  M5.Display.fillScreen(GREEN);
  M5.Display.setTextColor(WHITE);
  M5.Display.setTextSize(2);
  M5.Display.setCursor(10, 10);
  M5.Display.println("CAN Init Success");
  M5.Display.setCursor(10, 40);
  M5.Display.println("Waiting for Motor...");

  // Enable the motor
  // モーターを有効にする
  delay(100);
  motor.enableMotor();
  Serial.println("Motor Enabled.");
  delay(1000); // Wait a moment for the motor to be ready
}

void loop()
{
  // Send a control command every 20ms
  // 20msごとに制御コマンドを送信
  if (millis() - last_command_time > 20)
  {
    last_command_time = millis();

    // Generate a target position using a sine wave (amplitude 3.14 rad = 180 deg)
    // 目標位置をサインカーブで生成 (振幅 3.14 rad = 180度)
    float target_pos = 3.14f * sin(millis() * 0.001f);

    // Send parameters in MIT mode
    // position(rad), velocity(rad/s), Kp, Kd, torque(Nm)
    // For simple position control, velocity and torque targets are 0.
    // Kp and Kd are gains; adjust them based on motor response.
    motor.setMITParams(target_pos, 0.0f, 20.0f, 0.5f, 0.0f);

    // Debug: Print every 1 second if no motor communication
    // デバッグ: モーター通信がない場合、1秒ごとに表示
    if (!motor_communication_success && (millis() % 1000 < 50))
    {
      Serial.println("Sending commands, waiting for motor response...");
    }
  }

  // Check for feedback from the motor
  // モーターからのフィードバックを受信
  if (motor.readFeedback())
  {
    // Update last response time
    // 最後の応答時間を更新
    last_motor_response = millis();

    // Mark motor communication as successful
    // モーター通信を成功としてマーク
    if (!motor_communication_success)
    {
      motor_communication_success = true;
      // Update display to show motor communication success
      // モーター通信成功を表示するためにディスプレイを更新
      M5.Display.fillScreen(BLUE);
      M5.Display.setTextColor(WHITE);
      M5.Display.setTextSize(2);
      M5.Display.setCursor(10, 10);
      M5.Display.println("Motor Connected!");
      M5.Display.setCursor(10, 40);
      M5.Display.println("CAN: OK");
      M5.Display.setCursor(10, 70);
      M5.Display.println("Motor: OK");
    }

    // Print feedback data every 100ms
    // 100msごとにフィードバックデータを表示
    if (millis() - last_print_time > 100)
    {
      last_print_time = millis();

      MotorFeedback feedback = motor.getFeedback();

      // Update display with real-time motor data
      // リアルタイムのモーターデータでディスプレイを更新
      M5.Display.fillRect(10, 100, 300, 80, BLUE);
      M5.Display.setTextColor(WHITE);
      M5.Display.setTextSize(1);
      M5.Display.setCursor(10, 100);
      M5.Display.print("Pos: ");
      M5.Display.print(feedback.position, 2);
      M5.Display.print(" rad");
      M5.Display.setCursor(10, 115);
      M5.Display.print("Vel: ");
      M5.Display.print(feedback.velocity, 2);
      M5.Display.print(" rad/s");
      M5.Display.setCursor(10, 130);
      M5.Display.print("Tor: ");
      M5.Display.print(feedback.torque, 2);
      M5.Display.print(" Nm");
      M5.Display.setCursor(10, 145);
      M5.Display.print("Temp: ");
      M5.Display.print(feedback.temperature, 1);
      M5.Display.print(" C");

      Serial.print("ID: ");
      Serial.print(feedback.id);
      Serial.print("  Pos: ");
      Serial.print(feedback.position, 2);
      Serial.print("  Vel: ");
      Serial.print(feedback.velocity, 2);
      Serial.print("  Tor: ");
      Serial.print(feedback.torque, 2);
      Serial.print("  Temp: ");
      Serial.print(feedback.temperature, 1);
      Serial.println(" C");
    }
  }
  else
  {
    // Check if motor communication has timed out (no response for 1 second)
    // モーター通信がタイムアウトしたかチェック（1秒間応答なし）
    if (motor_communication_success && (millis() - last_motor_response > 1000))
    {
      motor_communication_success = false;
      // Update display to show motor communication lost
      // モーター通信が失われたことを表示
      M5.Display.fillScreen(ORANGE);
      M5.Display.setTextColor(WHITE);
      M5.Display.setTextSize(2);
      M5.Display.setCursor(10, 10);
      M5.Display.println("Motor Lost!");
      M5.Display.setCursor(10, 40);
      M5.Display.println("CAN: OK");
      M5.Display.setCursor(10, 70);
      M5.Display.println("Motor: NO");
    }
    // If motor was never connected, show waiting status
    // モーターが一度も接続されていない場合、待機状態を表示
    else if (!motor_communication_success && (millis() > 3000)) // Wait 3 seconds before showing waiting
    {
      M5.Display.fillScreen(YELLOW);
      M5.Display.setTextColor(BLACK);
      M5.Display.setTextSize(2);
      M5.Display.setCursor(10, 10);
      M5.Display.println("Waiting...");
      M5.Display.setCursor(10, 40);
      M5.Display.println("CAN: OK");
      M5.Display.setCursor(10, 70);
      M5.Display.println("Motor: ?");
      M5.Display.setCursor(10, 100);
      M5.Display.setTextSize(1);
      M5.Display.print("Time: ");
      M5.Display.print(millis() / 1000);
      M5.Display.println("s");
    }
  }
}
