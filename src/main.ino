#include "motor/motor.h"
#include "imu/imu.h"
#include "crsf.h"
#include "project_config.h"
#include "servo/servo.h"
#include <Ps3Controller.h>
void setup()
{
  // put your setup code here, to run once:
  motor_init();
  imu_init();
  servo_init();
  Serial2.begin(420000, SERIAL_8N1, RXD2, TXD2);
  Serial2.setTimeout(0);
  Ps3.begin();
  String address = Ps3.getAddress();
  Serial.print("The ESP32's Bluetooth MAC address is: ");
  Serial.println(address);
}

void loop()
{
  Notify();
  unsigned long now_us_servo = micros();
  static unsigned long last_servo_update_us = 0;
  if (now_us_servo - last_servo_update_us >= 5000)
  {

    last_servo_update_us = now_us_servo;
    write_servo(15, servo_mid_pos);
    write_servo(14, servo_left_mid_pos);
    write_servo(13, servo_right_mid_pos);
    write_servo(12, servo_up_pos);
    write_servo(11, servo_down_pos);
  }
  int serial2Available = Serial2.available();
  static int vx_cmd, vy_cmd, vr_cmd;
  if (serial2Available > 0)
  {
    size_t bytesToRead = (size_t)min(serial2Available, SBUS_BUFFER_SIZE);
    size_t numBytesRead = Serial2.readBytes(_rcs_buf, bytesToRead);
    if (numBytesRead > 0)
    {
      crsf_parse(&_rcs_buf[0], numBytesRead, &_raw_rc_values[0], &_raw_rc_count, RC_INPUT_MAX_CHANNELS);
      // channel 0: Roll (Strafe Left/Right)
      // channel 1: Pitch (Forward/Backward)
      // channel 2: Throttle
      // channel 3: Yaw (Rotation)
      // channel 8, channel 9, (servo)
      static bool servo_mid_pos_prev = false;
      servo_mid_pos = update_servo_toggle((_raw_rc_values[9] > 1500 ? 1 : 0), servo_mid_pos_prev, servo_mid_pos, Servo_open - 100, Servo_close + 250);
      static int speed;
      if (_raw_rc_values[10] < 1100 && _raw_rc_values[11] < 1100)
      {
        speed = 150 - 36; // High switch position -> full speed
      }
      else if (_raw_rc_values[10] < 1600 || _raw_rc_values[11] < 1600)
      {
        speed = 200 - 36; // Middle switch position -> stop
      }
      else if (_raw_rc_values[10] > 1800 || _raw_rc_values[11] > 1800)
      {
        speed = 250; // Middle switch position -> stop
      }
      if(Ps3.data.button.circle){
        speed = 250;
      }
      if (_raw_rc_values[5] < 1200)
      {
        mode = PHUC_PILOT;
      }
      else if (_raw_rc_values[5] < 1600)
      {
        mode = TU_PILOT;
      }
      else
      {
        mode = NONE;
      }

      headless_mode = (_raw_rc_values[HEADLESS_SWITCH_CH - 1] < 1200);

      switch (mode)
      {
      case TU_PILOT:
      {
        vx_cmd = mapRC(_raw_rc_values[1], speed); // Channel 2: Pitch (Forward/Backward)
        vy_cmd = mapRC(_raw_rc_values[0], speed); // Channel 1: Roll (Strafe Left/Right)
        vr_cmd = mapRC(_raw_rc_values[3], speed); // Channel 4: Yaw (Rotation)
        break;
      }
        // Tu pilot

      case PHUC_PILOT:
      {
        // Phuc pilot
        vx_cmd = mapRC(_raw_rc_values[2], speed); // Channel 2: Pitch (Forward/Backward)
        vy_cmd = mapRC(_raw_rc_values[3], speed); // Channel 1: Roll (Strafe Left/Right)
        vr_cmd = mapRC(_raw_rc_values[0], 120);
        break;
      }

      default:
        break;
      }
    }
  }
  if (abs(vr_cmd) > 20)
  {
    yaw_hold_enabled = false;
    portENTER_CRITICAL(&g_imu_mux);
    yaw_target = yaw_angle;
    portEXIT_CRITICAL(&g_imu_mux);
    pid_integral = 0;
  }
  else
  {
    yaw_hold_enabled = true;
  }
  uint32_t imu_ticks = imu_update(vr_cmd);
  float current_yaw;
  portENTER_CRITICAL(&g_imu_mux);
  current_yaw = yaw_angle;
  portEXIT_CRITICAL(&g_imu_mux);
  static unsigned long imu_monitor_last_ms = 0;
  static unsigned long imu_monitor_last_us = 0;
  static uint64_t imu_monitor_delta_sum_us = 0;
  static uint32_t imu_monitor_tick_sum = 0;
  unsigned long now_us = micros();
  if (imu_ticks > 0)
  {
    if (imu_monitor_last_us != 0)
    {
      imu_monitor_delta_sum_us += (uint64_t)(now_us - imu_monitor_last_us);
      imu_monitor_tick_sum += imu_ticks;
    }
    imu_monitor_last_us = now_us;
  }
  unsigned long now_ms = millis();
  if (now_ms - imu_monitor_last_ms >= 1000)
  {
    imu_monitor_last_ms = now_ms;
    float avg_us = imu_monitor_tick_sum ? ((float)imu_monitor_delta_sum_us / imu_monitor_tick_sum) : 0.0f;
    float freq = avg_us > 0.0f ? 1000000.0f / avg_us : 0.0f;
    float yaw_snapshot = 0.0f;
    int vr_snapshot = 0;
    portENTER_CRITICAL(&g_imu_mux);
    yaw_snapshot = yaw_angle;
    vr_snapshot = vr_final;
    portEXIT_CRITICAL(&g_imu_mux);
    Serial.printf("IMU ticks:%lu avg:%.1f us freq:%.1f Hz yaw:%.2f vr:%d\n",
                  (unsigned long)imu_monitor_tick_sum, avg_us, freq, yaw_snapshot, vr_snapshot);
    imu_monitor_delta_sum_us = 0;
    imu_monitor_tick_sum = 0;
  }
  if (headless_mode && !headless_prev)
  {
    headless_yaw_ref = current_yaw; // lock reference at the moment headless is enabled
  }
  headless_prev = headless_mode;

  if (mode != NONE)
  {
    if (headless_mode)
    {
      // Rotate joystick input by -(current_yaw - headless_yaw_ref) to keep initial heading
      float yaw_delta = current_yaw - headless_yaw_ref;
      while (yaw_delta > 180.0f)
        yaw_delta -= 360.0f;
      while (yaw_delta < -180.0f)
        yaw_delta += 360.0f;
      float radians = -yaw_delta * 3.14159f / 180.0f;
      int vx_headless = vx_cmd * cos(radians) + vy_cmd * sin(radians);
      int vy_headless = -vx_cmd * sin(radians) + vy_cmd * cos(radians);
      drive_omni(vx_headless, vy_headless, vr_final);
    }
    else
    {
      drive_omni(vx_cmd, vy_cmd, vr_final);
    }
  }
  else
  {
    drive_omni(0, 0, 0); // stop if no pilot mode selected
  }
  // IMUSnapshot imu = imu_get_snapshot();
  // Serial.printf("Yaw: %7.2f°  gz: %6.2f°/s  ax:%.2f ay:%.2f az:%.2f\n",
  //               imu.yaw, imu.gz, imu.ax, imu.ay, imu.az);
  // delay(20);
  // setMotorSpeed(0, 100); //RL - REAR LEFT
  // setMotorSpeed(1, 100); //RR - REAR RIGHT
  // setMotorSpeed(2, 100);
  // setMotorSpeed(3, 100);
  // setMotorSpeed(4, 100); //FL - FRONT LEFT
  // setMotorSpeed(5, 100); //FR - FRONT RIGHT
}

void Notify()
{
  if (Ps3.data.button.up || Ps3.data.button.triangle)
  {
    setMotorSpeed(3, -200); // RL - REAR LEFT
  }
  else if (Ps3.data.button.down || Ps3.data.button.cross)
  {
    setMotorSpeed(3, 200); // RR - REAR RIGHT
  }
  else
  {
    setMotorSpeed(3, 0);
  }
  static bool servo_left_mid_pos_prev = false;
  servo_left_mid_pos = update_servo_toggle(Ps3.data.button.r1, servo_left_mid_pos_prev, servo_left_mid_pos, Servo_open, Servo_close + 300);
  static bool servo_right_mid_pos_prev = false;
  servo_right_mid_pos = update_servo_toggle(Ps3.data.button.l1, servo_right_mid_pos_prev, servo_right_mid_pos, Servo_open, Servo_close);
  static bool servo_up_pos_prev = false;
  servo_up_pos = update_servo_toggle(Ps3.data.button.r2, servo_up_pos_prev, servo_up_pos, Servo_open, Servo_close + 100);
  static bool servo_down_pos_prev = false;
  servo_down_pos = update_servo_toggle(Ps3.data.button.l2, servo_down_pos_prev, servo_down_pos, Servo_open - 200, Servo_close);
}