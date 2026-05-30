#pragma once

#include <stdint.h>

// Project-wide constants and pin mapping.
#define Servo_open 800
#define Servo_close 1900
#define RXD2 17
#define TXD2 16
#define TU_PILOT 0
#define PHUC_PILOT 1
#define NONE 2
#define SERVO_FREQ 50
#define HEADLESS_SWITCH_CH 8 //_rc_values[HEADLESS_SWITCH_CH-1]
#define SERVO_I2C_ADDR 0x40
#define SBUS_BUFFER_SIZE 25

// NA CA TEAM//
//--rc variable---//                                                                                            //NA CA TEAM//
extern uint8_t _rcs_buf[SBUS_BUFFER_SIZE];
extern uint16_t _raw_rc_values[];
extern uint16_t _raw_rc_count;
extern int aileronsMapped;
extern int elevatorMapped;
extern int throttleMapped;
extern int rudderMapped;
extern int switchMapped1;
extern int switchMapped2;
extern int switchMapped3;
extern int switchMapped4;
extern int mode;
extern bool headless_mode;
extern bool headless_prev;
extern float headless_yaw_ref;
extern volatile bool yaw_hold_enabled; // bật/tắt giữ hướng
extern int servo_mid_pos;
extern int servo_left_mid_pos;
extern int servo_right_mid_pos;

extern int servo_up_pos;
extern int servo_down_pos;