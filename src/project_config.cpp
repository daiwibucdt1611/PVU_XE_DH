#include "project_config.h"
#include "crsf.h"

// NA CA TEAM//
//--rc variable---//                                                                                            //NA CA TEAM//
uint8_t _rcs_buf[SBUS_BUFFER_SIZE]{};
uint16_t _raw_rc_values[RC_INPUT_MAX_CHANNELS]{};
uint16_t _raw_rc_count{};
int aileronsMapped{};
int elevatorMapped{};
int throttleMapped{};
int rudderMapped{};
int switchMapped1{};
int switchMapped2{};
int switchMapped3{};
int switchMapped4{};
int mode = NONE;
bool headless_mode = false;
bool headless_prev = false;
float headless_yaw_ref = 0.0f;
volatile bool yaw_hold_enabled = false; // bật/tắt giữ hướng
int servo_mid_pos = 0;
int servo_left_mid_pos = 0;
int servo_right_mid_pos = 0;
int servo_up_pos = 0;
int servo_down_pos = 0;
