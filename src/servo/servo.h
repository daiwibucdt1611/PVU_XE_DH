#ifndef SERVO_H
#define SERVO_H
#include <Arduino.h>
#include "project_config.h"
#include <Adafruit_PWMServoDriver.h>

void servo_init();
void write_servo(int channel, int pulse_us);
int update_servo_toggle(bool button_now, bool &button_prev, int current_pos, int open_pos, int close_pos);
#endif // SERVO_H