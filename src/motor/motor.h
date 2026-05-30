#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

#define MOTOR_COUNT 6
#define MAX_SPEED 255
#define MIN_SPEED -255
#define MOTOR_FREQUENCY 20000 // Tần số PWM cho động cơ (20 kHz)
#define MOTOR_LOW_FREQUENCY 1000 // Tần số PWM thấp hơn cho chế độ slow decay
#define SLOW_DECAY_MODE 1
#define FAST_DECAY_MODE 2

#define MOTOR0_DIR_PIN 25
#define MOTOR0_PWM_PIN 26
#define MOTOR0_DIR_CHANNEL 0
#define MOTOR0_PWM_CHANNEL 1

#define MOTOR1_DIR_PIN 32
#define MOTOR1_PWM_PIN 33
#define MOTOR1_DIR_CHANNEL 2
#define MOTOR1_PWM_CHANNEL 3

#define MOTOR2_DIR_PIN 15
#define MOTOR2_PWM_PIN 4
#define MOTOR2_DIR_CHANNEL 8
#define MOTOR2_PWM_CHANNEL 9

#define MOTOR3_DIR_PIN 5
#define MOTOR3_PWM_PIN 2
#define MOTOR3_DIR_CHANNEL 10
#define MOTOR3_PWM_CHANNEL 11

#define MOTOR4_DIR_PIN 12
#define MOTOR4_PWM_PIN 27
#define MOTOR4_DIR_CHANNEL 4
#define MOTOR4_PWM_CHANNEL 5

#define MOTOR5_DIR_PIN 13
#define MOTOR5_PWM_PIN 14
#define MOTOR5_DIR_CHANNEL 6
#define MOTOR5_PWM_CHANNEL 7

void motor_init(); // Gọi 1 lần trong setup()
// speed: -255..255, âm là quay ngược, dương là quay thuận
void setMotorSpeed(int motorIndex, int speed, int decayMode = SLOW_DECAY_MODE);
int mapRC(uint16_t raw_value, int speed);
void omni_kinematics(int vx, int vy, int vr, int *FL, int *FR, int *RR, int *RL);
void drive_omni(int vx, int vy, int vr);
#endif // MOTOR_H