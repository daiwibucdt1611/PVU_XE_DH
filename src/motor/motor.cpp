#include "motor.h"

void motor_init()
{
    // Cấu hình chân DIR và PWM cho mỗi motor
    pinMode(MOTOR0_DIR_PIN, OUTPUT);
    pinMode(MOTOR0_PWM_PIN, OUTPUT);
    ledcSetup(MOTOR0_PWM_CHANNEL, MOTOR_FREQUENCY, 8); // 8-bit resolution
    ledcSetup(MOTOR0_DIR_CHANNEL, MOTOR_FREQUENCY, 8);
    ledcAttachPin(MOTOR0_PWM_PIN, MOTOR0_PWM_CHANNEL);
    ledcAttachPin(MOTOR0_DIR_PIN, MOTOR0_DIR_CHANNEL);

    pinMode(MOTOR1_DIR_PIN, OUTPUT);
    pinMode(MOTOR1_PWM_PIN, OUTPUT);
    ledcSetup(MOTOR1_PWM_CHANNEL, MOTOR_FREQUENCY, 8);
    ledcSetup(MOTOR1_DIR_CHANNEL, MOTOR_FREQUENCY, 8);
    ledcAttachPin(MOTOR1_PWM_PIN, MOTOR1_PWM_CHANNEL);
    ledcAttachPin(MOTOR1_DIR_PIN, MOTOR1_DIR_CHANNEL);

    pinMode(MOTOR2_DIR_PIN, OUTPUT);
    pinMode(MOTOR2_PWM_PIN, OUTPUT);
    ledcSetup(MOTOR2_PWM_CHANNEL, MOTOR_LOW_FREQUENCY, 8);
    ledcSetup(MOTOR2_DIR_CHANNEL, MOTOR_LOW_FREQUENCY, 8);
    ledcAttachPin(MOTOR2_PWM_PIN, MOTOR2_PWM_CHANNEL);
    ledcAttachPin(MOTOR2_DIR_PIN, MOTOR2_DIR_CHANNEL);

    pinMode(MOTOR3_DIR_PIN, OUTPUT);
    pinMode(MOTOR3_PWM_PIN, OUTPUT);
    ledcSetup(MOTOR3_PWM_CHANNEL, MOTOR_LOW_FREQUENCY, 8);
    ledcSetup(MOTOR3_DIR_CHANNEL, MOTOR_LOW_FREQUENCY, 8);
    ledcAttachPin(MOTOR3_PWM_PIN, MOTOR3_PWM_CHANNEL);
    ledcAttachPin(MOTOR3_DIR_PIN, MOTOR3_DIR_CHANNEL);

    pinMode(MOTOR4_DIR_PIN, OUTPUT);
    pinMode(MOTOR4_PWM_PIN, OUTPUT);
    ledcSetup(MOTOR4_PWM_CHANNEL, MOTOR_FREQUENCY, 8);
    ledcSetup(MOTOR4_DIR_CHANNEL, MOTOR_FREQUENCY, 8);
    ledcAttachPin(MOTOR4_PWM_PIN, MOTOR4_PWM_CHANNEL);
    ledcAttachPin(MOTOR4_DIR_PIN, MOTOR4_DIR_CHANNEL);

    pinMode(MOTOR5_DIR_PIN, OUTPUT);
    pinMode(MOTOR5_PWM_PIN, OUTPUT);
    ledcSetup(MOTOR5_PWM_CHANNEL, MOTOR_FREQUENCY, 8);
    ledcSetup(MOTOR5_DIR_CHANNEL, MOTOR_FREQUENCY, 8);
    ledcAttachPin(MOTOR5_PWM_PIN, MOTOR5_PWM_CHANNEL);
    ledcAttachPin(MOTOR5_DIR_PIN, MOTOR5_DIR_CHANNEL);
}

int mapRC(uint16_t raw_value, int speed)
{
    const int DEAD_ZONE = 8;
    const int MIN_RC = 900;
    const int MAX_RC = 2100;

    // Check for valid RC signal (if out of range, return 0 - no signal)
    if (raw_value < MIN_RC || raw_value > MAX_RC)
    {
        return 0;
    }

    // Map 1000-2000 to -100 to 100
    int mapped = map(raw_value, MIN_RC, MAX_RC, -speed, speed);

    // Apply dead zone
    if (abs(mapped) < DEAD_ZONE)
    {
        mapped = 0;
    }

    return mapped;
}

void setMotorSpeed(int motorIndex, int speed, int decayMode)
{
    if (speed > MAX_SPEED)
        speed = MAX_SPEED;
    else if (speed < MIN_SPEED)
        speed = MIN_SPEED;


    if (decayMode == FAST_DECAY_MODE)
    {
        if(speed < 0)
        {
            switch (motorIndex)
            {
                case 0:
                    ledcWrite(MOTOR0_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR0_PWM_CHANNEL, -speed);
                    break;
                case 1:
                    ledcWrite(MOTOR1_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR1_PWM_CHANNEL, -speed);
                    break;
                case 2:
                    ledcWrite(MOTOR2_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR2_PWM_CHANNEL, -speed);
                    break;
                case 3:
                    ledcWrite(MOTOR3_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR3_PWM_CHANNEL, -speed);
                    break;
                case 4:
                    ledcWrite(MOTOR4_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR4_PWM_CHANNEL, -speed);
                    break;
                case 5:
                    ledcWrite(MOTOR5_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR5_PWM_CHANNEL, -speed);
                    break;
            }
        }
        else if (speed > 0)
        {
            switch (motorIndex)
            {
                case 0:
                    ledcWrite(MOTOR0_DIR_CHANNEL, speed);
                    ledcWrite(MOTOR0_PWM_CHANNEL, 0);
                    break;
                case 1:
                    ledcWrite(MOTOR1_DIR_CHANNEL, speed);
                    ledcWrite(MOTOR1_PWM_CHANNEL, 0);
                    break;
                case 2:
                    ledcWrite(MOTOR2_DIR_CHANNEL, speed);
                    ledcWrite(MOTOR2_PWM_CHANNEL, 0);
                    break;
                case 3:
                    ledcWrite(MOTOR3_DIR_CHANNEL, speed);
                    ledcWrite(MOTOR3_PWM_CHANNEL, 0);
                    break;
                case 4:
                    ledcWrite(MOTOR4_DIR_CHANNEL, speed);
                    ledcWrite(MOTOR4_PWM_CHANNEL, 0);
                    break;
                case 5:
                    ledcWrite(MOTOR5_DIR_CHANNEL, speed);
                    ledcWrite(MOTOR5_PWM_CHANNEL, 0);
                    break;
            }
        }
        else{
            switch (motorIndex)
            {
                case 0:
                    ledcWrite(MOTOR0_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR0_PWM_CHANNEL, 0);
                    break;
                case 1:
                    ledcWrite(MOTOR1_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR1_PWM_CHANNEL, 0);
                    break;
                case 2:
                    ledcWrite(MOTOR2_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR2_PWM_CHANNEL, 0);
                    break;
                case 3:
                    ledcWrite(MOTOR3_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR3_PWM_CHANNEL, 0);
                    break;
                case 4:
                    ledcWrite(MOTOR4_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR4_PWM_CHANNEL, 0);
                    break;
                case 5:
                    ledcWrite(MOTOR5_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR5_PWM_CHANNEL, 0);
                    break;
            }
        }
    }
    else if (decayMode == SLOW_DECAY_MODE)
    {
        if(speed < 0)
        {
            switch (motorIndex)
            {
                case 0:
                    ledcWrite(MOTOR0_DIR_CHANNEL, 255 + speed);
                    ledcWrite(MOTOR0_PWM_CHANNEL, 255);
                    break;
                case 1:
                    ledcWrite(MOTOR1_DIR_CHANNEL, 255 + speed);
                    ledcWrite(MOTOR1_PWM_CHANNEL, 255);
                    break;
                case 2:
                    ledcWrite(MOTOR2_DIR_CHANNEL, 255 + speed);
                    ledcWrite(MOTOR2_PWM_CHANNEL, 255);
                    break;
                case 3:
                    ledcWrite(MOTOR3_DIR_CHANNEL, 255 + speed);
                    ledcWrite(MOTOR3_PWM_CHANNEL, 255);
                    break;
                case 4:
                    ledcWrite(MOTOR4_DIR_CHANNEL, 255 + speed);
                    ledcWrite(MOTOR4_PWM_CHANNEL, 255);
                    break;
                case 5:
                    ledcWrite(MOTOR5_DIR_CHANNEL, 255 + speed);
                    ledcWrite(MOTOR5_PWM_CHANNEL, 255);
                    break;
            }
        }
        else if (speed > 0)
        {
            switch (motorIndex)
            {
                case 0:
                    ledcWrite(MOTOR0_DIR_CHANNEL, 255);
                    ledcWrite(MOTOR0_PWM_CHANNEL, 255 - speed);
                    break;
                case 1:
                    ledcWrite(MOTOR1_DIR_CHANNEL, 255);
                    ledcWrite(MOTOR1_PWM_CHANNEL, 255 - speed);
                    break;
                case 2:
                    ledcWrite(MOTOR2_DIR_CHANNEL, 255);
                    ledcWrite(MOTOR2_PWM_CHANNEL, 255 - speed);
                    break;
                case 3:
                    ledcWrite(MOTOR3_DIR_CHANNEL, 255);
                    ledcWrite(MOTOR3_PWM_CHANNEL, 255 - speed);
                    break;
                case 4:
                    ledcWrite(MOTOR4_DIR_CHANNEL, 255);
                    ledcWrite(MOTOR4_PWM_CHANNEL, 255 - speed);
                    break;
                case 5:
                    ledcWrite(MOTOR5_DIR_CHANNEL, 255);
                    ledcWrite(MOTOR5_PWM_CHANNEL, 255 - speed);
                    break;
            }
        }
        else{
            switch (motorIndex)
            {
                case 0:
                    ledcWrite(MOTOR0_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR0_PWM_CHANNEL, 0);
                    break;
                case 1:
                    ledcWrite(MOTOR1_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR1_PWM_CHANNEL, 0);
                    break;
                case 2:
                    ledcWrite(MOTOR2_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR2_PWM_CHANNEL, 0);
                    break;
                case 3:
                    ledcWrite(MOTOR3_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR3_PWM_CHANNEL, 0);
                    break;
                case 4:
                    ledcWrite(MOTOR4_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR4_PWM_CHANNEL, 0);
                    break;
                case 5:
                    ledcWrite(MOTOR5_DIR_CHANNEL, 0);
                    ledcWrite(MOTOR5_PWM_CHANNEL, 0);
                    break;
            }
        }
    }
}

void omni_kinematics(int vx, int vy, int vr, int *FL, int *FR, int *RR, int *RL)
{
    // Kinematics equations for 4-wheel omni (assuming 45° wheel orientation)
    // Each motor handles one corner
    // vd vx = 0; vy = 100; vr = 0 -> FL = -100, FR = 100, RR = 100, RL = -100
    *FL = vx + vy + vr; // FL (Front-Left)
    *FR = vx - vy - vr; // FR (Front-Right)
    *RR = vx + vy - vr; // RR (Back-Right)
    *RL = vx - vy + vr; // RL (Back-Left)
    // Normalize if any speed exceeds 255
    int max_speed = max({abs(*FL), abs(*FR), abs(*RR), abs(*RL)});
    if (max_speed > 255)
    {
        float scale = 255.0 / max_speed;
        *FL = (int)(*FL * scale);
        *FR = (int)(*FR * scale);
        *RR = (int)(*RR * scale);
        *RL = (int)(*RL * scale);
    }
}

void drive_omni(int vx, int vy, int vr)
{
    int FL, FR, RR, RL;
    omni_kinematics(vx, vy, vr, &FL, &FR, &RR, &RL);

    setMotorSpeed(0, RL);
    setMotorSpeed(1, RR);
    setMotorSpeed(4, FL);
    setMotorSpeed(5, FR);
}