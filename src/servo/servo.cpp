#include "servo.h"

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(SERVO_I2C_ADDR);
void servo_init()
{
    Wire.begin(22, 21); // Initialize I2C for PCA9685
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(50); // Analog servos run at ~50 Hz updates
}

void write_servo(int channel, int pulse_us)
{
    pwm.writeMicroseconds(channel, pulse_us);
}

int update_servo_toggle(bool button_now, bool &button_prev, int current_pos, int open_pos, int close_pos)
{
    int servo_pos = current_pos;
    // Toggle only on press edge (not while holding button)
    if (button_now && !button_prev)
    {
        servo_pos = (servo_pos == open_pos) ? close_pos : open_pos;
    }
    button_prev = button_now;
    return servo_pos;
}