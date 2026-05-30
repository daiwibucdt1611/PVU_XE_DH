#ifndef IMU_H
#define IMU_H
#include <Arduino.h>
#include <SPI.h>
#include "freertos/FreeRTOS.h"

// ─── Pin config ──────────────────────────────────────────────────────────────
#define IMU_CS_PIN   0
#define IMU_MOSI_PIN 23
#define IMU_MISO_PIN 19
#define IMU_SCLK_PIN 18

// ─── LSM6DS3 Register map ────────────────────────────────────────────────────
#define WHO_AM_I   0x0F
#define CTRL1_XL   0x10
#define CTRL2_G    0x11
#define CTRL3_C    0x12
#define OUTX_L_G   0x22
#define OUTX_L_XL  0x28

// LSM6DS3 SPI protocol: bit7=1 → read, bit7=0 → write
#define SPI_READ_FLAG  0x80

// ─── Tuning constants ────────────────────────────────────────────────────────
#define ACCEL_SENSITIVITY  0.000598f   // g/LSB  (±2g, 16-bit)
#define GYRO_SENSITIVITY   0.00875f    // °/s/LSB (250°/s, 16-bit)
#define TIMER_HZ           1000
#define DT                 (1.0f / TIMER_HZ)

// ─── Shared state ────────────────────────────────────────────────────────────
struct IMUSnapshot
{
    float yaw, gz, ax, ay, az;
};
// Yaw PID variables
extern volatile float yaw_angle;
extern volatile float yaw_target;
extern volatile float pid_integral;
extern volatile float pid_prev_err;
extern volatile unsigned long last_imu_time;
extern volatile int vr_final;
extern volatile bool yaw_hold_enabled;
// PID gains - chỉnh sau khi test
extern volatile float KP;
extern volatile float KI;
extern volatile float KD;
extern volatile IMUSnapshot g_imu;
extern portMUX_TYPE g_imu_mux;
extern volatile float gz_offset;
extern volatile float g_deadzone; // fallback, sẽ được ghi đè sau calibrate

// Timer ISR tick counter
extern volatile uint32_t imu_tick_pending;

// ─── Timer ISR ───────────────────────────────────────────────────────────────
extern hw_timer_t *imu_timer;
IMUSnapshot imu_get_snapshot();
void imu_reset_yaw();
void imu_init();
uint32_t imu_update(int vr_cmd);
bool lsm6ds3_init();
void IRAM_ATTR onImuTimer();
uint8_t lsm6ds3_read_reg(uint8_t reg);
void lsm6ds3_write_reg(uint8_t reg, uint8_t val);
void lsm6ds3_read_burst(uint8_t reg, uint8_t *buf, uint8_t len);
void lsm6ds3_calibrate_gyro(int samples);
float lsm6ds3_measure_noise(int samples);
int compute_yaw_pid(float target, float current);
int compute_yaw_pid();
#endif // IMU_H