#include "imu.h"

static SPISettings imu_spi_cfg(8000000, MSBFIRST, SPI_MODE3);
hw_timer_t *imu_timer = NULL;

// Shared IMU state (single instance)
volatile float yaw_angle = 0.0f;
volatile float yaw_target = 0.0f;
volatile float pid_integral = 0.0f;
volatile float pid_prev_err = 0.0f;
volatile unsigned long last_imu_time = 0;
volatile int vr_final = 0;
// volatile float KP = 4.0f;
// volatile float KI = 0.005f;
// volatile float KD = 20.0f;

volatile float KP = 3.0f;
volatile float KI = 0.08f;
volatile float KD = 0.001f;

volatile IMUSnapshot g_imu = {};
portMUX_TYPE g_imu_mux = portMUX_INITIALIZER_UNLOCKED;
volatile float gz_offset = 0.0f;
volatile float g_deadzone = 0.2f; // fallback, will be overwritten after calibrate
volatile uint32_t imu_tick_pending = 0;



void IRAM_ATTR onImuTimer()
{
    portENTER_CRITICAL_ISR(&g_imu_mux);
    imu_tick_pending++;
    portEXIT_CRITICAL_ISR(&g_imu_mux);
}

// ─── Low-level SPI ───────────────────────────────────────────────────────────

/**
 * Đọc 1 register.
 * LSM6DS3: gửi (reg | 0x80) → dummy byte → nhận kết quả.
 */
uint8_t lsm6ds3_read_reg(uint8_t reg)
{
    SPI.beginTransaction(imu_spi_cfg);
    digitalWrite(IMU_CS_PIN, LOW);
    SPI.transfer(reg | SPI_READ_FLAG);
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(IMU_CS_PIN, HIGH);
    SPI.endTransaction();
    return val;
}

/**
 * Ghi 1 register.
 * LSM6DS3: bit7=0 (write) → tự động khi reg < 0x80.
 */
void lsm6ds3_write_reg(uint8_t reg, uint8_t val)
{
    SPI.beginTransaction(imu_spi_cfg);
    digitalWrite(IMU_CS_PIN, LOW);
    SPI.transfer(reg & ~SPI_READ_FLAG); // đảm bảo bit7=0
    SPI.transfer(val);
    digitalWrite(IMU_CS_PIN, HIGH);
    SPI.endTransaction();
}

/**
 * Burst read: CS giữ thấp suốt quá trình, IF_INC=1 (đã set trong CTRL3_C)
 * tự động tăng địa chỉ register sau mỗi byte.
 */
void lsm6ds3_read_burst(uint8_t reg, uint8_t *buf, uint8_t len)
{
    SPI.beginTransaction(imu_spi_cfg);
    digitalWrite(IMU_CS_PIN, LOW);
    SPI.transfer(reg | SPI_READ_FLAG);
    for (uint8_t i = 0; i < len; i++)
        buf[i] = SPI.transfer(0x00);
    digitalWrite(IMU_CS_PIN, HIGH);
    SPI.endTransaction();
}

// ─── IMU Update (called from loop) ─────────────────────────────────────────-
uint32_t imu_update(int vr_cmd)
{
    uint8_t buf[12];
    uint32_t ticks_to_process = 0;
    uint32_t ticks_processed = 0;

    portENTER_CRITICAL(&g_imu_mux);
    ticks_to_process = imu_tick_pending;
    if (ticks_to_process > 10)
        ticks_to_process = 10; // avoid spiral of death when loop is slow
    imu_tick_pending -= ticks_to_process;
    bool hold_enabled = yaw_hold_enabled;
    float target_yaw = yaw_target;
    portEXIT_CRITICAL(&g_imu_mux);

    ticks_processed = ticks_to_process;

    while (ticks_to_process > 0)
    {
        ticks_to_process--;

        // Đọc 12 byte: OUTX_L_G(0x22) → OUTZ_H_XL(0x2D)
        // Gyro: buf[0..5], Accel: buf[6..11]
        lsm6ds3_read_burst(OUTX_L_G, buf, 12);

        int16_t raw_gz = (int16_t)(buf[5] << 8 | buf[4]);
        int16_t raw_ax = (int16_t)(buf[7] << 8 | buf[6]);
        int16_t raw_ay = (int16_t)(buf[9] << 8 | buf[8]);
        int16_t raw_az = (int16_t)(buf[11] << 8 | buf[10]);

        float gz = raw_gz * GYRO_SENSITIVITY;
        float gz_c = gz - gz_offset;

        float new_yaw;
        portENTER_CRITICAL(&g_imu_mux);
        new_yaw = g_imu.yaw;
        portEXIT_CRITICAL(&g_imu_mux);

        // Tích phân chỉ khi vượt dead-zone
        if (fabsf(gz_c) > g_deadzone)
        {
            new_yaw += gz_c * DT;
            if (new_yaw > 180.0f)
                new_yaw -= 360.0f;
            if (new_yaw < -180.0f)
                new_yaw += 360.0f;
        }

        portENTER_CRITICAL(&g_imu_mux);
        g_imu.yaw = new_yaw;
        g_imu.gz = gz_c;
        g_imu.ax = raw_ax * ACCEL_SENSITIVITY;
        g_imu.ay = raw_ay * ACCEL_SENSITIVITY;
        g_imu.az = raw_az * ACCEL_SENSITIVITY;
        yaw_angle = new_yaw;
        portEXIT_CRITICAL(&g_imu_mux);

        int vr_out = hold_enabled ? compute_yaw_pid(target_yaw, new_yaw) : vr_cmd;

        portENTER_CRITICAL(&g_imu_mux);
        vr_final = vr_out;
        portEXIT_CRITICAL(&g_imu_mux);
    }

    return ticks_processed;
}

// ─── Public API ──────────────────────────────────────────────────────────────
IMUSnapshot imu_get_snapshot()
{
    IMUSnapshot s;
    portENTER_CRITICAL(&g_imu_mux);
    s.yaw = g_imu.yaw;
    s.gz = g_imu.gz;
    s.ax = g_imu.ax;
    s.ay = g_imu.ay;
    s.az = g_imu.az;
    portEXIT_CRITICAL(&g_imu_mux);
    return s;
}

void imu_reset_yaw()
{
    portENTER_CRITICAL(&g_imu_mux);
    g_imu.yaw = 0.0f;
    portEXIT_CRITICAL(&g_imu_mux);
}

// ─── Init ────────────────────────────────────────────────────────────────────
bool lsm6ds3_init()
{
    uint8_t whoami = lsm6ds3_read_reg(WHO_AM_I);
    if (whoami != 0x69)
    {
        Serial.printf("LSM6DS3 not found! WHO_AM_I=0x%02X\n", whoami);
        return false;
    }
    lsm6ds3_write_reg(CTRL3_C, 0x01); // SW reset
    delay(10);
    lsm6ds3_write_reg(CTRL1_XL, 0x60); // 416Hz, ±2g
    lsm6ds3_write_reg(CTRL2_G, 0x60);  // 416Hz, 250°/s
    lsm6ds3_write_reg(CTRL3_C, 0x44);  // BDU=1, IF_INC=1 (quan trọng cho burst SPI)
    delay(10);
    Serial.println("LSM6DS3 OK (SPI)");
    return true;
}
void imu_init()
{
    Serial.begin(115200);
    // Khởi SPI bus với pin custom
    SPI.begin(IMU_SCLK_PIN, IMU_MISO_PIN, IMU_MOSI_PIN, IMU_CS_PIN);
    pinMode(IMU_CS_PIN, OUTPUT);
    digitalWrite(IMU_CS_PIN, HIGH); // CS idle HIGH

    if (!lsm6ds3_init())
    {
        while (1)
            ;
    }

    lsm6ds3_calibrate_gyro(2000);
    g_deadzone = lsm6ds3_measure_noise(500);
    Serial.printf("Dead-zone set to: %.4f deg/s\n", g_deadzone);

    imu_timer = timerBegin(0, 80, true); // 1µs/tick
    timerAttachInterrupt(imu_timer, &onImuTimer, true);
    timerAlarmWrite(imu_timer, 1000, true); // 1ms
    timerAlarmEnable(imu_timer);
}
// ─── Calibration ─────────────────────────────────────────────────────────────
void lsm6ds3_calibrate_gyro(int samples)
{
    Serial.println("Warming up sensor...");
    delay(2000);

    Serial.println("Calibrating, keep still...");
    double sum = 0;
    uint8_t buf[6];
    for (int i = 0; i < samples; i++)
    {
        lsm6ds3_read_burst(OUTX_L_G, buf, 6);
        int16_t raw_gz = (int16_t)(buf[5] << 8 | buf[4]);
        sum += raw_gz * GYRO_SENSITIVITY;
        delay(2);
    }
    gz_offset = (float)(sum / samples);
    Serial.printf("gz_offset = %.5f deg/s\n", gz_offset);

    if (fabsf(gz_offset) > 5.0f)
    {
        Serial.println("WARNING: offset too large, re-calibrating...");
        sum = 0;
        for (int i = 0; i < samples; i++)
        {
            lsm6ds3_read_burst(OUTX_L_G, buf, 6);
            int16_t raw_gz = (int16_t)(buf[5] << 8 | buf[4]);
            sum += raw_gz * GYRO_SENSITIVITY;
            delay(2);
        }
        gz_offset = (float)(sum / samples);
        Serial.printf("gz_offset retry = %.5f deg/s\n", gz_offset);
    }
}

float lsm6ds3_measure_noise(int samples)
{
    double sum = 0, sum_sq = 0;
    uint8_t buf[6];
    for (int i = 0; i < samples; i++)
    {
        lsm6ds3_read_burst(OUTX_L_G, buf, 6);
        int16_t raw_gz = (int16_t)(buf[5] << 8 | buf[4]);
        float gz = raw_gz * GYRO_SENSITIVITY - gz_offset;
        sum += gz;
        sum_sq += gz * gz;
        delay(2);
    }
    float mean = sum / samples;
    float stddev = sqrtf(sum_sq / samples - mean * mean);
    Serial.printf("gz noise: mean=%.4f stddev=%.4f deg/s\n", mean, stddev);
    return stddev * 3.0f; // dead-zone = 3σ
}
int compute_yaw_pid(float target, float current)
{

    float err = target - current;
    while (err > 180.0f)
        err -= 360.0f;
    while (err < -180.0f)
        err += 360.0f;

    pid_integral += err;
    pid_integral = constrain(pid_integral, -50.0f, 50.0f);
    float pid_d = (err - pid_prev_err);
    pid_prev_err = err;

    float out = KP * err + KI * pid_integral + KD * pid_d;
    return (int)constrain(-out, -250.0f, 250.0f);
}

int compute_yaw_pid()
{
    return compute_yaw_pid(yaw_target, yaw_angle);
}