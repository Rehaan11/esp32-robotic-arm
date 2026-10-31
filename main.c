
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "esp_log.h"
#include "esp_err.h"

#include "driver/adc.h"
#include "driver/i2c.h"
#include "driver/ledc.h"

// =====================================================
// CONFIGURATION
// =====================================================

#define SERVO_COUNT          5
#define SERVO_FREQ_HZ        50
#define SERVO_RESOLUTION     LEDC_TIMER_14_BIT
#define SERVO_MAX_DUTY       ((1U << 14) - 1U)

#define SERVO_MIN_US         500
#define SERVO_MAX_US         2500
#define SERVO_PERIOD_US      20000

#define I2C_PORT             I2C_NUM_0
#define MPU6050_ADDR         0x68

#define POT_COUNT            5
#define ADC_MAX_VALUE        4095
#define PI_VALUE             3.14159265f
#define KP_GAIN 1.5f 
#define TARGET_PITCH 0.0f

static const char *TAG = "ROBOT_ARM";

// =====================================================
// SERVO GPIO ASSIGNMENTS
// =====================================================

static const int servo_gpio[SERVO_COUNT] = {
    18,  // Servo 1: Base
    19,  // Servo 2: Gripper
    23,  // Servo 3: Elbow
    25,  // Servo 4: Shoulder
    26   // Servo 5: Wrist
};

// =====================================================
// POTENTIOMETER ADC MAPPING
// =====================================================

static const adc1_channel_t pot_channels[POT_COUNT] = {
    ADC1_CHANNEL_4,  // GPIO32
    ADC1_CHANNEL_5,  // GPIO33
    ADC1_CHANNEL_6,  // GPIO34
    ADC1_CHANNEL_3,  // GPIO39 / VN
    ADC1_CHANNEL_0   // GPIO36 / VP
};

// =====================================================
// ANGLE LIMIT
// =====================================================

static int clamp_angle(int angle)
{
    if (angle < 0) {
        return 0;
    }

    if (angle > 180) {
        return 180;
    }

    return angle;
}

// =====================================================
// ADC INITIALIZATION
// =====================================================

static void init_adc(void)
{
    ESP_ERROR_CHECK(
        adc1_config_width(ADC_WIDTH_BIT_12)
    );

    for (int i = 0; i < POT_COUNT; i++) {
        ESP_ERROR_CHECK(
            adc1_config_channel_atten(
                pot_channels[i],
                ADC_ATTEN_DB_11
            )
        );
    }

    ESP_LOGI(TAG, "Potentiometer ADC initialized");
}

// =====================================================
// READ POTENTIOMETERS
// =====================================================

static void read_potentiometers(int pot[POT_COUNT])
{
    for (int i = 0; i < POT_COUNT; i++) {
        int raw = adc1_get_raw(pot_channels[i]);

        if (raw < 0) {
            raw = 0;
        }

        if (raw > ADC_MAX_VALUE) {
            raw = ADC_MAX_VALUE;
        }

        pot[i] = raw;
    }
}

// =====================================================
// ADC VALUE TO SERVO ANGLE
// =====================================================

static int pot_to_angle(int raw)
{
    if (raw < 0) {
        raw = 0;
    }

    if (raw > ADC_MAX_VALUE) {
        raw = ADC_MAX_VALUE;
    }

    return (raw * 180) / ADC_MAX_VALUE;
}

// =====================================================
// SERVO PWM INITIALIZATION
// =====================================================

static void init_servos(void)
{
    ledc_timer_config_t timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .duty_resolution = SERVO_RESOLUTION,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK
    };

    ESP_ERROR_CHECK(
        ledc_timer_config(&timer)
    );

    for (int i = 0; i < SERVO_COUNT; i++) {
        ledc_channel_config_t channel = {
            .gpio_num = servo_gpio[i],
            .speed_mode = LEDC_LOW_SPEED_MODE,
            .channel = (ledc_channel_t)i,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = LEDC_TIMER_0,
            .duty = 0,
            .hpoint = 0
        };

        ESP_ERROR_CHECK(
            ledc_channel_config(&channel)
        );
    }

    ESP_LOGI(TAG, "Five servo PWM channels initialized");
}

// =====================================================
// SET SERVO ANGLE
// =====================================================

static void set_servo_angle(int index, int angle)
{
    if (index < 0 || index >= SERVO_COUNT) {
        return;
    }

    angle = clamp_angle(angle);

    uint32_t pulse_us =
        SERVO_MIN_US +
        ((uint32_t)angle *
         (SERVO_MAX_US - SERVO_MIN_US)) / 180U;

    uint32_t duty =
        (pulse_us * SERVO_MAX_DUTY) /
        SERVO_PERIOD_US;

    ESP_ERROR_CHECK(
        ledc_set_duty(
            LEDC_LOW_SPEED_MODE,
            (ledc_channel_t)index,
            duty
        )
    );

    ESP_ERROR_CHECK(
        ledc_update_duty(
            LEDC_LOW_SPEED_MODE,
            (ledc_channel_t)index
        )
    );
}

// =====================================================
// MPU6050 I2C REGISTER READ
// =====================================================

static esp_err_t mpu6050_read_register(
    uint8_t reg,
    uint8_t *value)
{
    return i2c_master_write_read_device(
        I2C_PORT,
        MPU6050_ADDR,
        &reg,
        1,
        value,
        1,
        pdMS_TO_TICKS(100)
    );
}

// =====================================================
// MPU6050 INITIALIZATION AND DIAGNOSTICS
// SDA = GPIO21, SCL = GPIO22
// =====================================================

static esp_err_t init_mpu6050(void)
{
    i2c_config_t config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000
    };

    esp_err_t err = i2c_param_config(
        I2C_PORT,
        &config
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C configuration failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(
        I2C_PORT,
        I2C_MODE_MASTER,
        0,
        0,
        0
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver installation failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    // Check the sensor identity BEFORE waking it.
    uint8_t who_am_i = 0;

    err = mpu6050_read_register(0x75, &who_am_i);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "WHO_AM_I read failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    printf("MPU6050 WHO_AM_I = 0x%02X\n", who_am_i);

    if (who_am_i != 0x68) {
        ESP_LOGW(TAG,
                 "Unexpected device identity; expected 0x68");
    }

    // PWR_MGMT_1 register 0x6B: clear sleep bit.
    uint8_t wake_data[2] = {0x6B, 0x00};

    err = i2c_master_write_to_device(
        I2C_PORT,
        MPU6050_ADDR,
        wake_data,
        sizeof(wake_data),
        pdMS_TO_TICKS(100)
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 wake-up failed: %s",
                 esp_err_to_name(err));
        return err;
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "MPU6050 initialized");

    return ESP_OK;
}

// =====================================================
// READ ACCELEROMETER AND CALCULATE ROLL/PITCH
// =====================================================

static esp_err_t read_mpu6050_angles(
    float *roll,
    float *pitch)
{
    uint8_t reg = 0x3B;
    uint8_t data[14] = {0};

    esp_err_t err = i2c_master_write_read_device(
        I2C_PORT,
        MPU6050_ADDR,
        &reg,
        1,
        data,
        sizeof(data),
        pdMS_TO_TICKS(100)
    );

    if (err != ESP_OK) {
        return err;
    }

    int16_t ax = (int16_t)(
        ((uint16_t)data[0] << 8) | data[1]
    );

    int16_t ay = (int16_t)(
        ((uint16_t)data[2] << 8) | data[3]
    );

    int16_t az = (int16_t)(
        ((uint16_t)data[4] << 8) | data[5]
    );

    // Debug output: raw accelerometer values.
    printf("ACCEL: ax=%d ay=%d az=%d\n", ax, ay, az);

    *roll = atan2f(
        (float)ay,
        (float)az
    ) * 180.0f / PI_VALUE;

    *pitch = atan2f(
        -(float)ax,
        sqrtf(
            (float)ay * (float)ay +
            (float)az * (float)az
        )
    ) * 180.0f / PI_VALUE;

    return ESP_OK;
}

// =====================================================
// MAIN APPLICATION
// =====================================================

void app_main(void)
{
    ESP_LOGI(TAG, "Starting ESP32 robotic arm");

    init_adc();
    init_servos();

    // Initialize MPU6050; keep potentiometer control
    // running even if the sensor cannot be reached.
    esp_err_t imu_status = init_mpu6050();

    if (imu_status != ESP_OK) {
        ESP_LOGE(TAG, "MPU6050 initialization failed");
        ESP_LOGW(TAG, "Continuing with potentiometers only");
    }

    while (1) {
        int pot[POT_COUNT];
    int angle[POT_COUNT];

    read_potentiometers(pot);

    for (int i = 0; i < POT_COUNT; i++) {
        angle[i] = pot_to_angle(pot[i]);
    }

    float roll = 0.0f;
    float pitch = 0.0f;

    if (imu_status == ESP_OK) {
        if (read_mpu6050_angles(&roll, &pitch) == ESP_OK) {
            
            // Calculate error between desired target and actual sensor pitch
            float pitch_error = TARGET_PITCH - pitch;

            // Apply proportional correction to Servo 5 (index 4)
            // This overrides or offsets the raw potentiometer input with feedback correction
            float correction = pitch_error * KP_GAIN;
            int corrected_servo5_angle = angle[4] + (int)correction;

            // Clamp the final angle to safe mechanical limits (0 to 180)
            angle[4] = clamp_angle(corrected_servo5_angle);
        }
    }

    // Drive all servos
    set_servo_angle(0, angle[0]);
    set_servo_angle(1, angle[1]);
    set_servo_angle(2, angle[2]);
    set_servo_angle(3, angle[3]);
    set_servo_angle(4, angle[4]);

    vTaskDelay(pdMS_TO_TICKS(50));
    }
}
