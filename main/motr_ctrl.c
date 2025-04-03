#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
/* 
*  NOW:   FIGURE OUT how i will implement IMU / pid loop 
*/

static const char *IMU_TAG = "imu read";
static const char *TAG = "init";
#define MPU9250_SENSOR_ADDR         0x68        /* MPU9250 sensor */
#define MPU9250_WHO_AM_I_REG_ADDR   0x75        /* "who am I" register */
#define MPU9250_PWR_MGMT_1_REG_ADDR 0x6B        /* power management register */
#define MPU9250_RESET_BIT           7


i2c_master_bus_handle_t bus_handle;
i2c_master_dev_handle_t device_handle;

static void motor_init(){
    // set LEDC timer 
    ledc_timer_config_t time0 = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 10000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&time0));
    // set LEDC channel0 (PWM1)
    ledc_channel_config_t chan0 = {
        .gpio_num = GPIO_NUM_18,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan0));
    // set LEDC channel1 (PWM2)
    ledc_channel_config_t chan1 = {
        .gpio_num = GPIO_NUM_19,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = LEDC_TIMER_0,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&chan1));
    // configure GPIO pins
    gpio_config_t gp0 = {
        .pin_bit_mask = (1ULL<<10),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config_t gp1 = {
        .pin_bit_mask = (1ULL<<11),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config_t gp2 = {
        .pin_bit_mask = (1ULL<<12),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config_t gp3 = {
        .pin_bit_mask = (1ULL<<13),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    ESP_ERROR_CHECK(gpio_config(&gp0));
    ESP_ERROR_CHECK(gpio_config(&gp1));
    ESP_ERROR_CHECK(gpio_config(&gp2));
    ESP_ERROR_CHECK(gpio_config(&gp3));
    gpio_dump_io_configuration(stdout, (1ULL << 10) | (1ULL << 11) | (1ULL << 12) | (1ULL << 13));
}

static void i2c_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *device_handle){
    i2c_master_bus_config_t bus_config = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = GPIO_NUM_21,
        .scl_io_num = GPIO_NUM_22,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true
    };
    ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));

    i2c_device_config_t device_config = {
        .dev_addr_length = I2C_ADDR_BIT_7,
        .device_address = 0x67,
        .scl_speed_hz = 10000
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &device_config, device_handle));
}

void set_motor_direction(int motor, bool forward){
    if (motor == 0) {  // Motor A
        gpio_set_level(GPIO_NUM_10, forward ? 1 : 0);
        gpio_set_level(GPIO_NUM_11, forward ? 0 : 1);
    } else if (motor == 1) {  // Motor B
        gpio_set_level(GPIO_NUM_12, forward ? 1 : 0);
        gpio_set_level(GPIO_NUM_13, forward ? 0 : 1);
    }
}

void set_motor_speed(int motor, uint32_t speed){
    if (motor == 0) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, speed);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    } else if (motor == 1) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, speed);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    }
}

static esp_err_t mpu9250_register_read(i2c_master_dev_handle_t device_handle, uint8_t reg_addr, uint8_t *data, size_t len){
    return i2c_master_transmit_receive(device_handle, &reg_addr, 1, data, len, 1000 / portTICK_PERIOD_MS);
}

void imu_task(void *arg){
    int16_t gyro_x, gyro_y, gyro_z;
    uint8_t raw_data[6];

    while (1){
        mpu9250_register_read(device_handle, MPU9250_SENSOR_ADDR, raw_data, 6);
        gyro_x = (raw_data[0] << 8) | raw_data[1];
        gyro_y = (raw_data[2] << 8) | raw_data[3];
        gyro_z = (raw_data[4] << 8) | raw_data[5];

        ESP_LOGI(IMU_TAG, "Gyro:  X=%d Y=%d Z=%d", gyro_x, gyro_y, gyro_z);

        vTaskDelay(pdMS_TO_TICKS(100)); 
    }
}

/* Attempting to read IMU measurement for processing */
void app_main(void){
    motor_init();
    ESP_LOGI(TAG, "GPIO/LEDC initialized succesfully");
    i2c_init(&bus_handle, &device_handle);
    ESP_LOGI(TAG, "I2C initialized succesfully");

    xTaskCreate(imu_task, "imu_task", 4096, NULL, 5, NULL);
}