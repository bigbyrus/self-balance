#include <stdio.h>
#include "driver/ledc.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "hal/adc_types.h"
#include "esp_adc/adc_oneshot.h"

/* 
*  NOW:   FIGURE OUT how i will implement IMU / pid loop 
*/

static const char *TAG = "JOYSTICK";
QueueHandle_t joystick_queue;
adc_oneshot_unit_handle_t adc1_handle;

typedef struct {
    int x_value;
    int y_value;
} joystick_data_t;

void joystick_task(void *arg) {
    joystick_data_t joystick;
    while (1) {
        joystick.x_value = adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &joystick.x_value);
        joystick.y_value = adc_oneshot_read(adc1_handle, ADC_CHANNEL_3, &joystick.y_value);

        // Send data to queue (non-blocking)
        if (xQueueSend(joystick_queue, &joystick, 10) == pdPASS) {
            ESP_LOGI(TAG, "Joystick X: %d, Y: %d", joystick.x_value, joystick.y_value);
        }

        // Read joystick every 100ms
        vTaskDelay(pdMS_TO_TICKS(100));  
    }
}

// Set motor direction
void set_motor_direction(int motor, bool forward) {
    if (motor == 0) {  // Motor A
        gpio_set_level(GPIO_NUM_10, forward ? 1 : 0);
        gpio_set_level(GPIO_NUM_11, forward ? 0 : 1);
    } else if (motor == 1) {  // Motor B
        gpio_set_level(GPIO_NUM_12, forward ? 1 : 0);
        gpio_set_level(GPIO_NUM_13, forward ? 0 : 1);
    }
}

// Set motor speed using PWM
void set_motor_speed(int motor, uint32_t speed) {
    if (motor == 0) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, speed);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
    } else if (motor == 1) {
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, speed);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
    }
}

void init(){
    // set LEDC timer 
    ledc_timer_config_t time0 = {
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .duty_resolution = LEDC_TIMER_10_BIT,
        .timer_num = LEDC_TIMER_0,
        .freq_hz = 10000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&time0);
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
    ledc_channel_config(&chan0);
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
    ledc_channel_config(&chan1);
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
    gpio_config(&gp0);
    gpio_config(&gp1);
    gpio_config(&gp2);
    gpio_config(&gp3);
    gpio_dump_io_configuration(stdout, (1ULL << 10) | (1ULL << 11) | (1ULL << 12) | (1ULL << 13));


    /* only one ADC unit needed for the two GPIO pins reading analog signals */
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config1, &adc1_handle));
    adc_oneshot_chan_cfg_t config = {
        .bitwidth = ADC_BITWIDTH_DEFAULT,
        .atten = ADC_ATTEN_DB_12,
    };
    /* configure GPIO 39 and GPIO 36 to act as ADC */
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_0, &config));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc1_handle, ADC_CHANNEL_3, &config));
}

/* Sending joystick data to queue every 100ms */
void app_main(void)
{
    init();
    xTaskCreate(joystick_task, "joystck_task", 2048, NULL, 5, NULL);
}
