#include <stdio.h>

#include "driver/dac.h"
#include "driver/touch_pad.h"
#include "driver/sigmadelta.h"
#include "esp_timer.h"
#include "esp_system.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"


//DAC channel 1 is GPIO25
//DAC channel 2 is GPIO26

#define TOUCH_PIN 7  //T7 = GPIO27 - on huzzah that's 6th pin from the battery connector
#define TOUCHPAD_FILTER_TOUCH_PERIOD 10
#define THRESH_SCALE 2 / 3

#define LED_GPIO GPIO_NUM_13

#define NFREQS 6
uint32_t FREQS[NFREQS] = {130, 260, 1250, 2500, 5000, 25000};


bool touching = false, increment = false;
int touch_i = 0;


esp_err_t setup_cw(uint32_t freq) {
    dac_output_enable(DAC_CHANNEL_2);

    dac_cw_config_t cw_conf;

    cw_conf.en_ch = DAC_CHANNEL_2;
    cw_conf.scale = DAC_CW_SCALE_1;
    cw_conf.phase = DAC_CW_PHASE_0;
    cw_conf.freq = freq;
    cw_conf.offset = 0;

    return dac_cw_generator_config(&cw_conf);

    if (dac_cw_generator_config(&cw_conf) == ESP_OK) {
        printf("CW setup success.\n");
    } else {
        printf("CW setup failed.\n");
    };
}


static void touch_isr(void *arg) {
    if ((touch_pad_get_status() > 0) & ! touching) {
        increment = true;
        touching = true;
    }
    touch_pad_clear_status();
}

void setup_touch(void) {
    uint16_t touch_value;

    touch_pad_init();
    touch_pad_set_fsm_mode(TOUCH_FSM_MODE_TIMER);
    touch_pad_set_voltage(TOUCH_HVOLT_2V7, TOUCH_LVOLT_0V5, TOUCH_HVOLT_ATTEN_1V);
    touch_pad_config(TOUCH_PIN, 0);
    touch_pad_filter_start(TOUCHPAD_FILTER_TOUCH_PERIOD);

    // threshold
    touch_pad_read_filtered(TOUCH_PIN, &touch_value);
    touch_pad_set_thresh(TOUCH_PIN, touch_value * THRESH_SCALE);

    touch_pad_isr_register(touch_isr, NULL);

    touch_pad_intr_enable();
}

esp_err_t setup_led(void) {
    sigmadelta_config_t sigmadelta_cfg = {
        .channel = SIGMADELTA_CHANNEL_0,
        .sigmadelta_prescale = 80,
        .sigmadelta_duty = 127,
        .sigmadelta_gpio = LED_GPIO,
    };
    return sigmadelta_config(&sigmadelta_cfg);
}


void app_main(void) {
    uint16_t touch_value, thresh_value;

    if (setup_led() == ESP_OK) {
        printf("LED setup success.\n");
    } else {
        printf("LED setup failed.\n");
    };

    setup_touch();
    touch_pad_get_thresh(TOUCH_PIN, &thresh_value);
    printf("Touch value threshold at startup: %d\n", thresh_value);

    if (setup_cw(FREQS[0]) == ESP_OK) {
        printf("CW setup success.\n");
    } else {
        printf("CW setup failed.\n");
    };

    dac_cw_generator_enable();
    sigmadelta_set_duty(SIGMADELTA_CHANNEL_0, -128);  // Initialize to off once things are actually going

    while (1) {
        if (increment) {
            touch_i++;

            if (setup_cw(FREQS[touch_i % NFREQS]) == ESP_OK) {
               printf("Set to new frequency %d\n", FREQS[touch_i % NFREQS]);
            } else {
               printf("Failed to set to new frequency %d\n", FREQS[touch_i % NFREQS]);
            };

            double touchi_frac = (touch_i % NFREQS) / (NFREQS-1.);
            sigmadelta_set_duty(SIGMADELTA_CHANNEL_0, 255 * (touchi_frac*touchi_frac) - 128);

            increment = false;
        }


        if (touching) {
            // allow for some debounce if need be
            vTaskDelay(10 / portTICK_PERIOD_MS);
            touch_pad_read_filtered(TOUCH_PIN, &touch_value);
            if (touch_value > thresh_value) {
                touching = false;
            }
        }

        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}