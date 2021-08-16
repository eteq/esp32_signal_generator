#include <stdio.h>
#include <math.h>
#include <driver/adc.h>
#include <driver/dac.h>
#include <driver/gpio.h>
#include <esp_timer.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//ADC1_CHANNEL_6 -> GPIO 34

#define ADC_CHANNEL ADC1_CHANNEL_6
#define ADC_ATTEN ADC_ATTEN_DB_11
#define ADC_WIDTH ADC_WIDTH_BIT_12

#define ARRAY_SIZE 4096

#define GPIO_OUTPUT_PIN 21


int adc_raw_arr[ARRAY_SIZE];

void setup_adc1(void) {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
}

void setup_gpio(void) {
    gpio_config_t io_conf;

    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_OUTPUT;
    io_conf.pin_bit_mask = 1ULL << GPIO_OUTPUT_PIN;
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);

    gpio_set_level(GPIO_OUTPUT_PIN, 0);
}

void setup_cw(void) {
    dac_output_enable(DAC_CHANNEL_2);

    dac_cw_config_t cw_conf;

    cw_conf.en_ch = DAC_CHANNEL_2;
    cw_conf.scale = DAC_CW_SCALE_2;
    cw_conf.phase = DAC_CW_PHASE_0;
    cw_conf.freq = 2335;
    cw_conf.offset = 25;

    if (dac_cw_generator_config(&cw_conf) == ESP_OK) {
        dac_cw_generator_enable();
        printf("CW startup success.\n");
    } else {
        printf("CW startup failed.\n");
    };
}


double fill_adc1_arr(void) {
    // return value is dt per sample in microseconds

    int64_t t1, t2;
    t1 = esp_timer_get_time();
    int ao2 = ARRAY_SIZE / 2;

    for (int i=0; i < ARRAY_SIZE; i++ ) {
        adc_raw_arr[i] = adc1_get_raw((adc1_channel_t)ADC_CHANNEL);
    }
    t2 = esp_timer_get_time();
    return (double)(t2 - t1)/(double)ARRAY_SIZE;
}

void cw_main(void)
{
    double dt;

    printf("Hello world!\n");

    setup_adc1();
    adc1_get_raw((adc1_channel_t)ADC_CHANNEL);  // Might not be necessary

    setup_cw();


    while (1) {
        printf("Voltage raw: %d\n", adc1_get_raw((adc1_channel_t)ADC_CHANNEL));
        printf("Time since startup: %lld\n", esp_timer_get_time());

        printf("Reading ADC\n");
        dt = fill_adc1_arr();
        printf("Data:\n");

        for (int i=0; i < (ARRAY_SIZE-1); i++ ) {
            printf("%d,", adc_raw_arr[i]);
        }
        printf("%d\ndt per sample=%f\n", adc_raw_arr[ARRAY_SIZE-1], dt);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}

void rc_main(void)
{
    double dt;

    printf("Hello world!\n");

    setup_adc1();
    adc1_get_raw((adc1_channel_t)ADC_CHANNEL);  // Might not be necessary


    while (1) {
        printf("Voltage raw: %d\n", adc1_get_raw((adc1_channel_t)ADC_CHANNEL));
        printf("Time since startup: %lld\n", esp_timer_get_time());

        printf("Charging cap for 2 sec\n");
        gpio_set_level(GPIO_OUTPUT_PIN, 1);
        vTaskDelay(2000 / portTICK_PERIOD_MS);

        printf("Reading ADC\n");
        gpio_set_level(GPIO_OUTPUT_PIN, 0);
        dt = fill_adc1_arr();
        printf("Data:\n");

        for (int i=0; i < (ARRAY_SIZE-1); i++ ) {
            printf("%d,", adc_raw_arr[i]);
        }
        printf("%d\ndt per sample=%f\n", adc_raw_arr[ARRAY_SIZE-1], dt);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}



void app_main(void)
{
    cw_main();
}