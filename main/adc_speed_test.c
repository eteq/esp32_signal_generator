#include <stdio.h>
#include <math.h>
#include <driver/adc.h>
#include <esp_timer.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

//ADC1_CHANNEL_6 -> GPIO 34

#define ADC_CHANNEL ADC1_CHANNEL_6
#define ADC_ATTEN ADC_ATTEN_DB_11
#define ADC_WIDTH ADC_WIDTH_BIT_12
#define ARRAY_SIZE 1024


int adc_raw_arr[ARRAY_SIZE];
int64_t time_arr[ARRAY_SIZE];

void setup_adc1(void) {
    adc1_config_width(ADC_WIDTH);
    adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN);
}

void read_adc1time_arr(void) {
    for (int i=0; i < ARRAY_SIZE; i++ ) {
        adc_raw_arr[i] = adc1_get_raw((adc1_channel_t)ADC_CHANNEL);
        time_arr[i] = esp_timer_get_time();
    }
}

float read_adc1_arr(void) {
    time_arr[0] = esp_timer_get_time();
    for (int i=0; i < ARRAY_SIZE; i++ ) {
        adc_raw_arr[i] = adc1_get_raw((adc1_channel_t)ADC_CHANNEL);
    }
    time_arr[1] = esp_timer_get_time();
    return (time_arr[1]-time_arr[0])/ARRAY_SIZE;
}

float avg_time_arr(void) {
    int64_t diffsum = 0;
    for (int i=0; i < (ARRAY_SIZE-1); i++ ) {
        diffsum += time_arr[i+1] - time_arr[i];
    }
    return diffsum/(ARRAY_SIZE-1);
}

float sd_time_arr(void) {
    float avg = avg_time_arr();
    float sum = 0;
    for (int i=0; i < (ARRAY_SIZE-1); i++ ) {
        float diff = time_arr[i+1] - time_arr[i] - avg;
        sum += diff*diff;
    }
    return sqrt(sum/(ARRAY_SIZE - 2));

}


void app_main(void)
{
    printf("Hello world!\n");

    setup_adc1();
    adc1_get_raw((adc1_channel_t)ADC_CHANNEL);  // Might not be necessary

    int i = 0;
    while (1) {
        printf("Voltage raw: %d\n", adc1_get_raw((adc1_channel_t)ADC_CHANNEL));
        printf("Time since startup: %lld\n", esp_timer_get_time());
        i++;

        read_adc1time_arr();

        printf("Array: %d, %d to %d\n", adc_raw_arr[0], adc_raw_arr[1], adc_raw_arr[ARRAY_SIZE-1]);
        printf("Times: %f, %f to %f\n", time_arr[0]*1e-6, time_arr[1]*1e-6, time_arr[ARRAY_SIZE-1]*1e-6);
        printf("Avg,SD dtime: %f,%f\n", avg_time_arr(), sd_time_arr());

        float dt = read_adc1_arr();

        printf("Array: %d, %d to %d\n", adc_raw_arr[0], adc_raw_arr[1], adc_raw_arr[ARRAY_SIZE-1]);
        printf("Dtime over sampling: %f\n", dt);

        vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
}
