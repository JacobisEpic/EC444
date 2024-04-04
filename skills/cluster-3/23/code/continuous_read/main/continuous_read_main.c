#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <math.h>   // for 'log'

#define DEFAULT_VREF    1100 
#define NO_OF_SAMPLES   64          

// Thermistor related definitions
#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25
#define BCOEFFICIENT 3950
#define SERIESRESISTOR 10000

static esp_adc_cal_characteristics_t *adc_chars;

static const adc_channel_t channel_thermistor = ADC_CHANNEL_6; // For example, ADC_CHANNEL_6
static const adc_channel_t channel_photocell = ADC_CHANNEL_3;  // Choose the correct channel, for example, ADC_CHANNEL_7

static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static void check_efuse(void);
static void print_char_val_type(esp_adc_cal_value_t val_type);

void app_main(void)
{
    //Check if Two Point or Vref are burned into eFuse
    check_efuse();

    //Configure ADC
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel_thermistor, atten);
        adc1_config_channel_atten(channel_photocell, atten); 
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel_thermistor, atten);
        adc2_config_channel_atten((adc2_channel_t)channel_photocell, atten);
    }
    //Characterize ADC
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    print_char_val_type(val_type);

    //Continuously sample ADC1
    while (1) {
        // Read from thermistor
        uint32_t adc_reading_thermistor = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading_thermistor += adc1_get_raw((adc1_channel_t)channel_thermistor);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel_thermistor, ADC_WIDTH_BIT_12, &raw);
                adc_reading_thermistor += raw;
            }
        }
        adc_reading_thermistor /= NO_OF_SAMPLES;
        float average = adc_reading_thermistor;
        average = 4095 / average  - 1;
        average = SERIESRESISTOR / average;
        float steinhart = average / THERMISTORNOMINAL;
        steinhart = log(steinhart);
        steinhart /= BCOEFFICIENT;
        steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);
        steinhart = 1.0 / steinhart;
        steinhart -= 273.15;
        // printf("Thermistor resistance: %f ohms\n", average);
        // printf("Temperature: %f *C\n", steinhart);

        vTaskDelay(pdMS_TO_TICKS(1000));
        // Now read from photocell
        uint32_t adc_reading_photocell = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading_photocell += adc1_get_raw((adc1_channel_t)channel_photocell);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel_photocell, ADC_WIDTH_BIT_12, &raw);
                adc_reading_photocell += raw;
            }
        }
        adc_reading_photocell /= NO_OF_SAMPLES;

        //Convert adc_reading to voltage in mV for photocell
        uint32_t voltage_photocell = esp_adc_cal_raw_to_voltage(adc_reading_photocell, adc_chars);
        printf("%f %ld\n", steinhart, adc_reading_photocell);

        // Delay before the next reading
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
static void check_efuse(void)
{
    //Check TP is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_TP) == ESP_OK) {
        printf("eFuse Two Point: Supported\n");
    } else {
        printf("eFuse Two Point: NOT supported\n");
    }

    //Check Vref is burned into eFuse
    if (esp_adc_cal_check_efuse(ESP_ADC_CAL_VAL_EFUSE_VREF) == ESP_OK) {
        printf("eFuse Vref: Supported\n");
    } else {
        printf("eFuse Vref: NOT supported\n");
    }
}

static void print_char_val_type(esp_adc_cal_value_t val_type)
{
    if (val_type == ESP_ADC_CAL_VAL_EFUSE_TP) {
        printf("Characterized using Two Point Value\n");
    } else if (val_type == ESP_ADC_CAL_VAL_EFUSE_VREF) {
        printf("Characterized using eFuse Vref\n");
    } else {
        printf("Characterized using Default Vref\n");
    }
}