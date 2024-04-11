//includes go here
#include <stdio.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_log.h"
#include "led_strip.h"
#include "sdkconfig.h"
#include "esp_adc_cal.h"
#include <math.h>   // for 'log'

//defines go here
#define BUTTON 15
#define GPIO_INPUT_IO_1       15
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1
#define LED1 12
#define LED2 27
#define LED3 33
#define LED4 32
#define BLINK_GPIO 12

// Thermistor Definitions
#define THERMISTORNOMINAL 10000
#define TEMPERATURENOMINAL 25
#define BCOEFFICIENT 3950
#define SERIESRESISTOR 10000
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling

//global variables
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_channel_t c = ADC_CHANNEL_3;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t a = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;

static uint8_t s_led_state = 0;
int flag = 0;
int lastFlag = 0;
int toggleButton = 0;
float thermBefore = 0.0;
float thermAfter = 0.0;
int photoBefore = 0;
int photoAfter = 0;
bool thermTrigger = false;
bool photoTrigger = false;
bool buttonTrigger = false;
TickType_t lastButtonPressTick = 0; // For debouncing

//configures leds
static void configure_led(void)
{
    gpio_reset_pin(LED1);
    gpio_reset_pin(LED2);
    gpio_reset_pin(LED3);
    gpio_reset_pin(LED4);
 
    gpio_set_direction(LED1, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED2, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED3, GPIO_MODE_OUTPUT);
    gpio_set_direction(LED4, GPIO_MODE_OUTPUT);
}

//interrupt handler
static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    TickType_t currentTick = xTaskGetTickCount();
    if (currentTick - lastButtonPressTick > pdMS_TO_TICKS(50)) {
        // lastFlag = flag;
        flag ^= 1;
        lastButtonPressTick = currentTick;
    }
}

//inits button
static void init_button(void)
{
    gpio_config_t io_conf;
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pull_up_en = 0;
    gpio_config(&io_conf);
    gpio_intr_enable(GPIO_INPUT_IO_1 );
    gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
    gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
}

//button logic when pressed
void buttonLogic(void *param)
{
    while(1){
        if (lastFlag != flag){
            printf("Button pressed.\n");
            toggleButton = 1;
            lastFlag = flag;
        }
        vTaskDelay(pdMS_TO_TICKS(150)); 
    }
}

//thermistor
void thermistor(void) {
    while (1) {
        thermBefore = thermAfter;
    if (unit == ADC_UNIT_1) {
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(channel, atten);
    } else {
        adc2_config_channel_atten((adc2_channel_t)channel, atten);
    }
    
    adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
    esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
    uint32_t adc_reading = 0;
        for (int i = 0; i < NO_OF_SAMPLES; i++) {
            if (unit == ADC_UNIT_1) {
                adc_reading += adc1_get_raw((adc1_channel_t)channel);
            } else {
                int raw;
                adc2_get_raw((adc2_channel_t)channel, ADC_WIDTH_BIT_12, &raw);
                adc_reading += raw;
            }
        }
        adc_reading /= NO_OF_SAMPLES;
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        printf("Raw: %ld\tVoltage: %ldmV\n", adc_reading, voltage);

        float average = adc_reading;
        average = 4095 / average  - 1;
        average = SERIESRESISTOR / average;
        float steinhart = average / THERMISTORNOMINAL;
        steinhart = log(steinhart);
        steinhart /= BCOEFFICIENT;
        steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);
        steinhart = 1.0 / steinhart;
        steinhart -= 273.15;
        printf("Temperature: %f *C\n", steinhart);
        thermAfter = steinhart;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//photocell
void photocell(void){
    while(1){
        photoBefore = photoAfter;
        if (unit == ADC_UNIT_1) {
            adc1_config_width(ADC_WIDTH_BIT_12);
            adc1_config_channel_atten(c, a);
        } else {
            adc2_config_channel_atten((adc2_channel_t)c, a);
        }
        
        adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
        esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, a, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
            uint32_t adc_reading = 0;
            for (int i = 0; i < NO_OF_SAMPLES; i++) {
                if (unit == ADC_UNIT_1) {
                    adc_reading += adc1_get_raw((adc1_channel_t)c);
                } else {
                    int raw;
                    adc2_get_raw((adc2_channel_t)c, ADC_WIDTH_BIT_12, &raw);
                    adc_reading += raw;
                }
            }
            
        adc_reading /= NO_OF_SAMPLES;
        uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
        printf("Raw: %ld\tVoltage: %ldmV\n", adc_reading, voltage);
        photoAfter = voltage;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

//lights led
void lightLED() 
{
    gpio_set_level(LED1, thermTrigger);
    gpio_set_level(LED3, photoTrigger);
    gpio_set_level(LED4, buttonTrigger);
    gpio_set_level(LED2, (!thermTrigger && !photoTrigger && !buttonTrigger));
    vTaskDelay(pdMS_TO_TICKS(1000));
    thermTrigger = false;
    photoTrigger = false;
    buttonTrigger = false;
    toggleButton = false;
    gpio_set_level(LED1, 0);
    gpio_set_level(LED3, 0);
    gpio_set_level(LED4, 0);
    gpio_set_level(LED2, 0);
}

void mission(void *param)
{
    while(1){
        if (toggleButton == 1){
            buttonTrigger = true;
        }
        if (abs(photoAfter - photoBefore) > 20){
            photoTrigger = true;
        }
        if (fabs(thermAfter - thermBefore) > 0.5){
            thermTrigger = true;
        }
        lightLED();
        vTaskDelay(pdMS_TO_TICKS(2000));
    }
}

void app_main(void)
{
    configure_led();
    init_button();
    
    xTaskCreate(mission, "mission", 2048, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(buttonLogic, "buttonLogic", 2048, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(photocell, "photocell", 2048, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(thermistor, "thermistor", 2048, NULL, configMAX_PRIORITIES, NULL);
}
