// All the includes and headers for the accelerometer, thermistor, buttons, alphanumeric display, buzzer, and the driver for the buzzer
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <inttypes.h>  
#include "driver/i2c.h"
#include "./ADXL343.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "driver/uart.h"
#include "esp_vfs_dev.h"	// This is associated with VFS -- virtual file system interface and abstraction -- see the docs                     
#include "freertos/queue.h"     
#include "esp_log.h"            // for error logging
#include "esp_system.h"         
#include "driver/gptimer.h"           
#include "driver/ledc.h"  
#include "sdkconfig.h"
#include "esp_vfs_dev.h"

#define SERIAL_PORT UART_NUM_0 // Replace with the UART port you want to use
#define BUF_SIZE (1024)

//14-Segment Display
#define DISPLAY_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd

// Master I2C
#define I2C_EXAMPLE_MASTER_SCL_IO          22   // gpio number for i2c clk
#define I2C_EXAMPLE_MASTER_SDA_IO          23   // gpio number for i2c data
#define I2C_EXAMPLE_MASTER_NUM             I2C_NUM_0  // i2c port
#define I2C_EXAMPLE_MASTER_TX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_RX_BUF_DISABLE  0    // i2c master no buffer needed
#define I2C_EXAMPLE_MASTER_FREQ_HZ         100000     // i2c master clock freq
#define WRITE_BIT                          I2C_MASTER_WRITE // i2c master write
#define READ_BIT                           I2C_MASTER_READ  // i2c master read
#define ACK_CHECK_EN                       true // i2c master will check ack
#define ACK_CHECK_DIS                      false// i2c master will not check ack
#define ACK_VAL                            0x00 // i2c ack value
#define NACK_VAL                           0xFF // i2c nack value

// Thermistor
#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#define SERIESRESISTOR 10000
#define BCOEFFICIENT 3950
#define TEMPERATURENOMINAL 25
#define THERMISTORNOMINAL 10000 

// ADXL343
#define ACCEL_ADDR                         ADXL343_ADDRESS // 0x53
#define STEP_THRESHOLD 2.1

// Buzzer
#define ONBOARD   13  // onboard LED
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<ONBOARD)
#define GPIO_OUTPUT_PIN_SEL (1ULL<<ONBOARD)
#define btn 39
#define modeBtn 34
#define BUZZER_GPIO 25
#define GPIO_INPUT_IO_1    39
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL  1ULL<<GPIO_INPUT_IO_1

// Global variables go here
float lastX;
float currX;
int numSteps = 0;
float steinhart;
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_2;
uint16_t displayBuffer[8]; // Global structure to store the string
char charList[93] = {'!', '"', '#', '$', '%', '&', '\'', '(', ')', '*', '+', ',', '-', '.', '/', '0', '1', '2', '3', '4', '5', '6', '7', '8', '9', ':', ';', '<', '=', '>', '?', '@', 'A', 'B', 'C', 'D', 'E', 'F', 'G', 'H', 'I', 'J', 'K', 'L', 'M', 'N', 'O', 'P', 'Q', 'R', 'S', 'T', 'U', 'V', 'W', 'X', 'Y', 'Z', '[', ']', '^', '_', '`', 'a', 'b', 'c', 'd', 'e', 'f', 'g', 'h', 'i', 'j', 'k', 'l', 'm', 'n', 'o', 'p', 'q', 'r', 's', 't', 'u', 'v', 'w', 'x', 'y', 'z', '{', '|', '}', '~'}; 
uint16_t alphaFontTable[93] = {0b0000000000000110, 0b0000001000100000, 0b0001001011001110, 0b0001001011101101, 0b0000110000100100, 0b0010001101011101, 0b0000010000000000, 0b0010010000000000, 0b0000100100000000, 0b0011111111000000, 0b0001001011000000, 0b0000100000000000, 0b0000000011000000, 0b0100000000000000, 0b0000110000000000, 0b0000110000111111, 0b0000000000000110, 0b0000000011011011, 0b0000000010001111, 0b0000000011100110, 0b0010000001101001, 0b0000000011111101, 0b0000000000000111, 0b0000000011111111, 0b0000000011101111, 0b0001001000000000, 0b0000101000000000, 0b0010010000000000, 0b0000000011001000, 0b0000100100000000, 0b0001000010000011, 0b0000001010111011, 0b0000000011110111, 0b0001001010001111, 0b0000000000111001, 0b0001001000001111, 0b0000000011111001, 0b0000000001110001, 0b0000000010111101, 0b0000000011110110, 0b0001001000001001, 0b0000000000011110, 0b0010010001110000, 0b0000000000111000, 0b0000010100110110, 0b0010000100110110, 0b0000000000111111, 0b0000000011110011, 0b0010000000111111, 0b0010000011110011, 0b0000000011101101, 0b0001001000000001, 0b0000000000111110, 0b0000110000110000, 0b0010100000110110, 0b0010110100000000, 0b0001010100000000, 0b0000110000001001, // Z
    0b0000000000111001, // [ 
    0b0000000000001111, // ]
    0b0000110000000011, // ^
    0b0000000000001000, // _
    0b0000000100000000, // `
    0b0001000001011000, // a
    0b0010000001111000, // b
    0b0000000011011000, // c
    0b0000100010001110, // d
    0b0000100001011000, // e
    0b0000000001110001, // f
    0b0000010010001110, // g
    0b0001000001110000, // h
    0b0001000000000000, // i
    0b0000000000001110, // j
    0b0011011000000000, // k
    0b0000000000110000, // l
    0b0001000011010100, // m
    0b0001000001010000, // n
    0b0000000011011100, // o
    0b0000000101110000, // p
    0b0000010010000110, // q
    0b0000000001010000, // r
    0b0010000010001000, // s
    0b0000000001111000, // t
    0b0000000000011100, // u
    0b0010000000000100, // v
    0b0010100000010100, // w
    0b0010100011000000, // x
    0b0010000000001100, // y
    0b0000100001001000, // z
    0b0000100101001001, // {
    0b0001001000000000, // |
    0b0010010010001001,
    0b0000010100100000};
bool flag = false;
int count = 0;
bool SWcounting = false;
bool timerCounting = false;
int timer = 0;
int mode = 0; // mode 0 is clock, mode 1 is stopwatch, mode 2 is timer
uint16_t alphafonttable[] = {
      0b0000110000111111, // 0
      0b0000000000000110, // 1
      0b0000000011011011, // 2
      0b0000000010001111, // 3
      0b0000000011100110, // 4
      0b0010000001101001, // 5
      0b0000000011111101, // 6
      0b0000000000000111, // 7
      0b0000000011111111, // 8
      0b0000000011101111, // 9
};
uint16_t alphafonttableMin[] = {
      0b1100110000111111, // 0
      0b1100000000000110, // 1
      0b1100000011011011, // 2
      0b1100000010001111, // 3
      0b1100000011100110, // 4
      0b1110000001101001, // 5
      0b1100000011111101, // 6
      0b1100000000000111, // 7
      0b1100000011111111, // 8
      0b1100000011101111, // 9
};
u_int16_t displaybuffer [4];
int minutes = 0;
int seconds = 0;
int tensPlaceSec = 0;
int onesPlaceSec = 0;
int tensPlaceMin = 0;
int onesPlaceMin = 0;
int displayNum = 0;

// button interupt 
static void IRAM_ATTR gpio_isr_handler(void* arg) // Interrupt handler for your GPIO
{
  flag ^= 1;
}

// button init as interrupt
static void btn_init()
{
  gpio_config_t io_conf;
  //interrupt of rising edge
  io_conf.intr_type = GPIO_INTR_POSEDGE;
  //bit mask of the pins, use GPIO4 here
  io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
  //set as input mode
  io_conf.mode = GPIO_MODE_INPUT;
  //enable pull-up mode
  io_conf.pull_up_en = 1;
  gpio_config(&io_conf);
  gpio_intr_enable(GPIO_INPUT_IO_1 );
  //install gpio isr service
  gpio_install_isr_service(ESP_INTR_FLAG_LEVEL3);
  //hook isr handler for specific gpio pin
  gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
  
  // gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);
  gpio_set_direction(modeBtn,GPIO_MODE_INPUT);
  esp_rom_gpio_pad_select_gpio(BUZZER_GPIO);
  gpio_set_direction(BUZZER_GPIO, GPIO_MODE_OUTPUT);
}

int presses = 0;

// stopwatch and timer btn detection
static void button_function(){
    while(1) {
        if (gpio_get_level(modeBtn) == 1) {     // increment the mode if the mode button is pressed
            mode++;
        }
        if ((gpio_get_level(btn) == 1) && (mode % 3 == 1)){     // mode 1 
            presses++;      
            SWcounting = !SWcounting;
            flag  = !flag;
            if (presses % 3 == 0) {
                count = 0;
                SWcounting = false;
            }
        }
        if (gpio_get_level(btn) == 1 && (mode % 3 == 2)){       // mode 2
            timerCounting = true;
            flag  = !flag;
            timer = timer + 30;         
        }
        vTaskDelay(150/ portTICK_PERIOD_MS);
    }
}

// Initialize GPIO for LED signals
static void led_init() {
    gpio_config_t io_conf = {};        // zero-initialize the config structure
    io_conf.mode = GPIO_MODE_OUTPUT;   // set as output mode
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL; // bit mask of the pins that you want to set
    io_conf.pull_down_en = 0;          // disable pull-down mode
    io_conf.pull_up_en = 0;            // disable pull-up mode
    gpio_config(&io_conf);             // configure GPIO with the given settings
}

// A simple structure for queue elements
typedef struct {
    uint64_t event_count;
} example_queue_element_t;

// Create a FIFO queue for timer-based events
example_queue_element_t ele;
QueueHandle_t timer_queue;

// System log tags -- get logged when things happen, for debugging 
static const char *TAG_TIMER = "ec444: timer";

int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( DISPLAY_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( DISPLAY_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( DISPLAY_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

// Timer interrupt handler -- callback timer function -- from GPTimer guide example
static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t timer_queue1 = (QueueHandle_t)user_data;    // represents state info passed to callback, if needed
    example_queue_element_t ele = {
          .event_count = edata->count_value                   // Retrieve count value and send to queue
      };
    xQueueSendFromISR(timer_queue1, &ele, &high_task_awoken); // Puts data into queue and alerts other recipients
    return (high_task_awoken == pdTRUE);  		      // pdTRUE indicates data posted successfully
}

// Timer configuration -- from GPTimer guide example
static void alarm_init() {
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer)); // instantiates timer
  
    gptimer_event_callbacks_t cbs = { // Set alarm callback
      .on_alarm = timer_on_alarm_cb,  // This is a specific supported callback from callbacks list
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, timer_queue)); // This registers the callback
    ESP_ERROR_CHECK(gptimer_enable(gptimer));                                      // Enables timer interrupt ISR

    ESP_LOGI(TAG_TIMER, "Start timer, update alarm value dynamically and auto reload"); 
    gptimer_alarm_config_t alarm_config = { // Configure the alarm 
      .reload_count = 0,                    // counter will reload with 0 on alarm event
      .alarm_count = 1*1000000,            // period = 1*1s = 1s
      .flags.auto_reload_on_alarm = true,   // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));  // this enacts the alarm config
    ESP_ERROR_CHECK(gptimer_start(gptimer));                            // this starts the timer
}

// LED task to light LED based on alarm flag
void led_task(){
  while(1) {
    if (flag) {                             // Uses flag set by Timer task; could be moved to there
        gpio_set_level(ONBOARD, 1);
	flag = !flag;
      }
    vTaskDelay(500 / portTICK_PERIOD_MS);
    gpio_set_level(ONBOARD, 0);
  }
}

// Timer task -- what to do when the timer alarm triggers 
static void timer_evt_task(void *arg) {
  while (1) {
    // Transfer from queue and do something if triggered
    if (xQueueReceive(timer_queue, &ele, pdMS_TO_TICKS(2000))) {
        if (SWcounting) {
            // stopwatch
            count++;
            flag = true;           // Set a flag to be used elsewhere 
        }
        if (timerCounting) {
            // timer
            timer--;
            flag = true;           // Set a flag to be used elsewhere 
            if (timer == 0) {
                timerCounting= false; // stop counting once timer reaches the end
                // Turn on the buzzer (make it buzz)
                gpio_set_level(BUZZER_GPIO, 1);
                vTaskDelay(3000 / portTICK_PERIOD_MS); // Buzz for 1 second
                gpio_set_level(BUZZER_GPIO, 0);
            }
      }
    }
  }
}

static void test_alpha_display() {
    // Debug
    int ret;

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    // Set display blink off
    ret = no_blink();
    ret = set_brightness_max(0xF);

    // Continually writes the same command
    while (1) {
        if (mode % 3 == 1) {        // stopwatch mode
            displayNum = count;
        }
        if (mode % 3 == 2) {
            displayNum = timer;     // timer mode
        }
        minutes = displayNum / 60;
        seconds = displayNum % 60;

        tensPlaceMin = minutes / 10;
        onesPlaceMin = minutes % 10;

        tensPlaceSec = seconds / 10;
        onesPlaceSec = seconds %10;

        displaybuffer[0] = alphafonttable[tensPlaceMin];
        displaybuffer[1] = alphafonttableMin[onesPlaceMin];
        displaybuffer[2] = alphafonttable[tensPlaceSec];
        displaybuffer[3] = alphafonttable[onesPlaceSec];

        // send count characters to display over I2C
        if (mode % 3 == 1 || mode % 3 == 2) 
        {
            i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
            i2c_master_start(cmd4);
            i2c_master_write_byte(cmd4, ( DISPLAY_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
            for (uint8_t i=0; i<4; i++) {
                i2c_master_write_byte(cmd4, displaybuffer[i] & 0xFF, ACK_CHECK_EN);
                i2c_master_write_byte(cmd4, displaybuffer[i] >> 8, ACK_CHECK_EN);
            }
            i2c_master_stop(cmd4);
            ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
            i2c_cmd_link_delete(cmd4);
            vTaskDelay(250/ portTICK_PERIOD_MS);
            }
    }
}

// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
  // Debug
  int err;

  // Port configuration
  int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

  /// Define I2C configurations
  i2c_config_t conf;
  conf.mode = I2C_MODE_MASTER;                              // Master mode
  conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
  conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
  conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
  conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
   conf.clk_flags = 0; 
  err = i2c_param_config(i2c_master_port, &conf);           // Configure

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);

  // Data in MSB mode
  i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

static void i2c_example_master_init(){
    // Debug
    int err;

    // Port configuration
    int i2c_master_port = I2C_EXAMPLE_MASTER_NUM;

    /// Define I2C configurations
    i2c_config_t conf;
    conf.mode = I2C_MODE_MASTER;                              // Master mode
    conf.sda_io_num = I2C_EXAMPLE_MASTER_SDA_IO;              // Default SDA pin
    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.scl_io_num = I2C_EXAMPLE_MASTER_SCL_IO;              // Default SCL pin
    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;                  // Internal pullup
    conf.master.clk_speed = I2C_EXAMPLE_MASTER_FREQ_HZ;       // CLK frequency
    conf.clk_flags = 0;                                     // <-- UNCOMMENT IF YOU GET ERRORS (see readme.md)
    err = i2c_param_config(i2c_master_port, &conf);           // Configure

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Write one byte to register
void writeRegister(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ACCEL_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); // Register that will be written to
    i2c_master_write_byte(cmd, data, ACK_CHECK_DIS); // Write one byte
    i2c_master_stop(cmd);
    int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// Read register
uint8_t readRegister(uint8_t reg) {
    // Reading 1 byte from reg
    uint8_t data = 0;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ACCEL_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); // Register that will be read from
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( ACCEL_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
    i2c_master_read_byte(cmd, &data, ACK_CHECK_DIS); // Read data from register
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    return data;
}

// read 16 bits (2 bytes)
int16_t read16(uint8_t reg) {
    uint8_t data1, data2;
    data1 = readRegister(reg);
    if (reg == 41)
    {
        data2 = 0;
    }
    else
    {
        data2 = readRegister(reg+1);
    }
    return (((int16_t)data2 << 8) | data1);
}

void setRange(range_t range) {
  /* Red the data format register to preserve bits */
  uint8_t format = readRegister(ADXL343_REG_DATA_FORMAT);

  /* Update the data rate */
  format &= ~0x0F;
  format |= range;

  /* Make sure that the FULL-RES bit is enabled for range scaling */
  format |= 0x08;

  /* Write the register back to the IC */
  writeRegister(ADXL343_REG_DATA_FORMAT, format);

}

range_t getRange(void) {
  /* Red the data format register to preserve bits */
  return (range_t)(readRegister(ADXL343_REG_DATA_FORMAT) & 0x03);
}

dataRate_t getDataRate(void) {
  return (dataRate_t)(readRegister(ADXL343_REG_BW_RATE) & 0x0F);
}

// function to get acceleration
void getAccel(float * xp, float *yp, float *zp) {
  *xp = read16(ADXL343_REG_DATAX0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *yp = read16(ADXL343_REG_DATAY0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
  *zp = read16(ADXL343_REG_DATAZ0) * ADXL343_MG2G_MULTIPLIER * SENSORS_GRAVITY_STANDARD;
 //printf("X: %.2f \t Y: %.2f \t Z: %.2f\n", *xp, *yp, *zp);
}

static void read_temp(){
      uint32_t adc_reading = 0;
      //Multisampling
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
      //Convert adc_reading to voltage in mV
      uint32_t voltage = esp_adc_cal_raw_to_voltage(adc_reading, adc_chars);
      //Converting adc_reading to Celsius
      float temp = adc_reading;
      temp = 4095 / temp - 1;
      temp = SERIESRESISTOR / temp;
      
      steinhart = temp / THERMISTORNOMINAL;
      steinhart = log(steinhart);
      steinhart /= BCOEFFICIENT;
      steinhart += 1.0 / (TEMPERATURENOMINAL + 273.15);
      steinhart = 1.0 / steinhart; 
      steinhart -= 273.15;
}

// Task to continuously poll acceleration and calculate roll and pitch
static void test_adxl343() {
  while (1) {
    lastX = currX;
    float xVal, yVal, zVal;
    getAccel(&xVal, &yVal, &zVal);
    currX = xVal;
    if (fabs(currX - lastX) > STEP_THRESHOLD)
    {
      numSteps++;
    }
    read_temp();
    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

static void timed_print()
{
  while(1)
  {
    vTaskDelay(10000 / portTICK_PERIOD_MS); // Waiting 10 seconds
    printf("t: %f\n", steinhart); // Printing temperature
    vTaskDelay(10 / portTICK_PERIOD_MS);
    printf("s: %d\n", numSteps); // Printing total steps
  }
}

static void init_temp(){
  //Check if Two Point or Vref are burned into eFuse

  //Configure ADC
  if (unit == ADC_UNIT_1) {
      adc1_config_width(ADC_WIDTH_BIT_12);
      adc1_config_channel_atten(channel, atten);
  } else {
      adc2_config_channel_atten((adc2_channel_t)channel, atten);
  }

    //Characterize ADC
  adc_chars = calloc(1, sizeof(esp_adc_cal_characteristics_t));
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(unit, atten, ADC_WIDTH_BIT_12, DEFAULT_VREF, adc_chars);
}

int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}

static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        if (testConnection(i, scanTimeout) == ESP_OK) {
            count++;
        }
    }
}

static void init_check()
{
  // Routine
  i2c_master_init();
  i2c_scanner();
  // Disable interrupts
  writeRegister(ADXL343_REG_INT_ENABLE, 0);

  // Set range
  setRange(ADXL343_RANGE_16_G);
  // Display range

  // Display data rate

  // Enable measurements
  writeRegister(ADXL343_REG_POWER_CTL, 0x08);
}

void init_i2c()
{
    int ret;

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    // Set display blink off
    ret = no_blink();
    ret = set_brightness_max(0xF);
}

void str_to_bin(char* input, size_t len)
{
    // Populating the global displaybuffer array with binary
    int tableIndex = 0;
    for (int j = 0; j < len; j++)   
    {
        // Convert letter to binary
        for (int i = 0; i < 93; i++)
        {
            // Finding correct index in table
            if (input[j] == charList[i])
            {
                tableIndex = i;
                break;
            }
        }
        displayBuffer[j] = alphaFontTable[tableIndex];
    }
}

void turn_off_all(size_t len)
{
    for (int i = 0; i < len; i++)
      {
        displayBuffer[i] = 0b0000000000000000;
      }
      i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, ( DISPLAY_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
      for (uint8_t i=0; i< (uint8_t)len; i++) {
            i2c_master_write_byte(cmd4, displayBuffer[i] & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, displayBuffer[i] >> 8, ACK_CHECK_EN);
        }
        i2c_master_stop(cmd4);
        int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd4);
}

void init_drivers()
{
    uart_config_t uart_config = {
        .baud_rate = 115200, 
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };

    uart_param_config(SERIAL_PORT, &uart_config);
    uart_driver_install(SERIAL_PORT, BUF_SIZE, 0, 0, NULL, 0);

    /* Tell VFS to use UART driver */
    esp_vfs_dev_uart_use_driver(UART_NUM_0);

    i2c_example_master_init();
    i2c_scanner();

    init_i2c();
}

void app_main() {
    // Timer queue initialize 
  timer_queue = xQueueCreate(10, sizeof(example_queue_element_t));
  if (!timer_queue) {
    ESP_LOGE(TAG_TIMER, "Creating queue failed");
    return;
  }
    // Initialize all the things
    btn_init();
    led_init();
    alarm_init();
    init_drivers();
    init_check();
    init_temp();

    xTaskCreate(test_adxl343,"test_adxl343", 4096, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(timed_print, "timed_print", 4096, NULL , configMAX_PRIORITIES, NULL);
    xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, configMAX_PRIORITIES-4, NULL);
    xTaskCreate(button_function, "button_function", 2048, NULL, configMAX_PRIORITIES-3, NULL);
    xTaskCreate(test_alpha_display, "test_alpha_display", 4096, NULL, configMAX_PRIORITIES-2, NULL);

    uint8_t* data = (uint8_t*) malloc(BUF_SIZE);
    char* time = (char*) malloc(5);
    while (1) {
        int len = uart_read_bytes(SERIAL_PORT, data, BUF_SIZE, 20 / portTICK_PERIOD_MS);
        if (len > 0) {
            data[len] = '\0'; // Null-terminate the received data
            *(time) = (char) *(data);
            *(time + 1) = (char) *(data + 1);
            *(time + 2) = (char) *(data + 3);
            *(time + 3) = (char) *(data + 4);
            *(time + 4) = '\0';
            str_to_bin(time, len);
            if (mode % 3 == 0)
            {
                i2c_cmd_handle_t cmd5 = i2c_cmd_link_create();
                i2c_master_start(cmd5);
                i2c_master_write_byte(cmd5, ( DISPLAY_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
                i2c_master_write_byte(cmd5, (uint8_t)0x00, ACK_CHECK_EN);
                for (uint8_t i=0; i < (uint8_t)4; i++) {
                    i2c_master_write_byte(cmd5, displayBuffer[i] & 0xFF, ACK_CHECK_EN);
                    i2c_master_write_byte(cmd5, displayBuffer[i] >> 8, ACK_CHECK_EN);
                }
                i2c_master_stop(cmd5);
                int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd5, 1000 / portTICK_PERIOD_MS);
                i2c_cmd_link_delete(cmd5);
            }
        }
    }
    free(data);
}
