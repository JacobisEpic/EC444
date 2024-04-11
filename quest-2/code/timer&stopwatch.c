/* This is an example of a single timer interrupt using the ESP GPTimer APIs
   The example is pulled from 
   https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/peripherals/gptimer.html#
   The timer triggers an event every 10s which is sent from the ISR/callback via an event queue
   The Event in queue signals action to turn on LED for 1s
   Runs in latest ESP32 distro as of 2023/09/22
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>             
#include <inttypes.h>           
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"     
#include "esp_log.h"            // for error logging
#include "esp_system.h"         
#include "driver/uart.h"
#include "driver/gptimer.h"     
#include "driver/gpio.h"        
#include "driver/ledc.h"  
#include "sdkconfig.h"
#include "esp_vfs_dev.h"
#include "driver/i2c.h"

#define ONBOARD   13  // onboard LED
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<ONBOARD)
#define GPIO_OUTPUT_PIN_SEL (1ULL<<ONBOARD)
#define btn 39
#define modeBtn 34
#define BUZZER_GPIO 25
#define GPIO_INPUT_IO_1    39
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL  1ULL<<GPIO_INPUT_IO_1

// Global flags and variables
bool flag = false;
int count = 0;
bool SWcounting = false;
bool timerCounting = false;
int timer = 0;
int mode = 0; // mode 0 is clock, mode 1 is stopwatch, mode 2 is timer

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
        vTaskDelay(200/ portTICK_PERIOD_MS);
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
//_____________________________________________________
// initialize alphanumeric display

// 14-Segment Display
#define SLAVE_ADDR                         0x70 // alphanumeric address
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
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST)
    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
}

// Utility function to test for I2C device address -- not used in deploy
int testConnection(uint8_t devAddr, int32_t timeout) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (devAddr << 1) | I2C_MASTER_WRITE, ACK_CHECK_EN);
    i2c_master_stop(cmd);
    int err = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return err;
}
// Utility function to scan for i2c device
static void i2c_scanner() {
    int32_t scanTimeout = 1000;
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            count++;
        }
    }
}

//////////////////////////////////////////////////////
// Turn on oscillator for alpha display
int alpha_oscillator() {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, OSC, ACK_CHECK_EN);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

// Set blink rate to off
int no_blink() {
  int ret;
  i2c_cmd_handle_t cmd2 = i2c_cmd_link_create();
  i2c_master_start(cmd2);
  i2c_master_write_byte(cmd2, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd2, HT16K33_BLINK_CMD | HT16K33_BLINK_DISPLAYON | (HT16K33_BLINK_OFF << 1), ACK_CHECK_EN);
  i2c_master_stop(cmd2);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd2, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd2);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}

// Set Brightness
int set_brightness_max(uint8_t val) {
  int ret;
  i2c_cmd_handle_t cmd3 = i2c_cmd_link_create();
  i2c_master_start(cmd3);
  i2c_master_write_byte(cmd3, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd3, HT16K33_CMD_BRIGHTNESS | val, ACK_CHECK_EN);
  i2c_master_stop(cmd3);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd3, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd3);
  vTaskDelay(200 / portTICK_PERIOD_MS);
  return ret;
}
/////////////////////////////////////////////////////////////////////////////////////////
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
        i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
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

    i2c_example_master_init();
    i2c_scanner();

  // Create task to handle timer-based events 
  xTaskCreate(timer_evt_task, "timer_evt_task", 2048, NULL, 5, NULL);

  // Create task to use LED based on flag
  xTaskCreate(led_task, "led_task", 2048, NULL, 5, NULL);

  // create task to detect button click
  xTaskCreate(button_function, "button_function", 2048, NULL, 5, NULL);

  // create alphanumeric display task
  xTaskCreate(test_alpha_display, "test_alpha_display", 4096, NULL, 5, NULL);
}
