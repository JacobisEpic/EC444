#include "sdkconfig.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_task_wdt.h"
#include "driver/pulse_cnt.h"
#include "driver/gpio.h"
#include "esp_sleep.h"
#include "driver/gptimer.h"
#include "driver/mcpwm_prelude.h"
#include "esp_private/esp_clk.h"
#include "driver/mcpwm_cap.h"
#include "driver/i2c.h"
#include <string.h>
#include <stdlib.h>
#include "driver/uart.h"
#include "driver/ledc.h"
#include <sys/param.h>
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include <stdio.h>
#include <inttypes.h>
#include "esp_vfs_dev.h"

#ifdef CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN
#include "addr_from_stdin.h"
#endif

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR "192.168.1.50"
#endif

#define PORT 3333

static const char *TAG = "example";

// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angleww

// ESC and steering servo
#define ESC_GPIO             27        //GPIO for esc
#define STEER_GPIO           12       //GPIO for steer
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms

#define EXAMPLE_PCNT_HIGH_LIMIT 100
#define EXAMPLE_PCNT_LOW_LIMIT  -100

// Ultrasonic Sensor defines
//switching speed sensor from 39 (A3) to 14
#define EXAMPLE_EC11_GPIO_A 14
#define EXAMPLE_EC11_GPIO_B 2
#define HC_SR04_LEFT_TRIG  4
#define HC_SR04_LEFT_ECHO  36
#define HC_SR04_RIGHT_TRIG 25
#define HC_SR04_RIGHT_ECHO 26

// Lidar defines
#define LIDAR_I2C_NUM I2C_NUM_0
#define LIDAR_ADDRESS 0x62
#define REG_ACQ_COMMAND 0x00
#define REG_STATUS 0x01
#define REG_FULL_DIST_LOW 0x10
#define REG_FULL_DIST_HIGH 0x11
#define ACQ_COMMAND 0x04

// Display defines
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<ONBOARD)
#define SLAVE_ADDR                         0x70 // alphanumeric address
#define OSC                                0x21 // oscillator cmd
#define HT16K33_BLINK_DISPLAYON            0x01 // Display on cmd
#define HT16K33_BLINK_OFF                  0    // Blink off cmd
#define HT16K33_BLINK_CMD                  0x80 // Blink cmd
#define HT16K33_CMD_BRIGHTNESS             0xE0 // Brightness cmd
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

// Wireless global variables
char rx_buffer[128];
char wasd = '0';    // Get wasd from socket or datagram

// A simple structure for queue elements (for timer)
typedef struct {
    uint64_t event_count;
} example_queue_element_t;
// Create a FIFO queue for timer-based events (for timer)
example_queue_element_t ele;
QueueHandle_t timer_queue;

// Global vars for the timer
int timer_count = 0;
uint16_t displayBuffer[4]; // Global structure to store the number (4 digits)
uint16_t lookupTable[10] = {0b00111111, 0b00000110, 0b01011011, 0b01001111, 0b01100110, 0b01101101, 0b01111101, 0b00000111, 0b01111111, 0b01101111};

// Global vars for the LIDAR 
uint8_t status;
uint8_t distanceArray[2];
int distance;

// Global vars for cruise
bool cruisingmode = false;

//Global boolean for measuring speed
bool measuringspeed = false;
int32_t speed = 0;
float steerAngle = -5;

// For speed PID
float currSpeed = 0;
float speed_error = 0;
float wheelspeed = 0;

// For steering PID
float distance_left = 0;
float distance_right = 0;
float steer_error = 0;
int currAngle = -5;
int prevAngle = -5;

// Counting laps
int lap_count = 0;
bool stopwatch_on = false;

static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)
      / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}

static bool example_pcnt_on_reach(pcnt_unit_handle_t unit, const pcnt_watch_event_data_t *edata, void *user_ctx)
{
    BaseType_t high_task_wakeup;
    QueueHandle_t queue = (QueueHandle_t)user_ctx;
    // send event data to queue, from this interrupt callbackr
    xQueueSendFromISR(queue, &(edata->watch_point_value), &high_task_wakeup);
    return (high_task_wakeup == pdTRUE);
}

void speed_task(){
    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
        .clk_src = GPTIMER_CLK_SRC_DEFAULT,
        .direction = GPTIMER_COUNT_UP,
        .resolution_hz = 1000000, 
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    ESP_LOGI(TAG, "install pcnt unit");
    pcnt_unit_config_t unit_config = {
        .high_limit = EXAMPLE_PCNT_HIGH_LIMIT,
        .low_limit = EXAMPLE_PCNT_LOW_LIMIT,
    };
    pcnt_unit_handle_t pcnt_unit = NULL;
    ESP_ERROR_CHECK(pcnt_new_unit(&unit_config, &pcnt_unit));

    ESP_LOGI(TAG, "set glitch filter");
    pcnt_glitch_filter_config_t filter_config = {
        .max_glitch_ns = 1000,
    };
    ESP_ERROR_CHECK(pcnt_unit_set_glitch_filter(pcnt_unit, &filter_config));

    ESP_LOGI(TAG, "install pcnt channels");
    pcnt_chan_config_t chan_a_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_A,
        .level_gpio_num = EXAMPLE_EC11_GPIO_B,
    };
    pcnt_channel_handle_t pcnt_chan_a = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_a_config, &pcnt_chan_a));
    pcnt_chan_config_t chan_b_config = {
        .edge_gpio_num = EXAMPLE_EC11_GPIO_B,
        .level_gpio_num = EXAMPLE_EC11_GPIO_A,
    };
    pcnt_channel_handle_t pcnt_chan_b = NULL;
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &chan_b_config, &pcnt_chan_b));

    ESP_LOGI(TAG, "set edge and level actions for pcnt channels");
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_a, PCNT_CHANNEL_EDGE_ACTION_DECREASE, PCNT_CHANNEL_EDGE_ACTION_INCREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_a, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_chan_b, PCNT_CHANNEL_EDGE_ACTION_INCREASE, PCNT_CHANNEL_EDGE_ACTION_DECREASE));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_chan_b, PCNT_CHANNEL_LEVEL_ACTION_KEEP, PCNT_CHANNEL_LEVEL_ACTION_INVERSE));

    ESP_LOGI(TAG, "add watch points and register callbacks");
    int watch_points[] = {EXAMPLE_PCNT_LOW_LIMIT, -50, 0, 50, EXAMPLE_PCNT_HIGH_LIMIT};
    for (size_t i = 0; i < sizeof(watch_points) / sizeof(watch_points[0]); i++) {
        ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, watch_points[i]));
    }
    pcnt_event_callbacks_t cbs = {
        .on_reach = example_pcnt_on_reach,
    };
    QueueHandle_t queue = xQueueCreate(10, sizeof(int));
    ESP_ERROR_CHECK(pcnt_unit_register_event_callbacks(pcnt_unit, &cbs, queue));

    ESP_LOGI(TAG, "enable pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
    ESP_LOGI(TAG, "clear pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
    ESP_LOGI(TAG, "start pcnt unit");
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));

#if CONFIG_EXAMPLE_WAKE_UP_LIGHT_SLEEP
    // EC11 channel output high level in normal state, so we set "low level" to wake up the chip
    ESP_ERROR_CHECK(gpio_wakeup_enable(EXAMPLE_EC11_GPIO_A, GPIO_INTR_LOW_LEVEL));
    ESP_ERROR_CHECK(esp_sleep_enable_gpio_wakeup());
    ESP_ERROR_CHECK(esp_light_sleep_start());
#endif

    // Report counter value
    int pulse_count = 0;
    int event_count = 0;
    int prevCount = -1; // Initialized to -1 to ensure the first check counts as a change if pulse_count starts at 0
    int change_count = 0; // This will count how many times the pulse_count has changed
    bool newRevolution = true;
    uint64_t count = 0;
    uint32_t out_resolution;
    float duration;
    ESP_ERROR_CHECK(gptimer_enable(gptimer)); // Enables timer


    while (1) {
        // ESP_ERROR_CHECK(gptimer_start(gptimer)); // Timer starts counting
        // Retrieve the timestamp at any time
        ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer, &count));

        if (xQueueReceive(queue, &event_count, pdMS_TO_TICKS(10))) {
            // ESP_LOGI(TAG, "Watch point event, count: %d", event_count);
        } else {
            if (newRevolution)
            {
                ESP_ERROR_CHECK(gptimer_start(gptimer)); // Timer starts counting
                newRevolution = false;
            }
            ESP_ERROR_CHECK(pcnt_unit_get_count(pcnt_unit, &pulse_count));
            if (pulse_count != prevCount) { // Check if pulse_count has changed
                prevCount = pulse_count; // Update prevCount to the new value
                change_count++; // Increment the change counter
                // printf("change count: %d\n", change_count);
                //ESP_LOGI("TAG", "Change count: %d", change_count);
                    if (change_count % 12 == 0){
                        ESP_ERROR_CHECK(gptimer_stop(gptimer)); // Stop the timer 
                        ESP_ERROR_CHECK(gptimer_get_resolution(gptimer, &out_resolution));
                        ESP_ERROR_CHECK(gptimer_get_raw_count(gptimer, &count)); // Get the value of count
                        duration = (float)count / out_resolution;
                        wheelspeed = .62 / duration;
                        // printf("Speed: %f\n",speed);
                        // ESP_LOGI(TAG, "Speed: %f", wheelspeed);
                        ESP_ERROR_CHECK(gptimer_set_raw_count(gptimer, 0));    // Sets count back to zero
                        // Start the timer again
                        count = 0;
                        newRevolution = true;
                    }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

void count_to_display(int num)
{
    // Converts current count to binary for i2c
    int tens_place = (num / 10) % 10;
    int ones_place = num % 10;
    int hundreds_place = (num / 100) % 10;
    int thousands_place = (num / 1000) % 10;
    displayBuffer[0] = lookupTable[thousands_place];
    displayBuffer[1] = lookupTable[hundreds_place];
    displayBuffer[2] = lookupTable[tens_place];
    displayBuffer[3] = lookupTable[ones_place];
}

void display()
{
    // Displays the displaybuffer
    i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
    i2c_master_start(cmd4);
    i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
    for (int i = 0; i < 4; i++) 
    {
        i2c_master_write_byte(cmd4, displayBuffer[i] & 0xFF, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, displayBuffer[i] >> 8, ACK_CHECK_EN);
    }
    i2c_master_stop(cmd4);
    int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd4);
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

    gptimer_alarm_config_t alarm_config = { // Configure the alarm 
      .reload_count = 0,                    // counter will reload with 0 on alarm event
      .alarm_count = 1*1000000,            // period = 1*1s = 1s
      .flags.auto_reload_on_alarm = true,   // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));  // this enacts the alarm config
    ESP_ERROR_CHECK(gptimer_start(gptimer));                            // this starts the timer
}

static void i2c_example_master_init(){
    // Debug
    printf("\n>> i2c Config\n");
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
    if (err == ESP_OK) {printf("- parameters: ok\n");}

    // Install I2C driver
    err = i2c_driver_install(i2c_master_port, conf.mode,
                       I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                       I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
    // i2c_set_data_mode(i2c_master_port,I2C_DATA_MODE_LSB_FIRST,I2C_DATA_MODE_LSB_FIRST);
    if (err == ESP_OK) {printf("- initialized: yes\n\n");}

    // Dat in MSB mode
    i2c_set_data_mode(i2c_master_port, I2C_DATA_MODE_MSB_FIRST, I2C_DATA_MODE_MSB_FIRST);
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
    printf("\n>> I2C scanning ..."  "\n");
    uint8_t count = 0;
    for (uint8_t i = 1; i < 127; i++) {
        // printf("0x%X%s",i,"\n");
        if (testConnection(i, scanTimeout) == ESP_OK) {
            printf( "- Device found at address: 0x%X%s", i, "\n");
            count++;
        }
    }
    if (count == 0)
        printf("- No I2C devices found!" "\n");
    printf("\n");
}

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

void init_i2c()
{
    int ret;
    printf(">> Test Alphanumeric Display: \n");

    // Set up routines
    // Turn on alpha oscillator
    ret = alpha_oscillator();
    if(ret == ESP_OK) {printf("- oscillator: ok \n");}
    // Set display blink off
    ret = no_blink();
    if(ret == ESP_OK) {printf("- blink: off \n");}
    ret = set_brightness_max(0xF);
    if(ret == ESP_OK) {printf("- brightness: max \n");}
}

void turn_off_all()
{
    for (int i = 0; i < 4; i++)
      {
        displayBuffer[i] = 0b0000000000000000;
      }
      i2c_cmd_handle_t cmd4 = i2c_cmd_link_create();
        i2c_master_start(cmd4);
        i2c_master_write_byte(cmd4, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
        i2c_master_write_byte(cmd4, (uint8_t)0x00, ACK_CHECK_EN);
      for (uint8_t i=0; i< (uint8_t)4; i++) {
            i2c_master_write_byte(cmd4, displayBuffer[i] & 0xFF, ACK_CHECK_EN);
            i2c_master_write_byte(cmd4, displayBuffer[i] >> 8, ACK_CHECK_EN);
        }
        i2c_master_stop(cmd4);
        int ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd4, 1000 / portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd4);
}

// Timer task -- what to do when the timer alarm triggers 
static void timer_evt_task(void *arg) {
    // Timer queue initialize 
    timer_queue = xQueueCreate(10, sizeof(example_queue_element_t));

    i2c_example_master_init();
    i2c_scanner();
    init_i2c();
    alarm_init();
    turn_off_all();

    count_to_display(timer_count);
    display();

  while (1) {
    // Transfer from queue and do something if triggered
    if (xQueueReceive(timer_queue, &ele, pdMS_TO_TICKS(2000))) {
      // Cycle count
      if(stopwatch_on == true){
            timer_count++;
      }
        count_to_display(timer_count);
        display();
    }
  }
}

void driving_task()
{   
    ESP_LOGI(TAG, "Create timer and operator");
    mcpwm_timer_handle_t timer = NULL;             // Create PWM timer
    mcpwm_timer_config_t timer_config = {          // Configure PWM timer 
      .group_id = 0,                               // Pick PWM group 0
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,      // Default clock source 
      .resolution_hz = SERVO_TIMEBASE_RESOLUTION_HZ, // Hz
      .period_ticks = SERVO_TIMEBASE_PERIOD,       // Set servo period (20ms -- 50 Hz)
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,     // Count up
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;               // Create PWM operator 
    mcpwm_operator_config_t operator_config = {    // Configure PWM operator
      .group_id = 0,                               // operator same group and PWM timer
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

    ESP_LOGI(TAG, "Connect timer and operator");   // Connect PWM timer and PWM operator
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    //Creating PWM for esc
    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t esc = NULL;         // Create PWM comparator
    mcpwm_comparator_config_t esc_config = {// Updates when timer = zero
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &esc_config, &esc));

    mcpwm_gen_handle_t esc_generator = NULL;            // Create generator
    mcpwm_generator_config_t esc_generator_config = {   // Output to GPIO pin 
        .gen_gpio_num = ESC_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &esc_generator_config, &esc_generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(esc_generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(esc_generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, esc, MCPWM_GEN_ACTION_LOW)));

    //Creating PWM for steering
    ESP_LOGI(TAG, "Create comparator and generator from the operator");
    mcpwm_cmpr_handle_t steer = NULL;         // Create PWM comparator
    mcpwm_comparator_config_t steer_config = {// Updates when timer = zero
        .flags.update_cmp_on_tez = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &steer_config, &steer));

    mcpwm_gen_handle_t steer_generator = NULL;            // Create generator
    mcpwm_generator_config_t steer_generator_config = {   // Output to GPIO pin 
        .gen_gpio_num = STEER_GPIO,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &steer_generator_config, &steer_generator));

    // set the initial compare value, so that the servo will spin to the center position
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(0)));

    ESP_LOGI(TAG, "Set generator action on timer and compare event");
    // go high on counter empty
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(steer_generator,
        MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    // go low on compare threshold
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(steer_generator,
        MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, steer, MCPWM_GEN_ACTION_LOW)));
    ESP_LOGI(TAG, "Enable and start timer");
    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));       // Enable
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP)); // Run continuously

    // This code drives the servo over a range
    // The PWM is changed slowly allow the (mechanical) servo to keep up 
    //int speed = 0;

    //calibrate esc_______________________________________________________________________________//
    //("Calibrating\n");
    ESP_LOGI(TAG, "Calibrating.");
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(-5)));
    vTaskDelay(pdMS_TO_TICKS(3600));
    //printf("Leaving at center position\n");
    ESP_LOGI(TAG, "Leaving at center position.");
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(-5)));
    //____________________________________________________________________________________________//

    //___________________________________
    // Key:                             |
    //                                  |
    // W --> move forward               |
    // A --> steer left                 |
    // S --> move backward              |
    // D --> steer right                |
    // Z --> reset steer to 0 degrees   |
    // X --> emergency break            |
    //__________________________________|

    while (1) {
        // This code changes the PWM slowly to allow the (mechanical) servo to keep up 
        // printf("Enter speed: \n");
        
        if (wasd == 'w' && speed < SERVO_MAX_DEGREE)
        {
            speed += 2;
            wasd = 'p';
            measuringspeed = true;
            if (speed == 0) // Checking when at midpoint after already moving in the other direction
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(speed)));
                //vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(0)));
                //vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(speed)));
            } else {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(speed)));
            }
            // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(speed)));
            // ESP_LOGI(TAG, "Speed set to: %ld", speed); //print speed
        }
        
        else if (wasd == 's' && speed > SERVO_MIN_DEGREE)
        {
            speed -= 2;
            wasd = 'p';
            measuringspeed = true;
        
            if (speed == 0)
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(speed)));
                //vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(0)));
                //vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(speed)));
            } else {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(speed)));
            }
            // ESP_LOGI(TAG, "Speed set to: %ld", speed); //print speed
        }
     
        else if (wasd == 'd') {
            steerAngle -= 15;
            if (steerAngle == 0)
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
                //vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer,example_angle_to_compare(0)));
                //vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
            } else {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
            }
            wasd = 'p';
        }
        else if (wasd == 'a') {
            steerAngle += 15;
            if (steerAngle == 0)
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
                //vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(0)));
                //vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
            } else {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
            }
            wasd = 'p';
        }
        else if (wasd == 'r'){
            cruisingmode = true;       
            measuringspeed = true; 
            if (stopwatch_on == false){
                stopwatch_on = true;
                timer_count = 0;
            }
            if (cruisingmode == true && distance > 110)
            {   
                if (currSpeed == 0) // Checking when at midpoint after already moving in the other direction
                {
                    currSpeed = 10;
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(currSpeed)));
                }

                ///////////////////////////
                // Speed adjustments
                float speed_setpoint = 3.5; // Setpoint is 4 meters/s
                speed_error = speed_setpoint - wheelspeed; // Checks to see how close we are to target speed
                if (speed_error > 0)
                {
                    // Need to decrease speed
                    currSpeed -= 0.05;
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(currSpeed)));
                }
                else if (speed_error < 0){
                    currSpeed += 0.05;
                    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(currSpeed)));
                }

                // Steering adjustments 
                if (distance_right < distance_left){
                    //We are going towards the end of the classroom
                    float distance_setpoint = 20;
                    float distance_right_error = distance_setpoint - distance_right;

                    if (distance_right_error > 0){
                        steerAngle += 1;
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
                    }
                    else if (distance_right_error < 0)
                    {
                        steerAngle -= 1;
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
                    }
                }
                else if (distance_right > distance_left){
                    float distance_setpoint = 20;
                    float distance_left_error = distance_setpoint - distance_left;
                    if (distance_left_error > 0) {
                        steerAngle -= 1;
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
                    } 
                    else if (distance_left_error < 0){
                        steerAngle += 1;
                        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
                    }
                }
                ESP_LOGI(TAG, "Steer angle is %f", steerAngle);
                ESP_LOGI(TAG, "Distance right is %f", distance_right);
                ESP_LOGI(TAG, "Distance left is %f", distance_left);
            }
            else { 
                ESP_LOGI(TAG, "Wall detected.");
                wasd = 'p';
                currSpeed = -55;
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(currSpeed)));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(0)));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(currSpeed)));
                //vTaskDelay(pdMS_TO_TICKS(10));
                cruisingmode = false;
                currSpeed = 0;
                lap_count += 1;
                if (lap_count % 2 == 0){
                    stopwatch_on = false;
                }
            }
        }
        
        else if (wasd == 'z')
        {
            wasd = 'p';
            // Set wheels back to zero
            steerAngle = -5;
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
            //vTaskDelay(pdMS_TO_TICKS(10));
        }

        if ((speed > 0 && distance < 45)  || wasd == 'x')
        {
            // Emergency stopping and moving forwards
            ESP_LOGI(TAG, "Emergency stopping");
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(-80)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(0)));
            ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(-80)));
            wasd = 'p';
        }

        // ESP_LOGI(TAG, "Speed: %ld ", speed);
        // Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
        vTaskDelay(pdMS_TO_TICKS(200));
    }
}

static bool hc_sr04_echo_callback(mcpwm_cap_channel_handle_t cap_chan_left, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    static uint32_t cap_val_begin_of_sample = 0;
    static uint32_t cap_val_end_of_sample = 0;
    TaskHandle_t task_to_notify = (TaskHandle_t)user_data;
    BaseType_t high_task_wakeup = pdFALSE;

    //calculate the interval in the ISR,
    //so that the interval will be always correct even when capture_queue is not handled in time and overflow.
    if (edata->cap_edge == MCPWM_CAP_EDGE_POS) {
        // store the timestamp when pos edge is detected
        cap_val_begin_of_sample = edata->cap_value;
        cap_val_end_of_sample = cap_val_begin_of_sample;
    } else {
        cap_val_end_of_sample = edata->cap_value;
        uint32_t tof_ticks = cap_val_end_of_sample - cap_val_begin_of_sample;

        // notify the task to calculate the distance
        xTaskNotifyFromISR(task_to_notify, tof_ticks, eSetValueWithOverwrite, &high_task_wakeup);
    }
    return high_task_wakeup == pdTRUE;
}

static void gen_trig_left(void)
{
    gpio_set_level(HC_SR04_LEFT_TRIG, 1); // set high
    esp_rom_delay_us(10);
    gpio_set_level(HC_SR04_LEFT_TRIG, 0); // set low
}

static void gen_trig_right(void)
{
    gpio_set_level(HC_SR04_RIGHT_TRIG, 1); // set high
    esp_rom_delay_us(10);
    gpio_set_level(HC_SR04_RIGHT_TRIG, 0); // set low
}

// For LIDAR
void i2c_master_init() {
    // Configuration for the I2C master interface
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 23,
        .scl_io_num = 22,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 100000  // Fast mode
    };
    i2c_param_config(LIDAR_I2C_NUM, &conf);
    i2c_driver_install(LIDAR_I2C_NUM, conf.mode, 0, 0, 0);
}

// Function to write byte to a register
void lidar_write_byte(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIDAR_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(LIDAR_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}

// Function to read byte(s) from a register
void lidar_read_bytes(uint8_t reg, uint8_t *data, size_t length) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIDAR_ADDRESS << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (LIDAR_ADDRESS << 1) | I2C_MASTER_READ, true);
    if (length > 1) {
        i2c_master_read(cmd, data, length - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + length - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(LIDAR_I2C_NUM, cmd, 1000 / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
}
    
void sensor_task()
{
    ESP_LOGI(TAG, "Install capture timer for Left sensor");
    mcpwm_cap_timer_handle_t cap_timer_left = NULL;
    mcpwm_capture_timer_config_t cap_conf_left = {
    .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
    .group_id = 0,  // Use group 0 for Left sensor
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf_left, &cap_timer_left));

    ESP_LOGI(TAG, "Install capture channel for Left sensor");
    mcpwm_cap_channel_handle_t cap_chan_left = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf_left = {
    .gpio_num = HC_SR04_LEFT_ECHO,
    .prescale = 1,
    .flags.neg_edge = true,  // Capture on the falling edge
    .flags.pos_edge = true,  // Capture on the rising edge
    .flags.pull_up = true,   // Internal pull-up enabled
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer_left, &cap_ch_conf_left, &cap_chan_left));

    ESP_LOGI(TAG, "Install capture timer for Right sensor");
    mcpwm_cap_timer_handle_t cap_timer_right = NULL;
    mcpwm_capture_timer_config_t cap_conf_right = {
    .clk_src = MCPWM_CAPTURE_CLK_SRC_DEFAULT,
    .group_id = 1,  // Use group 1 for Right sensor
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_conf_right, &cap_timer_right));

    ESP_LOGI(TAG, "Install capture channel for Right sensor");
    mcpwm_cap_channel_handle_t cap_chan_right = NULL;
    mcpwm_capture_channel_config_t cap_ch_conf_right = {
    .gpio_num = HC_SR04_RIGHT_ECHO,
    .prescale = 1,
    .flags.neg_edge = true,  // Capture on the falling edge
    .flags.pos_edge = true,  // Capture on the rising edge
    .flags.pull_up = true,   // Internal pull-up enabled
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer_right, &cap_ch_conf_right, &cap_chan_right));

    ESP_LOGI(TAG, "Register capture callback for Left sensor");
    TaskHandle_t left_task = xTaskGetCurrentTaskHandle();
    mcpwm_capture_event_callbacks_t left_cbs = {
    .on_cap = hc_sr04_echo_callback,  // Use your callback function
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan_left, &left_cbs, left_task));

    ESP_LOGI(TAG, "Register capture callback for Right sensor");
    TaskHandle_t right_task = xTaskGetCurrentTaskHandle();
    mcpwm_capture_event_callbacks_t right_cbs = {
    .on_cap = hc_sr04_echo_callback,  // Use your callback function
    };
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan_right, &right_cbs, right_task));

    ESP_LOGI(TAG, "Enable capture channels");
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan_left));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan_right));

    ESP_LOGI(TAG, "Configure Trig pins");
    gpio_config_t io_conf_left = {
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = 1ULL << HC_SR04_LEFT_TRIG,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_left));

    gpio_config_t io_conf_right = {
    .mode = GPIO_MODE_OUTPUT,
    .pin_bit_mask = 1ULL << HC_SR04_RIGHT_TRIG,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf_right));

    // Drive Trig pins low by default
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_LEFT_TRIG, 0));
    ESP_ERROR_CHECK(gpio_set_level(HC_SR04_RIGHT_TRIG, 0));

    ESP_LOGI(TAG, "Enable and start capture timers");
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer_left));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer_left));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer_right));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer_right));

    while (1) {
    // Trigger the sensors to start a new sample
        gen_trig_left();
        uint32_t tof_ticks_left;
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks_left, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us = tof_ticks_left * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us <= 35000) {
                distance_left = pulse_width_us / 58;
                // ESP_LOGI(TAG, "Left sensor distance: %.2f cm", distance_left);
            }
        }
        gen_trig_right();
        uint32_t tof_ticks_right;
        if (xTaskNotifyWait(0x00, ULONG_MAX, &tof_ticks_right, pdMS_TO_TICKS(1000)) == pdTRUE) {
            float pulse_width_us = tof_ticks_right * (1000000.0 / esp_clk_apb_freq());
            if (pulse_width_us <= 35000) {
                distance_right = pulse_width_us / 58;
                // ESP_LOGI(TAG, "Right sensor distance: %.2f cm", distance_right);
            }
        }
        lidar_write_byte(REG_ACQ_COMMAND, ACQ_COMMAND);
        lidar_write_byte(0x05, 0x80);
        lidar_write_byte(0xE5, 0x08);
        vTaskDelay(22 / portTICK_PERIOD_MS);
        do {
            lidar_read_bytes(REG_STATUS, &status, 1);
        } while (status & 0x01);
        lidar_write_byte(0x05, 0x80);
        lidar_write_byte(0xE5, 0x08);
        lidar_read_bytes(REG_FULL_DIST_LOW, distanceArray, 2);
        distance = (distanceArray[1] << 8) | distanceArray[0];
        // ESP_LOGI(TAG, "Distance: %d cm", distance);
        vTaskDelay(pdMS_TO_TICKS(1));
    }   
}

static void udp_server_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    struct sockaddr_in6 dest_addr;
    while (1) {
        if (addr_family == AF_INET) {
            struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
            dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
            dest_addr_ip4->sin_family = AF_INET;
            dest_addr_ip4->sin_port = htons(PORT);
            ip_protocol = IPPROTO_IP;
        } else if (addr_family == AF_INET6) {
            bzero(&dest_addr.sin6_addr.un, sizeof(dest_addr.sin6_addr.un));
            dest_addr.sin6_family = AF_INET6;
            dest_addr.sin6_port = htons(PORT);
            ip_protocol = IPPROTO_IPV6;
        }

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket created");

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        int enable = 1;
        lwip_setsockopt(sock, IPPROTO_IP, IP_PKTINFO, &enable, sizeof(enable));
#endif

#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
        if (addr_family == AF_INET6) {
            // Note that by default IPV6 binds to both protocols, it is must be disabled
            // if both protocols used at the same time (used in CI)
            int opt = 1;
            setsockopt(sock, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
            setsockopt(sock, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
        }
#endif
        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        int err = bind(sock, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
        if (err < 0) {
            ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        }
        ESP_LOGI(TAG, "Socket bound, port %d", PORT);

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t socklen = sizeof(source_addr);

#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
        struct iovec iov;
        struct msghdr msg;
        struct cmsghdr *cmsgtmp;
        u8_t cmsg_buf[CMSG_SPACE(sizeof(struct in_pktinfo))];

        iov.iov_base = rx_buffer;
        iov.iov_len = sizeof(rx_buffer);
        msg.msg_control = cmsg_buf;
        msg.msg_controllen = sizeof(cmsg_buf);
        msg.msg_flags = 0;
        msg.msg_iov = &iov;
        msg.msg_iovlen = 1;
        msg.msg_name = (struct sockaddr *)&source_addr;
        msg.msg_namelen = socklen;
#endif

        while (1) {
            ESP_LOGI(TAG, "Waiting for data");
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
            int len = recvmsg(sock, &msg, 0);
#else
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
#endif
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
            }
            // Data received
            else {
                // Get the sender's ip address as string
                if (source_addr.ss_family == PF_INET) {
                    inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
#if defined(CONFIG_LWIP_NETBUF_RECVINFO) && !defined(CONFIG_EXAMPLE_IPV6)
                    for ( cmsgtmp = CMSG_FIRSTHDR(&msg); cmsgtmp != NULL; cmsgtmp = CMSG_NXTHDR(&msg, cmsgtmp) ) {
                        if ( cmsgtmp->cmsg_level == IPPROTO_IP && cmsgtmp->cmsg_type == IP_PKTINFO ) {
                            struct in_pktinfo *pktinfo;
                            pktinfo = (struct in_pktinfo*)CMSG_DATA(cmsgtmp);
                            ESP_LOGI(TAG, "dest ip: %s\n", inet_ntoa(pktinfo->ipi_addr));
                        }
                    }
#endif
                } else if (source_addr.ss_family == PF_INET6) {
                    inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
                }
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string...
                ESP_LOGI(TAG, "Received %d bytes from %s:", len, addr_str);
                ESP_LOGI(TAG, "%s", rx_buffer);
                wasd = rx_buffer[0];
                ESP_LOGI(TAG, "WASD value is %c", wasd);
                int err = sendto(sock, rx_buffer, len, 0, (struct sockaddr *)&source_addr, sizeof(source_addr));
                if (err < 0) {
                    ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                    break;
                }
            }
        }

        if (sock != -1) {
            ESP_LOGE(TAG, "Shutting down socket and restarting...");
            shutdown(sock, 0);
            close(sock);
        }
    }
    vTaskDelete(NULL);
}

void app_main(void)
{
    i2c_master_init();
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());

    xTaskCreate(&udp_server_task, "udp_server", 4096, (void*)AF_INET, configMAX_PRIORITIES, NULL);

    xTaskCreate(&driving_task, "Driving_Task", 4096, NULL, configMAX_PRIORITIES, NULL);
    xTaskCreate(&speed_task, "Speed_Task", 4096, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(&sensor_task, "Sensor_Task", 4096, NULL, configMAX_PRIORITIES, NULL); //added PID to the sensor task 
    // Create task to handle timer-based events 
    xTaskCreate(&timer_evt_task, "Timer_evt_task", 4096, NULL, configMAX_PRIORITIES-1, NULL);
}

 
