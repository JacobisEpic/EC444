// Code contributers: Celine Chen, Eric Chen, Jacob Chin, Nuo Lin
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "protocol_examples_common.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "driver/gpio.h"
#include <stdio.h>
#include <stdlib.h>             // Added in 2023..
#include <inttypes.h>           // Added in 2023
#include "freertos/queue.h"     // Added in 2023
#include "esp_log.h"
#include "esp_system.h"         // Added in 2023
#include "driver/rmt_tx.h"      // Modified in 2023
#include "soc/rmt_reg.h"        // Not needed?
#include "driver/uart.h"
#include "driver/gptimer.h"     // Added in 2023
#include "driver/mcpwm_prelude.h"// Added in 2023
#include "driver/ledc.h"        // Added in 2023
#include "driver/i2c.h"
#include "ssd1306.h"
#include "font8x8_basic.h"
#include "sdkconfig.h"
#include "esp_task_wdt.h"

//includes for the qr code
#define SDA_PIN 23 //CHANGED TO OUR ESP PINOUT 
#define SCL_PIN 22 //CHANGED TO OUR ESP PINOUT
#define tag "SSD1306"
////////////////////

// MCPWM defintions -- 2023: modified
#define MCPWM_TIMER_RESOLUTION_HZ 10000000 // 10MHz, 1 tick = 0.1us
#define MCPWM_FREQ_HZ             38000    // 38KHz PWM -- 1/38kHz = 26.3us
#define MCPWM_FREQ_PERIOD         263      // 263 ticks = 263 * 0.1us = 26.3us
#define MCPWM_GPIO_NUM            25

// LEDC definitions -- 2023: modified 
// NOT USED / altnernative to MCPWM above
#define LEDC_TIMER              LEDC_TIMER_0
#define LEDC_MODE               LEDC_LOW_SPEED_MODE
#define LEDC_OUTPUT_IO          25
#define LEDC_CHANNEL            LEDC_CHANNEL_0
#define LEDC_DUTY_RES           6     // Set duty resolution to 6 bits
#define LEDC_DUTY               32    // Set duty to 50%. ((2^6) - 1) * 50% = 32
#define LEDC_FREQUENCY          38000 // Frequency in Hertz. 38kHz

// UART definitions -- 2023: no changes
#define UART_TX_GPIO_NUM 26 // A0
#define UART_RX_GPIO_NUM 34 // A2
#define BUF_SIZE (1024)
#define BUF_SIZE2 (32)

// Hardware interrupt definitions -- 2023: no changes
#define GPIO_INPUT_IO_1       4
#define ESP_INTR_FLAG_DEFAULT 0
#define GPIO_INPUT_PIN_SEL    1ULL<<GPIO_INPUT_IO_1

#define REDLED 15
#define GREENLED 32
#define YELLOWLED 14

// Variables for my ID, minVal and status plus string fragments
char start = 0x1B;              // START BYTE that UART looks for
int len_out = 3;
char MeterID = '1';

// Mutex (for resources), and Queues (for button)
SemaphoreHandle_t mux = NULL; // 2023: no changes
static QueueHandle_t gpio_evt_queue = NULL; // 2023: Changed
// static xQueueHandle_t timer_queue; -- 2023: removed

// A simple structure to pass "events" to main task -- 2023: modified
typedef struct {
    uint64_t event_count;
} example_queue_element_t;

// Create a FIFO queue for timer-based events -- Modified
example_queue_element_t ele;
QueueHandle_t timer_queue;

// System tags for diagnostics -- 2023: modified
//static const char *TAG_SYSTEM = "ec444: system";       // For debug logs
static const char *TAG_TIMER = "ec444: timer";         // For timer logs
static const char *TAG_UART = "ec444: uart";           // For UART logs 

// Button interrupt handler -- add to queue -- 2023: no changes
static void IRAM_ATTR gpio_isr_handler(void* arg){
  uint32_t gpio_num = (uint32_t) arg;
  xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

// Timmer interrupt handler -- Callback timer function -- 2023: modified
// Note we enabled time for auto-reload
static bool IRAM_ATTR timer_on_alarm_cb(gptimer_handle_t timer, const gptimer_alarm_event_data_t *edata, void *user_data) {
    BaseType_t high_task_awoken = pdFALSE;
    QueueHandle_t timer_queue1 = (QueueHandle_t)user_data;
    // Retrieve count value and send to queue
    example_queue_element_t ele = {
        .event_count = edata->count_value
    };
    xQueueSendFromISR(timer_queue1, &ele, &high_task_awoken);
    return (high_task_awoken == pdTRUE);
}


#ifdef CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN
#include "addr_from_stdin.h"
#endif

#if defined(CONFIG_EXAMPLE_IPV4)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV4_ADDR
#elif defined(CONFIG_EXAMPLE_IPV6)
#define HOST_IP_ADDR CONFIG_EXAMPLE_IPV6_ADDR
#else
#define HOST_IP_ADDR "192.168.1.10"
#endif

#define PORT 3337

static const char *TAG = "example";
static const char *payload = "Meter";

// Create a FIFO queue for timer-based events -- Modified
example_queue_element_t ele;
QueueHandle_t timer_queue;

char genCheckSum(char *p, int len) {
  char temp = 0;
  for (int i = 0; i < len; i++){
    temp = temp^p[i];
  }
  // printf("%X\n",temp);  // Diagnostic

  return temp;
}

bool checkCheckSum(uint8_t *p, int len) {
  char temp = (char) 0;
  bool isValid;
  for (int i = 0; i < len-1; i++){
    temp = temp^p[i];
  }
  // printf("Check: %02X ", temp); // Diagnostic
  if (temp == p[len-1]) {
    isValid = true; }
  else {
    isValid = false; }
  return isValid;
}

static void pwm_init() {

  // Create timer
  mcpwm_timer_handle_t pwm_timer = NULL;
  mcpwm_timer_config_t pwm_timer_config = {
      .group_id = 0,
      .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
      .resolution_hz = MCPWM_TIMER_RESOLUTION_HZ,
      .period_ticks = MCPWM_FREQ_PERIOD,
      .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
  };
  ESP_ERROR_CHECK(mcpwm_new_timer(&pwm_timer_config, &pwm_timer));

  // Create operator
  mcpwm_oper_handle_t oper = NULL;
  mcpwm_operator_config_t operator_config = {
      .group_id = 0, // operator must be in the same group to the timer
  };
  ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));

  // Connect timer and operator
  ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, pwm_timer));

  // Create comparator from the operator
  mcpwm_cmpr_handle_t comparator = NULL;
  mcpwm_comparator_config_t comparator_config = {
      .flags.update_cmp_on_tez = true,
  };
  ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

  // Create generator from the operator
  mcpwm_gen_handle_t generator = NULL;
  mcpwm_generator_config_t generator_config = {
      .gen_gpio_num = MCPWM_GPIO_NUM,
  };
  ESP_ERROR_CHECK(mcpwm_new_generator(oper, &generator_config, &generator));

  // set the initial compare value, so that the duty cycle is 50%
  ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator,132));
  // CANNOT FIGURE OUT HOW MANY TICKS TO COMPARE TO TO GET 50%

  // Set generator action on timer and compare event
  // go high on counter empty
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(generator,
                  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
  // go low on compare threshold
  ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(generator,
                  MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));

  // Enable and start timer
  ESP_ERROR_CHECK(mcpwm_timer_enable(pwm_timer));
  ESP_ERROR_CHECK(mcpwm_timer_start_stop(pwm_timer, MCPWM_TIMER_START_NO_STOP));

}

static void uart_init() {
  // Basic configs
  const uart_config_t uart_config = {
      .baud_rate = 1200, // Slow BAUD rate
      .data_bits = UART_DATA_8_BITS,
      .parity    = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
      .source_clk = UART_SCLK_DEFAULT
  };
  uart_param_config(UART_NUM_1, &uart_config);

  // Set UART pins using UART0 default pins
  uart_set_pin(UART_NUM_1, UART_TX_GPIO_NUM, UART_RX_GPIO_NUM, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

  // Reverse receive logic line
  uart_set_line_inverse(UART_NUM_1,UART_SIGNAL_RXD_INV);

  // Install UART driver
  uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

static void alarm_init() {

    gptimer_handle_t gptimer = NULL;
    gptimer_config_t timer_config = {
      .clk_src = GPTIMER_CLK_SRC_DEFAULT,
      .direction = GPTIMER_COUNT_UP,
      .resolution_hz = 1000000, // 1MHz, 1 tick=1us
    };
    ESP_ERROR_CHECK(gptimer_new_timer(&timer_config, &gptimer));

    // Set alarm callback
    gptimer_event_callbacks_t cbs = {
        .on_alarm = timer_on_alarm_cb,
    };
    ESP_ERROR_CHECK(gptimer_register_event_callbacks(gptimer, &cbs, timer_queue));

    // Enable timer
    ESP_ERROR_CHECK(gptimer_enable(gptimer));

    ESP_LOGI(TAG_TIMER, "Start timer, update alarm value dynamically and auto reload");
    gptimer_alarm_config_t alarm_config = {
      .reload_count = 0, // counter will reload with 0 on alarm event
      .alarm_count = 10*1000000, // period = 10*1s = 10s
      .flags.auto_reload_on_alarm = true, // enable auto-reload
    };
    ESP_ERROR_CHECK(gptimer_set_alarm_action(gptimer, &alarm_config));
    ESP_ERROR_CHECK(gptimer_start(gptimer));

}

///////////////////////////////////////// needed for the OLED display
void i2c_master_init()
{
	i2c_config_t i2c_config = {
		.mode = I2C_MODE_MASTER,
		.sda_io_num = SDA_PIN,
		.scl_io_num = SCL_PIN,
		.sda_pullup_en = GPIO_PULLUP_ENABLE,
		.scl_pullup_en = GPIO_PULLUP_ENABLE,
		.master.clk_speed = 1000000
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER, 0, 0, 0);
}

void ssd1306_init() {
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
	i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_STREAM, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_CHARGE_PUMP, true);
	i2c_master_write_byte(cmd, 0x14, true);

	i2c_master_write_byte(cmd, OLED_CMD_SET_SEGMENT_REMAP, true); // reverse left-right mapping
	i2c_master_write_byte(cmd, OLED_CMD_SET_COM_SCAN_MODE, true); // reverse up-bottom mapping

	i2c_master_write_byte(cmd, OLED_CMD_DISPLAY_ON, true);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		ESP_LOGI(tag, "OLED configured successfully");
	} else {
		ESP_LOGE(tag, "OLED configuration failed. code: 0x%.2X", espRc);
	}
	i2c_cmd_link_delete(cmd);
}

void task_ssd1306_display_clear() {
	i2c_cmd_handle_t cmd;

	uint8_t zero[128];
	memset(zero, 0, sizeof(zero)); 

	for (uint8_t i = 0; i < 8; i++) {
		cmd = i2c_cmd_link_create();
		i2c_master_start(cmd);
		i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
		i2c_master_write_byte(cmd, 0xB0 | i, true);

		i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
		i2c_master_write(cmd, zero, 128, true);
		i2c_master_stop(cmd);
		i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
		i2c_cmd_link_delete(cmd);
	}
}

extern const uint8_t bitmap[1024] = {
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0x1f, 0x0f, 0x0f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f,
0x8f, 0x8f, 0x0f, 0x0f, 0xff, 0xff, 0x0f, 0x0f, 0x8f, 0x8f, 0x0f, 0x0f, 0x0f, 0x7f, 0x7f, 0x8f,
0x8f, 0xff, 0xff, 0x7f, 0x7f, 0x3f, 0x0f, 0x0f, 0x8f, 0x8f, 0xff, 0xff, 0x0f, 0x0f, 0x8f, 0x8f,
0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x8f, 0x0f, 0x0f, 0x1f, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xff,
0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0xe6, 0xe6, 0xe7, 0xe7, 0xe0, 0xe0, 0xe0, 0x18, 0x18, 0x7f,
0x7f, 0x99, 0x99, 0x78, 0x78, 0xf8, 0xf8, 0xf8, 0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0xff, 0xff,
0xff, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0x30, 0x30, 0x30, 0xf3, 0xf3, 0x33, 0x33, 0x33, 0x33, 0xf3, 0xf3, 0x33,
0x33, 0x31, 0x30, 0x30, 0x3f, 0x3f, 0xc3, 0xc3, 0xff, 0xff, 0x03, 0x03, 0x03, 0x0c, 0x0c, 0xc0,
0xc0, 0x0f, 0x0f, 0x00, 0x00, 0x00, 0x0f, 0x0f, 0x03, 0x03, 0xff, 0xff, 0x30, 0x30, 0xf1, 0xf3,
0xf3, 0xf3, 0xf3, 0x33, 0x33, 0xf3, 0xf3, 0x33, 0x33, 0x30, 0x30, 0x30, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0x78, 0x78, 0xe0, 0xe0, 0x00, 0x00, 0x63, 0x63, 0x63,
0x63, 0x63, 0x63, 0x63, 0xe0, 0xe0, 0x18, 0x18, 0x7f, 0x7f, 0x00, 0x00, 0x00, 0xe0, 0xe0, 0x98,
0x98, 0x9f, 0x9f, 0x87, 0x83, 0x87, 0x9f, 0x9f, 0xe3, 0xe3, 0x1f, 0x1f, 0x7f, 0x7f, 0x7c, 0x78,
0x78, 0x63, 0x63, 0x9f, 0x9f, 0x1f, 0x1f, 0x78, 0x78, 0x7f, 0x7f, 0x7f, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0x3e, 0x3e, 0x3e, 0xc6, 0xc6, 0x39, 0x39, 0x06, 0x06, 0x38, 0x38, 0xc0,
0xc0, 0xc0, 0xc6, 0xc6, 0x07, 0x07, 0xfe, 0xfe, 0xfe, 0xfe, 0xee, 0xc6, 0xe6, 0xf9, 0xf9, 0x07,
0x07, 0x39, 0x39, 0xc7, 0xc7, 0xc7, 0x39, 0x39, 0xf9, 0xf9, 0xc6, 0xc6, 0x38, 0x38, 0x38, 0x38,
0x18, 0x00, 0x00, 0xff, 0xff, 0xc6, 0xc6, 0xc6, 0xc6, 0x38, 0x38, 0x38, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0x0f, 0x0f, 0x0f, 0xcf, 0xcf, 0xcc, 0xcc, 0xcf, 0xcf, 0xcc, 0xcc, 0xcc,
0xcc, 0x8c, 0x0c, 0x0c, 0xfc, 0xfc, 0x03, 0x03, 0x33, 0x33, 0x03, 0x03, 0x03, 0xcc, 0xcc, 0x03,
0x03, 0xf3, 0xf3, 0xff, 0xff, 0xff, 0x3c, 0x3c, 0x00, 0x00, 0xfc, 0xfc, 0xcc, 0xcc, 0xfc, 0xfc,
0xfc, 0x00, 0x00, 0xfc, 0xfc, 0xc0, 0xc0, 0x0c, 0x0c, 0xc0, 0xc0, 0xc0, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0x00, 0x00, 0x00, 0xff, 0xff, 0x81, 0x81, 0x81, 0x81, 0x81, 0x81, 0xff,
0xff, 0xff, 0x00, 0x00, 0xff, 0xff, 0x86, 0x86, 0x86, 0x86, 0x86, 0x9e, 0x9e, 0x99, 0x99, 0x80,
0x80, 0x81, 0x81, 0xe7, 0xe7, 0xe7, 0x86, 0x86, 0x80, 0x80, 0xe1, 0xe1, 0x81, 0x81, 0xf9, 0xf9,
0xf8, 0x00, 0x00, 0xe1, 0xe1, 0x07, 0x07, 0x86, 0x86, 0xe7, 0xe7, 0xe7, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xf8, 0xf0, 0xf0, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1,
0xf1, 0xf1, 0xf0, 0xf0, 0xff, 0xff, 0xf1, 0xf1, 0xff, 0xff, 0xfb, 0xf1, 0xf1, 0xff, 0xff, 0xf1,
0xf1, 0xff, 0xff, 0xf1, 0xf1, 0xf1, 0xf1, 0xf1, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf1, 0xf1,
0xf1, 0xf0, 0xf0, 0xf1, 0xf1, 0xf0, 0xf0, 0xf1, 0xf1, 0xf1, 0xf1, 0xfb, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};

void task_ssd1306_display_bitmap() {
    i2c_cmd_handle_t cmd;

    for (uint8_t i = 0; i < 8; i++) {
        cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (OLED_I2C_ADDRESS << 1) | I2C_MASTER_WRITE, true);
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_CMD_SINGLE, true);
        i2c_master_write_byte(cmd, 0xB0 | i, true);
        i2c_master_write_byte(cmd, OLED_CONTROL_BYTE_DATA_STREAM, true);
        i2c_master_write(cmd, &bitmap[i * 128], 128, true);
        i2c_master_stop(cmd);
        i2c_master_cmd_begin(I2C_NUM_0, cmd, 10/portTICK_PERIOD_MS);
        i2c_cmd_link_delete(cmd);
    }
}
/////////////////////////////////////////

// controls the LEDs to reflect the status
static void blink_led(char input)
{
    if (input == '1') {
      gpio_set_level(REDLED, 1);
      gpio_set_level(GREENLED, 0);
      gpio_set_level(YELLOWLED, 0);
    }
    else if (input == '2') {
      gpio_set_level(YELLOWLED, 1);
      gpio_set_level(REDLED, 0);
      gpio_set_level(GREENLED, 0);
    }
    else {
      gpio_set_level(GREENLED, 1);
      gpio_set_level(REDLED, 0);
      gpio_set_level(YELLOWLED, 0);
    }
}

// when receives a fob ID the OLED display will display the QR code and the yellow LED will turn on to show in progress

void recv_task(){
  // Buffer for input data
  uint8_t *data_in = (uint8_t *) malloc(BUF_SIZE2);
  while (1) {
    int len_in = uart_read_bytes(UART_NUM_1, data_in, BUF_SIZE2, 100 / portTICK_PERIOD_MS);
    if (len_in > 0) {
      int nn = 0;
      while (nn < len_in && data_in[nn] != start) {
        nn++;
      }
      if (nn < len_in - len_out + 1) {
        uint8_t copied[len_out];
        memcpy(copied, data_in + nn, len_out * sizeof(uint8_t));
        if (checkCheckSum(copied,len_out)) {
          char FobID = copied[1];
          if (FobID == '1'){
            char MeterFOB[2];
            MeterFOB[0] = FobID;
            MeterFOB[1] = MeterID;
            task_ssd1306_display_clear();
            vTaskDelay(1000/ portTICK_PERIOD_MS);
	// OLED display shows QR code
            task_ssd1306_display_bitmap();
	// LED is yellow
            blink_led('2');
            vTaskDelay(10000/ portTICK_PERIOD_MS);
	// After 10 seconds the screen clears
            task_ssd1306_display_clear();
            printf("First index: %c\n", FobID);
            printf("Second index: %c\n", MeterID);
          }
        }
      }
    }
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
  free(data_in);
}

static void configure_led(void)
{
    gpio_reset_pin(REDLED);
    gpio_reset_pin(YELLOWLED);
    gpio_reset_pin(GREENLED);
    /* Set the GPIO as a push/pull output */
    gpio_set_direction(REDLED, GPIO_MODE_OUTPUT);
    gpio_set_direction(YELLOWLED, GPIO_MODE_OUTPUT);
    gpio_set_direction(GREENLED, GPIO_MODE_OUTPUT);
}

static void udp_client_task(void *pvParameters)
{
    configure_led();
    char rx_buffer[128];
    char host_ip[] = HOST_IP_ADDR;
    int addr_family = 0;
    int ip_protocol = 0;

    while (1) {

#if defined(CONFIG_EXAMPLE_IPV4)
        struct sockaddr_in dest_addr;
        dest_addr.sin_addr.s_addr = inet_addr(HOST_IP_ADDR);
        dest_addr.sin_family = AF_INET;
        dest_addr.sin_port = htons(PORT);
        addr_family = AF_INET;
        ip_protocol = IPPROTO_IP;
#elif defined(CONFIG_EXAMPLE_IPV6)
        struct sockaddr_in6 dest_addr = { 0 };
        inet6_aton(HOST_IP_ADDR, &dest_addr.sin6_addr);
        dest_addr.sin6_family = AF_INET6;
        dest_addr.sin6_port = htons(PORT);
        dest_addr.sin6_scope_id = esp_netif_get_netif_impl_index(EXAMPLE_INTERFACE);
        addr_family = AF_INET6;
        ip_protocol = IPPROTO_IPV6;
#elif defined(CONFIG_EXAMPLE_SOCKET_IP_INPUT_STDIN)
        struct sockaddr_storage dest_addr = { 0 };
        ESP_ERROR_CHECK(get_addr_from_stdin(PORT, SOCK_DGRAM, &ip_protocol, &addr_family, &dest_addr));
#endif

        int sock = socket(addr_family, SOCK_DGRAM, ip_protocol);
        if (sock < 0) {
            ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
            break;
        }

        // Set timeout
        struct timeval timeout;
        timeout.tv_sec = 10;
        timeout.tv_usec = 0;
        setsockopt (sock, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof timeout);

        ESP_LOGI(TAG, "Socket created, sending to %s:%d", HOST_IP_ADDR, PORT);

        while (1) {

            int err = sendto(sock, payload, strlen(payload), 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
            if (err < 0) {
               ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
                break;
             }
             ESP_LOGI(TAG, "Message sent");

            struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
            socklen_t socklen = sizeof(source_addr);
            //ESP_LOGI(TAG, "Before receive: %s", rx_buffer);
            int len = recvfrom(sock, rx_buffer, sizeof(rx_buffer) - 1, 0, (struct sockaddr *)&source_addr, &socklen);
            //ESP_LOGI(TAG, "Received packet: %s", rx_buffer);
            // Error occurred during receiving
            if (len < 0) {
                ESP_LOGE(TAG, "recvfrom failed: errno %d", errno);
                break;
            }
            // Data received
            else {
                rx_buffer[len] = 0; // Null-terminate whatever we received and treat like a string
                char input = rx_buffer[3];
                // printf("this is input %c", input);
                printf("this is rxbuffer %s\n", rx_buffer);
                printf("this is rxbuffer position 2 %c\n", (char)rx_buffer[3]);
                ESP_LOGI(TAG, "Input: %c", input);
                blink_led(input);
                ESP_LOGI(TAG, "Received: %s", rx_buffer);
            }

            vTaskDelay(1000 / portTICK_PERIOD_MS);
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
    // Mutex for current values when sending -- no changes
    mux = xSemaphoreCreateMutex();

    // Timer queue initialize -- 2023: modified
    timer_queue = xQueueCreate(10, sizeof(example_queue_element_t));
    if (!timer_queue) {
        ESP_LOGE(TAG_TIMER, "Creating queue failed");
        return;
    }

    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    /* This helper function configures Wi-Fi or Ethernet, as selected in menuconfig.
     * Read "Establishing Wi-Fi or Ethernet Connection" section in
     * examples/protocols/README.md for more information about this function.
     */
    ESP_ERROR_CHECK(example_connect());
    configure_led();
    uart_init();
    alarm_init();
    i2c_master_init();
	ssd1306_init();
    pwm_init();

    xTaskCreate(udp_client_task, "udp_client", 4096, NULL, 5, NULL);
    xTaskCreate(recv_task, "uart_rx_task", 1024*4, NULL, configMAX_PRIORITIES, NULL);
      vTaskDelay(1000 / portTICK_PERIOD_MS);
}
