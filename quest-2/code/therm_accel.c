
// Includes go here
#include <stdio.h>
#include <math.h>
#include "driver/i2c.h"
#include "./ADXL343.h"
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/adc.h"
#include "esp_adc_cal.h"

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

#define DEFAULT_VREF    1100        //Use adc2_vref_to_gpio() to obtain a better estimate
#define NO_OF_SAMPLES   64          //Multisampling
#define SERIESRESISTOR 10000
#define BCOEFFICIENT 3950
#define TEMPERATURENOMINAL 25
#define THERMISTORNOMINAL 10000 

// ADXL343
#define SLAVE_ADDR                         ADXL343_ADDRESS // 0x53

// Other defines
#define STEP_THRESHOLD 2.1

// Global variables go here
float lastX;
float currX;
int numSteps = 0;
float steinhart;
static esp_adc_cal_characteristics_t *adc_chars;
static const adc_channel_t channel = ADC_CHANNEL_6;     //GPIO34 if ADC1, GPIO14 if ADC2
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_2;

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

// Function to initiate i2c -- note the MSB declaration!
static void i2c_master_init(){
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
   conf.clk_flags = 0; 
  err = i2c_param_config(i2c_master_port, &conf);           // Configure
  if (err == ESP_OK) {printf("- parameters: ok\n");}

  // Install I2C driver
  err = i2c_driver_install(i2c_master_port, conf.mode,
                     I2C_EXAMPLE_MASTER_RX_BUF_DISABLE,
                     I2C_EXAMPLE_MASTER_TX_BUF_DISABLE, 0);
  if (err == ESP_OK) {printf("- initialized: yes\n");}

  // Data in MSB mode
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
  printf("\n>> I2C scanning ..."  "\n");
  uint8_t count = 0;
  for (uint8_t i = 1; i < 127; i++) {
    // printf("0x%X%s",i,"\n");
    if (testConnection(i, scanTimeout) == ESP_OK) {
      printf( "- Device found at address: 0x%X%s", i, "\n");
      count++;
    }
  }
  if (count == 0) {printf("- No I2C devices found!" "\n");}
}

// ADXL343 Functions ///////////////////////////////////////////////////////////

// Get Device ID
int getDeviceID(uint8_t *data) {
  int ret;
  i2c_cmd_handle_t cmd = i2c_cmd_link_create();
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
  i2c_master_write_byte(cmd, ADXL343_REG_DEVID, ACK_CHECK_EN);
  i2c_master_start(cmd);
  i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
  i2c_master_read_byte(cmd, data, ACK_CHECK_DIS);
  i2c_master_stop(cmd);
  ret = i2c_master_cmd_begin(I2C_EXAMPLE_MASTER_NUM, cmd, 1000 / portTICK_PERIOD_MS);
  i2c_cmd_link_delete(cmd);
  return ret;
}

// Write one byte to register
void writeRegister(uint8_t reg, uint8_t data) {
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
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
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | WRITE_BIT, ACK_CHECK_EN);
    i2c_master_write_byte(cmd, reg, ACK_CHECK_EN); // Register that will be read from
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ( SLAVE_ADDR << 1 ) | READ_BIT, ACK_CHECK_EN);
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
  printf("\n>> Polling ADAXL343\n");
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
  check_efuse();

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
  print_char_val_type(val_type);
}

static void init_check()
{
  // Routine
  i2c_master_init();
  i2c_scanner();

  // Check for ADXL343
  uint8_t deviceID;
  getDeviceID(&deviceID);
  if (deviceID == 0xE5) {
    printf("\n>> Found ADAXL343\n");
  }
  // Disable interrupts
  writeRegister(ADXL343_REG_INT_ENABLE, 0);

  // Set range
  setRange(ADXL343_RANGE_16_G);
  // Display range
  printf  ("- Range:         +/- ");
  switch(getRange()) {
    case ADXL343_RANGE_16_G:
      printf  ("16 ");
      break;
    case ADXL343_RANGE_8_G:
      printf  ("8 ");
      break;
    case ADXL343_RANGE_4_G:
      printf  ("4 ");
      break;
    case ADXL343_RANGE_2_G:
      printf  ("2 ");
      break;
    default:
      printf  ("?? ");
      break;
  }
  printf(" g\n");

  // Display data rate
  printf ("- Data Rate:    ");
  switch(getDataRate()) {
    case ADXL343_DATARATE_3200_HZ:
      printf  ("3200 ");
      break;
    case ADXL343_DATARATE_1600_HZ:
      printf  ("1600 ");
      break;
    case ADXL343_DATARATE_800_HZ:
      printf  ("800 ");
      break;
    case ADXL343_DATARATE_400_HZ:
      printf  ("400 ");
      break;
    case ADXL343_DATARATE_200_HZ:
      printf  ("200 ");
      break;
    case ADXL343_DATARATE_100_HZ:
      printf  ("100 ");
      break;
    case ADXL343_DATARATE_50_HZ:
      printf  ("50 ");
      break;
    case ADXL343_DATARATE_25_HZ:
      printf  ("25 ");
      break;
    case ADXL343_DATARATE_12_5_HZ:
      printf  ("12.5 ");
      break;
    case ADXL343_DATARATE_6_25HZ:
      printf  ("6.25 ");
      break;
    case ADXL343_DATARATE_3_13_HZ:
      printf  ("3.13 ");
      break;
    case ADXL343_DATARATE_1_56_HZ:
      printf  ("1.56 ");
      break;
    case ADXL343_DATARATE_0_78_HZ:
      printf  ("0.78 ");
      break;
    case ADXL343_DATARATE_0_39_HZ:
      printf  ("0.39 ");
      break;
    case ADXL343_DATARATE_0_20_HZ:
      printf  ("0.20 ");
      break;
    case ADXL343_DATARATE_0_10_HZ:
      printf  ("0.10 ");
      break;
    default:
      printf  ("???? ");
      break;
  }
  printf(" Hz\n\n");

  // Enable measurements
  writeRegister(ADXL343_REG_POWER_CTL, 0x08);
}

void app_main() {
  init_check();
  init_temp();

  // Create task to poll ADXL343
  // xTaskCreate(read_temp, "read_temp", 4096, NULL, configMAX_PRIORITIES-1, NULL);
  xTaskCreate(test_adxl343,"test_adxl343", 4096, NULL, configMAX_PRIORITIES-1, NULL);
  xTaskCreate(timed_print, "timed_print", 4096, NULL , configMAX_PRIORITIES, NULL);
}

