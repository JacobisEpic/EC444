#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/mcpwm_prelude.h"
static const char *TAG = "example";
// Consult the ESC before you change these parameters for the buggy speed control
// Please consult the datasheet of your servo before changing the following parameters
#define SERVO_MIN_PULSEWIDTH_US 500  // Minimum pulse width in microsecond
#define SERVO_MAX_PULSEWIDTH_US 2500  // Maximum pulse width in microsecond
#define SERVO_MIN_DEGREE        -90   // Minimum angle
#define SERVO_MAX_DEGREE        90    // Maximum angle
#define ESC_GPIO             25        //GPIO for esc
#define STEER_GPIO           26        //GPIO for steer
#define SERVO_TIMEBASE_RESOLUTION_HZ 1000000  // 1MHz, 1us per tick
#define SERVO_TIMEBASE_PERIOD        20000    // 20000 ticks, 20ms
static inline uint32_t example_angle_to_compare(int angle)
{
    return (angle - SERVO_MIN_DEGREE) * (SERVO_MAX_PULSEWIDTH_US - SERVO_MIN_PULSEWIDTH_US)
      / (SERVO_MAX_DEGREE - SERVO_MIN_DEGREE) + SERVO_MIN_PULSEWIDTH_US;
}
void app_main(void)
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
    char input;
    //int speed = 0;
    int32_t speed = 0;
    int32_t steerAngle = 0;
    //calibrate esc_______________________________________________________________________________//
    printf("Calibrating\n");
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(0)));
    vTaskDelay(pdMS_TO_TICKS(3600));
    printf("Leaving at center position\n");
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(0)));
    ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(0)));
    //____________________________________________________________________________________________//
    while (1) {// This code changes the PWM slowly to allow the (mechanical) servo to keep up
        printf("Enter speed: \n");
        scanf("%c",&input);
        if (input == 'w' && speed < SERVO_MAX_DEGREE)
        {
            speed += 15;
            input = 'p';
            if (speed == 0) // Checking when at midpoint after already moving in the other direction
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(speed)));
                vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(0)));
                vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(speed)));
            } else {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(speed)));
            }
            // ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, example_angle_to_compare(speed)));
            ESP_LOGI(TAG, "Speed set to: %ld", speed); //print speed
        }
        else if (input == 's' && speed > SERVO_MIN_DEGREE)
        {
            speed -= 15;
            input = 'p';
            if (speed == 0)
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(speed)));
                vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(0)));
                vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(speed)));
            } else {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(esc, example_angle_to_compare(speed)));
            }
            ESP_LOGI(TAG, "Speed set to: %ld", speed); //print speed
        }
        else if (input == 'd') {
            steerAngle -= 15;
            if (steerAngle == 0)
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
                vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer,example_angle_to_compare(0)));
                vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
            } else {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
            }
            input = 'p';
        }
        else if (input == 'a') {
            steerAngle += 15;
            if (steerAngle == 0)
            {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
                vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(0)));
                vTaskDelay(pdMS_TO_TICKS(10));
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
            } else {
                ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(steer, example_angle_to_compare(steerAngle)));
            }
            input = 'p';
        }
        // Add delay, since it takes time for servo to rotate, usually 200ms/60degree rotation under 5V power supply
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}