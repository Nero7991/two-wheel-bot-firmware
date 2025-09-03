/**
 * Two-Wheel Balancing Robot - Inference Mode
 * 
 * This program uses a pre-trained DQN model from the web simulator
 * to control the balancing robot. No training occurs on the device.
 * 
 * Usage:
 * 1. Train a model using the web simulator
 * 2. Export the model as C++ code
 * 3. Copy the exported .cpp file to this directory (or rename to .h)
 * 4. Update the include path below to your model file
 * 5. Compile and run this program
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/logging/log.h>
#include "i2c_manual_init.h"
#include <zephyr/console/console.h>
#include <stdio.h>
#include <math.h>

#ifndef isnan
#define isnan(x) ((x) != (x))
#endif

#ifndef isfinite
#define isfinite(x) (!(isnan(x)) && !((x) == INFINITY) && !((x) == -INFINITY))
#endif

// Include your exported model file here
// Examples:
// #include "good_two_wheel_bot_dqn_2025-08-17T19-28-58.cpp"
// #include "model.h"
#include "model.cpp"

#include "Motor.h"  // Motor control
#include "KalmanFilter.h"  // Kalman filter for sensor fusion
#include "PIDController.h"  // PID controller
#include "ControlMode.h"  // Control mode management
#include "DataLogger.h"  // Serial data logging


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

LOG_MODULE_REGISTER(main_inference, CONFIG_APP_LOG_LEVEL);

// MPU6050 IMU sensor
#ifdef CONFIG_BOARD_ESP32S3_DEVKITC
// ESP32-S3: Initialize I2C manually for MPU6050
const struct device *mpu6050 = NULL;  // Will be initialized in main
#else
const struct device *const mpu6050 = DEVICE_DT_GET(DT_NODELABEL(mpu6050));
#endif

// Control parameters
#define CONTROL_LOOP_PERIOD_MS 20  // 50Hz control loop
#define ROLLING_AVERAGE_SIZE 2      // Smoothing for sensor readings
#define ANGLE_OFFSET 0.180f         // Calibrated angle offset for vertical position
#define ANGULAR_VELOCITY_OFFSET 0.060f  // Calibrated angular velocity offset for zero movement
#define MAX_MOTOR_SPEED 400       // Maximum motor PWM value
#define GRAPH_RATE_DIVIDER 1      // Data output rate divider (1=50Hz, 2=25Hz, 5=10Hz)

// PID tuning parameters (adjustable via serial commands)
#define DEFAULT_KP 30.0f
#define DEFAULT_KI 0.1f
#define DEFAULT_KD 2.0f

// Kalman filter parameters
#define KALMAN_Q_ANGLE 0.001f
#define KALMAN_Q_VELOCITY 0.003f
#define KALMAN_Q_BIAS 0.00003f
#define KALMAN_R_ANGLE 0.03f

// Safety limits
#define MAX_TILT_ANGLE 0.6f         // ~30 degrees in radians
#define SAFETY_TIMEOUT_MS 300      // Stop if robot falls for this long

// Rolling average buffers
static float angle_buffer[ROLLING_AVERAGE_SIZE] = {0};
static float angular_velocity_buffer[ROLLING_AVERAGE_SIZE] = {0};
static int angle_buffer_index = 0;
static int angular_velocity_buffer_index = 0;

// Pre-trained model instance from the included file
static TwoWheelBotDQN bot;

// Control system instances
static KalmanFilter kalman_filter(CONTROL_LOOP_PERIOD_MS / 1000.0f);
static PIDController pid_controller(CONTROL_LOOP_PERIOD_MS / 1000.0f);
static ControlModeManager control_mode;
static DataLogger data_logger;

// Fallback sensor state for when Kalman fails
static bool use_kalman_filter = true;
static int kalman_failure_count = 0;
static uint32_t kalman_retry_counter = 0;
static const int MAX_KALMAN_FAILURES = 10;

// Safety monitoring
static int consecutive_falls = 0;
static const int MAX_CONSECUTIVE_FALLS = 10;

// Manual control variables
static bool manual_override = false;
static float manual_torque = 0.0f;
#define MANUAL_TORQUE_STRENGTH 0.3f  // Strength of manual control torque

/**
 * Process console input for manual control and tuning
 */
static void process_console_input(void) {
    // Check for console input
    int input_char = console_getchar();
    
    if (input_char >= 0) {        // Got a character from console
        switch (input_char) {
            case 0x1B:  // ESC sequence for arrow keys
                // Read the next two characters for arrow key sequence
                input_char = console_getchar();
                if (input_char == '[') {
                    input_char = console_getchar();
                    switch (input_char) {
                        case 'D':  // Left arrow
                            manual_override = true;
                            manual_torque = -MANUAL_TORQUE_STRENGTH;
                            break;
                        case 'C':  // Right arrow
                            manual_override = true;
                            manual_torque = MANUAL_TORQUE_STRENGTH;
                            break;
                        case 'A':  // Up arrow - increase torque strength
                        case 'B':  // Down arrow - brake
                            manual_override = true;
                            manual_torque = 0.0f;
                            break;
                        default:
                            // Invalid arrow sequence - disable manual
                            manual_override = false;
                            break;
                    }
                } else {
                    // Invalid escape sequence - disable manual
                    manual_override = false;
                }
                break;
            case 'a':  // Alternative: 'a' for left
            case 'A':
                manual_override = true;
                manual_torque = -MANUAL_TORQUE_STRENGTH;
                break;
            case 'd':  // Alternative: 'd' for right
            case 'D':
                manual_override = true;
                manual_torque = MANUAL_TORQUE_STRENGTH;
                break;
            case 's':  // Alternative: 's' for stop/brake
            case 'S':
            case ' ':  // Spacebar for brake
                manual_override = true;
                manual_torque = 0.0f;
                break;
            case 'r':  // 'r' to explicitly disable manual control
            case 'R':
                manual_override = false;
                break;
            case 'm':  // 'm' to toggle between PID and DQN modes
            case 'M':
                control_mode.toggleMode();
                data_logger.logModeChange(
                    control_mode.getPreviousMode() == ControlMode::PID_MODE ? "PID" : "DQN",
                    control_mode.getModeName()
                );
                LOG_INF("Switched to %s mode", control_mode.getModeName());
                break;
            case 'l':  // 'l' to toggle data logging
            case 'L':
                static bool logging_enabled = true;
                logging_enabled = !logging_enabled;
                data_logger.setEnabled(logging_enabled);
                LOG_INF("Data logging %s", logging_enabled ? "enabled" : "disabled");
                break;
            case 'p':  // 'p' to increase Kp
            case 'P':
                {
                    float kp, ki, kd;
                    pid_controller.getGains(kp, ki, kd);
                    kp += 5.0f;
                    pid_controller.setGains(kp, ki, kd);
                    data_logger.logTuning(kp, ki, kd);
                    LOG_INF("PID gains: Kp=%.1f Ki=%.1f Kd=%.1f", (double)kp, (double)ki, (double)kd);
                }
                break;
            case 'i':  // 'i' to increase Ki
            case 'I':
                {
                    float kp, ki, kd;
                    pid_controller.getGains(kp, ki, kd);
                    ki += 0.5f;
                    pid_controller.setGains(kp, ki, kd);
                    data_logger.logTuning(kp, ki, kd);
                    LOG_INF("PID gains: Kp=%.1f Ki=%.1f Kd=%.1f", (double)kp, (double)ki, (double)kd);
                }
                break;
            case 'o':  // 'o' to decrease Kp
            case 'O':
                {
                    float kp, ki, kd;
                    pid_controller.getGains(kp, ki, kd);
                    kp -= 5.0f;
                    if (kp < 0) kp = 0;
                    pid_controller.setGains(kp, ki, kd);
                    data_logger.logTuning(kp, ki, kd);
                    LOG_INF("PID gains: Kp=%.1f Ki=%.1f Kd=%.1f", (double)kp, (double)ki, (double)kd);
                }
                break;
            case 'u':  // 'u' to decrease Ki
            case 'U':
                {
                    float kp, ki, kd;
                    pid_controller.getGains(kp, ki, kd);
                    ki -= 0.5f;
                    if (ki < 0) ki = 0;
                    pid_controller.setGains(kp, ki, kd);
                    data_logger.logTuning(kp, ki, kd);
                    LOG_INF("PID gains: Kp=%.1f Ki=%.1f Kd=%.1f", (double)kp, (double)ki, (double)kd);
                }
                break;
            case 'h':  // 'h' for help
            case 'H':
            case '?':
                LOG_INF("Controls:");
                LOG_INF("  A/D or arrows: Manual left/right");
                LOG_INF("  S or Space: Brake");
                LOG_INF("  M: Toggle PID/DQN mode");
                LOG_INF("  L: Toggle data logging");
                LOG_INF("  P/O: Increase/decrease Kp");
                LOG_INF("  I/U: Increase/decrease Ki");
                LOG_INF("  R: Resume auto control");
                LOG_INF("  H/?: Show this help");
                break;
            default:
                // Keep current state for unrecognized keys
                break;
        }
        
        // Clear any remaining characters in buffer to prevent buildup
        while (console_getchar() >= 0) {
            // Drain buffer
        }
    } else {
        // No new input - disable manual control
        manual_override = false;
    }
}

/**
 * Calculate rolling average
 */
static float rolling_average(float *buffer, int size, float new_value, int *index) {
    buffer[*index] = new_value;
    *index = (*index + 1) % size;
    
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
}

/**
 * Validate sensor reading for NaN and extreme values
 */
static bool validate_sensor_reading(float value, float min_val, float max_val) {
    if (isnan(value) || !isfinite(value)) {
        return false;
    }
    if (value < min_val || value > max_val) {
        return false;
    }
    return true;
}

/**
 * Get tilt angle from MPU6050
 */
static int get_angle(const struct device *dev, float &angle) {
    struct sensor_value accel[3];
    int rc = sensor_sample_fetch(dev);
    
    if (rc == -EBADMSG) {
        LOG_ERR("MPU6050 fetch error: %d", rc);
        return rc;
    }
    
    if (rc == 0) {
        rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        if (rc == 0) {
            // Convert accelerometer reading to angle
            // Assuming Y-axis is vertical when robot is upright
            float raw_accel = (float)(sensor_value_to_double(&accel[1]) / 9.81);
            
            // Validate accelerometer reading
            if (!validate_sensor_reading(raw_accel, -2.0f, 2.0f)) {
                LOG_WRN("Invalid accelerometer reading: %.3f", (double)raw_accel);
                return -EINVAL;
            }
            
            // Clamp to valid range for asin
            if (raw_accel > 1.0f) raw_accel = 1.0f;
            if (raw_accel < -1.0f) raw_accel = -1.0f;
            
            // Calculate raw angle
            float calculated_angle = asinf(raw_accel);
            
            // Validate calculated angle
            if (!validate_sensor_reading(calculated_angle, -M_PI/2, M_PI/2)) {
                LOG_WRN("Invalid calculated angle: %.3f", (double)calculated_angle);
                return -EINVAL;
            }
            
            // Apply rolling average for smoothing
            float smoothed_angle = rolling_average(angle_buffer, ROLLING_AVERAGE_SIZE, calculated_angle, &angle_buffer_index);
            
            // Validate smoothed result
            if (!validate_sensor_reading(smoothed_angle, -M_PI, M_PI)) {
                LOG_WRN("Invalid smoothed angle: %.3f", (double)smoothed_angle);
                return -EINVAL;
            }
            
            // Apply offset after smoothing
            angle = smoothed_angle - ANGLE_OFFSET;
            
            // Final validation
            if (!validate_sensor_reading(angle, -M_PI, M_PI)) {
                LOG_WRN("Invalid final angle: %.3f", (double)angle);
                return -EINVAL;
            }
        }
    }
    
    return rc;
}

/**
 * Get angular velocity from MPU6050
 */
static int get_angular_velocity(const struct device *dev, float &angular_velocity) {
    struct sensor_value gyro[3];
    int rc = sensor_sample_fetch(dev);
    
    if (rc == -EBADMSG) {
        LOG_ERR("MPU6050 fetch error: %d", rc);
        return rc;
    }
    
    if (rc == 0) {
        rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
        if (rc == 0) {
            // Get angular velocity around X-axis (pitch)
            float raw_velocity = sensor_value_to_double(&gyro[0]);
            
            // Validate raw gyro reading
            if (!validate_sensor_reading(raw_velocity, -50.0f, 50.0f)) {
                LOG_WRN("Invalid gyro reading: %.3f", (double)raw_velocity);
                return -EINVAL;
            }
            
            // Apply rolling average for smoothing
            float smoothed_velocity = rolling_average(angular_velocity_buffer, 
                                                    ROLLING_AVERAGE_SIZE, raw_velocity, 
                                                    &angular_velocity_buffer_index);
            
            // Validate smoothed result
            if (!validate_sensor_reading(smoothed_velocity, -50.0f, 50.0f)) {
                LOG_WRN("Invalid smoothed velocity: %.3f", (double)smoothed_velocity);
                return -EINVAL;
            }
            
            // Apply offset after smoothing
            angular_velocity = smoothed_velocity - ANGULAR_VELOCITY_OFFSET;
            
            // Final validation
            if (!validate_sensor_reading(angular_velocity, -50.0f, 50.0f)) {
                LOG_WRN("Invalid final velocity: %.3f", (double)angular_velocity);
                return -EINVAL;
            }
        }
    }
    
    return rc;
}

/**
 * Check if robot has fallen
 */
static bool has_fallen(float angle) {
    return fabsf(angle) > MAX_TILT_ANGLE;
}

/**
 * Apply motor control based on selected control mode
 * Returns the torque applied and motor speeds via parameters
 */
static float apply_control(float raw_angle, float raw_gyro, 
                         float kalman_angle, float kalman_velocity,
                         int16_t& motor_left, int16_t& motor_right,
                         float& pid_p, float& pid_i, float& pid_d) {
    float torque = 0.0f;
    int16_t motor_speed = 0;
    
    if (manual_override) {
        // Use manual torque input
        torque = manual_torque;
        motor_speed = -(int16_t)(torque * MAX_MOTOR_SPEED);
        motor_left = motor_right = motor_speed;
        
        // Apply to both motors
        set_motor_speed(motor_speed, motor_speed);
        
        #ifdef DEBUG_INFERENCE
        LOG_INF("MANUAL: Torque: %.2f Speed: %d", (double)torque, motor_speed);
        #endif
    } else if (control_mode.getMode() == ControlMode::PID_MODE) {
        // PID control with Kalman filtered data
        torque = pid_controller.calculate(kalman_angle, kalman_velocity, 0.0f);
        
        // Get PID components for logging
        pid_controller.getComponents(kalman_angle, kalman_velocity, 0.0f,
                                    pid_p, pid_i, pid_d);
        
        // Convert torque to motor speeds
        motor_speed = (int16_t)(torque * MAX_MOTOR_SPEED);
        motor_left = motor_right = motor_speed;
        
        // Apply to both motors
        set_motor_speed(motor_speed, motor_speed);
        
        #ifdef DEBUG_INFERENCE
        LOG_INF("PID: Angle: %.3f Vel: %.3f Torque: %.2f Speed: %d",
                (double)kalman_angle, (double)kalman_velocity, (double)torque, motor_speed);
        #endif
    } else if (control_mode.getMode() == ControlMode::DQN_MODE) {
        // DQN model control using Kalman filtered data
        int action = bot.getAction(kalman_angle, kalman_velocity);
        torque = bot.getMotorTorque(action);
        
        // Convert torque to motor speeds
        motor_speed = -(int16_t)(torque * MAX_MOTOR_SPEED);
        motor_left = motor_right = motor_speed;
        
        // Apply to both motors
        set_motor_speed(motor_speed, motor_speed);
        
        #ifdef DEBUG_INFERENCE
        LOG_INF("DQN: State: [%.3f, %.3f] Action: %d Torque: %.2f",
                (double)kalman_angle, (double)kalman_velocity, action, (double)torque);
        #endif
    } else {
        // Disabled mode - stop motors
        motor_left = motor_right = 0;
        set_motor_speed(0, 0);
    }
    
    return torque;
}

/**
 * Safety check and recovery
 */
static bool safety_check(float angle) {
    if (has_fallen(angle)) {
        consecutive_falls++;
        
        if (consecutive_falls > MAX_CONSECUTIVE_FALLS) {
            LOG_WRN("Robot has fallen. Stopping motors for safety.");
            set_motor_speed(0, 0);
            return false;
        }
    } else {
        consecutive_falls = 0;
    }
    
    return true;
}

/**
 * Calibration routine
 */
// static void calibrate_sensors(void) {
//     LOG_INF("Calibrating sensors... Keep robot upright and still.");
//     
//     // Clear buffers
//     for (int i = 0; i < ROLLING_AVERAGE_SIZE; i++) {
//         angle_buffer[i] = 0;
//         angular_velocity_buffer[i] = 0;
//     }
//     
//     // Take initial readings to populate buffers
//     float angle, angular_velocity;
//     for (int i = 0; i < ROLLING_AVERAGE_SIZE * 2; i++) {
//         get_angle(mpu6050, angle);
//         get_angular_velocity(mpu6050, angular_velocity);
//         k_sleep(K_MSEC(20));
//     }
//     
//     LOG_INF("Calibration complete. Starting control loop.");
// }

/**
 * Run simulation mode for benchmarking inference speed
 */
void run_simulation_mode(void) {
    LOG_INF("=== SIMULATION MODE ===");
    LOG_INF("Hardware not available - running inference benchmark");
    
    // Simulated sensor read time (typical MPU6050 I2C transaction)
    const uint32_t SENSOR_READ_TIME_US = 500;  // 0.5ms for I2C read
    const uint32_t MOTOR_WRITE_TIME_US = 100;  // 0.1ms for PWM update
    
    // Test states covering different scenarios
    struct TestState {
        float angle;
        float angular_velocity;
        const char* description;
    };
    
    TestState test_states[] = {
        {0.0f, 0.0f, "Perfectly balanced"},
        {0.1f, 0.0f, "Slight forward tilt"},
        {-0.1f, 0.0f, "Slight backward tilt"},
        {0.2f, 0.5f, "Forward tilt with velocity"},
        {-0.2f, -0.5f, "Backward tilt with velocity"},
        {0.3f, 1.0f, "Large forward motion"},
        {-0.3f, -1.0f, "Large backward motion"},
        {0.05f, 0.2f, "Small disturbance"}
    };
    
    const int num_test_states = sizeof(test_states) / sizeof(test_states[0]);
    
    LOG_INF("Running inference benchmark...");
    LOG_INF("Simulated sensor read: %d us", SENSOR_READ_TIME_US);
    LOG_INF("Simulated motor write: %d us", MOTOR_WRITE_TIME_US);
    
    // Warm-up runs
    for (int i = 0; i < 10; i++) {
        int action = bot.getAction(0.1f, 0.1f);
        (void)action; // Suppress unused warning
    }
    
    // Benchmark different scenarios
    LOG_INF("\n--- Inference Performance ---");
    
    for (int i = 0; i < num_test_states; i++) {
        uint32_t start_time = k_cycle_get_32();
        
        // Inference
        int action = bot.getAction(test_states[i].angle, 
                                 test_states[i].angular_velocity);
        float torque = bot.getMotorTorque(action);
        
        uint32_t end_time = k_cycle_get_32();
        uint32_t cycles = end_time - start_time;
        
        // Convert cycles to microseconds (assuming 48MHz clock for STM32WL)
        uint32_t inference_time_us = (cycles * 1000) / 48000;
        
        LOG_INF("State: [%0.2f, %0.2f] -> Action: %d (%.1f) - %d us - %s",
                (double)test_states[i].angle,
                (double)test_states[i].angular_velocity,
                action,
                (double)torque,
                inference_time_us,
                test_states[i].description);
    }
    
    // Run continuous benchmark for 1000 iterations
    LOG_INF("\n--- Throughput Test (1000 iterations) ---");
    
    uint32_t total_start = k_cycle_get_32();
    int action_counts[3] = {0, 0, 0};
    
    for (int iter = 0; iter < 1000; iter++) {
        // Cycle through test states
        TestState& state = test_states[iter % num_test_states];
        
        // Simulate sensor read delay
        k_busy_wait(SENSOR_READ_TIME_US);
        
        // Run inference
        int action = bot.getAction(state.angle, state.angular_velocity);
        action_counts[action]++;
        
        // Simulate motor write delay
        k_busy_wait(MOTOR_WRITE_TIME_US);
    }
    
    uint32_t total_end = k_cycle_get_32();
    uint32_t total_cycles = total_end - total_start;
    uint32_t total_time_ms = (total_cycles / 48000);
    
    LOG_INF("1000 iterations completed in %d ms", total_time_ms);
    LOG_INF("Average time per iteration: %d us", total_time_ms * 1000 / 1000);
    LOG_INF("Theoretical max frequency: %d Hz", 1000000 / (total_time_ms * 1000 / 1000));
    LOG_INF("Action distribution: Left=%d, Brake=%d, Right=%d",
            action_counts[0], action_counts[1], action_counts[2]);
    
    // Test with 50Hz control loop
    LOG_INF("\n--- 50Hz Control Loop Simulation (10 seconds) ---");
    
    uint32_t loop_count = 0;
    uint32_t missed_deadlines = 0;
    uint32_t sim_start = k_uptime_get_32();
    
    while (k_uptime_get_32() - sim_start < 10000) { // 10 seconds
        uint32_t loop_start = k_uptime_get_32();
        
        // Simulate full control cycle
        k_busy_wait(SENSOR_READ_TIME_US);
        
        // Get simulated state (sinusoidal motion)
        float t = (float)loop_count * 0.02f; // 20ms per loop
        float angle = 0.2f * sinf(t * 2.0f);
        float angular_velocity = 0.4f * cosf(t * 2.0f);
        
        int action = bot.getAction(angle, angular_velocity);
        float torque = bot.getMotorTorque(action);
        (void)torque; // Suppress unused warning
        
        k_busy_wait(MOTOR_WRITE_TIME_US);
        
        // Check if we met the 20ms deadline
        uint32_t loop_time = k_uptime_get_32() - loop_start;
        if (loop_time > CONTROL_LOOP_PERIOD_MS) {
            missed_deadlines++;
        }
        
        loop_count++;
        
        // Maintain 50Hz timing
        if (loop_time < CONTROL_LOOP_PERIOD_MS) {
            k_sleep(K_MSEC(CONTROL_LOOP_PERIOD_MS - loop_time));
        }
        
        // Status every second
        if (loop_count % 50 == 0) {
            LOG_INF("Sim time: %d s, Loops: %d, Missed: %d",
                    loop_count / 50, loop_count, missed_deadlines);
        }
    }
    
    LOG_INF("\n--- Simulation Complete ---");
    LOG_INF("Total loops: %d", loop_count);
    LOG_INF("Missed deadlines: %d (%.1f%%)", 
            missed_deadlines, 
            (double)((float)missed_deadlines * 100.0f / loop_count));
    LOG_INF("Effective rate: %.1f Hz", 
            (double)((float)loop_count / 10.0f));
    
    LOG_INF("\nSimulation mode complete. System halted.");
    
    // Blink LED to indicate simulation mode
    float sim_angle = 0.0f, sim_angular_velocity = 0.0f;
    while (true) {
        k_sleep(K_MSEC(200));
        
        // Simulate changing angle and angular velocity for demo
        float t = (float)k_uptime_get_32() / 1000.0f; // Time in seconds
        sim_angle = 0.2f * sinf(t * 0.5f);
        sim_angular_velocity = 0.1f * cosf(t * 0.5f);
        
        LOG_INF("Status: Angle=%.3f rad, Vel=%.3f rad/s", (double)sim_angle, (double)sim_angular_velocity);
    }
}

int main(void) {
    LOG_INF("Two-Wheel Balancing Robot - Multi-Mode Control");
    LOG_INF("Modes: PID (default), DQN, Manual");
    
    bool hardware_available = true;
    bool simulation_mode = false;
    
#ifdef CONFIG_BOARD_ESP32S3_DEVKITC
    // ESP32-S3: Check hardware availability
    LOG_INF("ESP32-S3 DevKitC detected");
    LOG_INF("I2C pins: GPIO8 (SDA), GPIO9 (SCL)");
    
    // Try manual I2C initialization first
    int i2c_result = manual_i2c_init();
    if (i2c_result != 0) {
        LOG_WRN("Manual I2C initialization failed: %d", i2c_result);
    }
    
    // Get MPU6050 device from device tree
#if DT_NODE_EXISTS(DT_NODELABEL(mpu6050))
    mpu6050 = DEVICE_DT_GET(DT_NODELABEL(mpu6050));
#else
    mpu6050 = NULL;
#endif
    if (!mpu6050 || !device_is_ready(mpu6050)) {
        LOG_WRN("MPU6050 device not ready - will use simulation mode");
        hardware_available = false;
        simulation_mode = true;
    } else {
        LOG_INF("MPU6050 ready");
        
        // Try to read from MPU6050 to verify it's working
        struct sensor_value test_val __attribute__((unused));
        int rc = sensor_sample_fetch(mpu6050);
        if (rc != 0) {
            LOG_WRN("MPU6050 fetch failed (%d) - will use simulation mode", rc);
            hardware_available = false;
            simulation_mode = true;
        } else {
            LOG_INF("MPU6050 sensor communication verified");
        }
    }
#else
    // Check MPU6050 availability
    if (!device_is_ready(mpu6050)) {
        LOG_WRN("MPU6050 device not ready - will use simulation mode");
        hardware_available = false;
        simulation_mode = true;
    } else {
        LOG_INF("MPU6050 ready");
        
        // Try to read from MPU6050 to verify it's working
        struct sensor_value test_val __attribute__((unused));
        int rc = sensor_sample_fetch(mpu6050);
        if (rc != 0) {
            LOG_WRN("MPU6050 fetch failed (%d) - will use simulation mode", rc);
            hardware_available = false;
            simulation_mode = true;
        }
    }
#endif
    
    // Try to initialize motor driver
    if (hardware_available) {
        int motor_result = initialize_pwm_driver();
        if (motor_result != 0) {
            LOG_WRN("Motor driver init failed - will use simulation mode");
            hardware_available = false;
            simulation_mode = true;
        } else {
            LOG_INF("Motor driver initialized");
        }
    }
    
    // Enter simulation mode if hardware not available
    if (simulation_mode) {
        run_simulation_mode();
        return 0; // Never reached
    }
    
    // Normal operation with hardware
    LOG_INF("Hardware detected - running normal inference mode");
    
    // Initialize console for manual control
    console_init();
    LOG_INF("Console initialized for control");
    LOG_INF("Press H or ? for help on controls");
    
    // Initialize control systems
    kalman_filter.setProcessNoise(KALMAN_Q_ANGLE, KALMAN_Q_VELOCITY, KALMAN_Q_BIAS);
    kalman_filter.setMeasurementNoise(KALMAN_R_ANGLE);
    
    pid_controller.setGains(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    pid_controller.setOutputLimits(-1.0f, 1.0f);
    pid_controller.setIntegralLimit(50.0f);
    
    // Start data logging
    data_logger.setEnabled(true);
    data_logger.setInterval(CONTROL_LOOP_PERIOD_MS * GRAPH_RATE_DIVIDER);  // Log at control rate divided by graph rate divider
    data_logger.logEvent("System initialized");
    data_logger.logTuning(DEFAULT_KP, DEFAULT_KI, DEFAULT_KD);
    data_logger.logKalmanTuning(KALMAN_Q_ANGLE, KALMAN_Q_VELOCITY, KALMAN_Q_BIAS, KALMAN_R_ANGLE);
    
    LOG_INF("Default mode: %s", control_mode.getModeName());
    
    // Skip calibration since we have preset offsets
    LOG_INF("Using preset sensor offsets - skipping calibration");
    kalman_filter.setInitialAngle(-ANGLE_OFFSET);  // Set initial angle based on calibration
    
    // Main control loop
    LOG_INF("Starting balance control...");
    
    float raw_angle, raw_angular_velocity;
    float kalman_angle, kalman_velocity, kalman_bias;
    uint32_t loop_start_time;
    uint32_t total_iterations = 0;
    uint32_t successful_iterations = 0;
    
    while (true) {
        loop_start_time = k_uptime_get_32();
        
        // Process console input for manual control and commands
        //process_console_input();
        
        // Read raw sensors with validation
        int rc = get_angle(mpu6050, raw_angle);
        if (rc != 0) {
            LOG_ERR("Failed to read angle: %d", rc);
            
            // If sensor read fails, use safe default
            raw_angle = 0.0f;
            LOG_WRN("Sensor read failed, using default angle: 0.0");
        }
        
        rc = get_angular_velocity(mpu6050, raw_angular_velocity);
        if (rc != 0) {
            LOG_ERR("Failed to read angular velocity: %d", rc);
            
            // If sensor read fails, use safe default
            raw_angular_velocity = 0.0f;
            LOG_WRN("Sensor read failed, using default velocity: 0.0");
        }
        
        // Decide whether to use Kalman filter or raw sensors
        if (use_kalman_filter) {
            // Update Kalman filter with validation
            kalman_filter.update(raw_angle, raw_angular_velocity);
            
            // Check filter health and get state
            if (!kalman_filter.isHealthy()) {
                kalman_failure_count++;
                LOG_WRN("Kalman filter unhealthy (NaN count: %d, failures: %d)", 
                       kalman_filter.getNaNCount(), kalman_failure_count);
                
                // Use raw sensor data as fallback
                kalman_angle = raw_angle;
                kalman_velocity = raw_angular_velocity;
                kalman_bias = 0.0f;
                
                // Disable Kalman if too many failures
                if (kalman_failure_count >= MAX_KALMAN_FAILURES) {
                    use_kalman_filter = false;
                    LOG_WRN("Disabling Kalman filter due to persistent failures");
                    data_logger.logEvent("Kalman filter disabled - too many failures");
                }
            } else {
                kalman_filter.getState(kalman_angle, kalman_velocity, kalman_bias);
                
                // Double-check for NaN in outputs
                if (isnan(kalman_angle) || isnan(kalman_velocity) || isnan(kalman_bias)) {
                    kalman_failure_count++;
                    LOG_WRN("Kalman outputs contain NaN (failures: %d), falling back to raw sensors", kalman_failure_count);
                    
                    kalman_angle = raw_angle;
                    kalman_velocity = raw_angular_velocity;
                    kalman_bias = 0.0f;
                    
                    // Reset the filter
                    kalman_filter.reset();
                    
                    // Disable Kalman if too many failures
                    if (kalman_failure_count >= MAX_KALMAN_FAILURES) {
                        use_kalman_filter = false;
                        LOG_WRN("Disabling Kalman filter due to NaN outputs");
                        data_logger.logEvent("Kalman filter disabled - NaN outputs");
                    }
                } else {
                    // Filter is working well, reset failure count
                    if (kalman_failure_count > 0) {
                        kalman_failure_count--;
                    }
                }
            }
        } else {
            // Use raw sensors directly (Kalman disabled)
            kalman_angle = raw_angle;
            kalman_velocity = raw_angular_velocity;
            kalman_bias = 0.0f;
            
            // Periodically try to re-enable Kalman filter
            kalman_retry_counter++;
            
            if (kalman_retry_counter >= 500) {  // Try every 10 seconds at 50Hz
                kalman_retry_counter = 0;
                kalman_failure_count = 0;
                use_kalman_filter = true;
                kalman_filter.reset();
                LOG_INF("Attempting to re-enable Kalman filter");
                data_logger.logEvent("Kalman filter re-enabled - retry");
            }
        }
        
        // Debug: Print sensor values every 50 iterations (1 second at 50Hz)
        if (total_iterations % 50 == 0) {
            const char* filter_status = use_kalman_filter ? 
                (kalman_filter.isHealthy() ? "HEALTHY" : "DEGRADED") : "DISABLED";
            
            LOG_INF("Sensors - Raw angle: %.3f, Kalman angle: %.3f, Max tilt: %.3f, Filter: %s", 
                   (double)raw_angle, (double)kalman_angle, (double)MAX_TILT_ANGLE, filter_status);
        }
        
        // Safety check using filtered angle
        bool fallen = has_fallen(kalman_angle);
        
        // Apply control based on mode (before safety check so we always get data)
        int16_t motor_left = 0, motor_right = 0;
        float pid_p = 0, pid_i = 0, pid_d = 0;
        float torque = 0.0f;
        
        if (!fallen) {
            torque = apply_control(raw_angle, raw_angular_velocity,
                                kalman_angle, kalman_velocity,
                                motor_left, motor_right,
                                pid_p, pid_i, pid_d);
        } else {
            // Robot has fallen - stop motors but still log data
            set_motor_speed(0, 0);
            motor_left = motor_right = 0;
        }
        
        // Log data for plotting (always log, even when fallen)
        const char* mode_str = fallen ? "FALLEN" : 
                              (manual_override ? "MANUAL" : 
                              (!use_kalman_filter ? "RAW" : control_mode.getModeName()));
        
        data_logger.log(
            loop_start_time,
            mode_str,
            raw_angle,
            raw_angular_velocity,
            kalman_angle,
            kalman_velocity,
            kalman_bias,
            torque,
            pid_p,
            pid_i,
            pid_d,
            motor_left,
            motor_right,
            0.0f,  // angle_setpoint
            fallen
        );
        
        // Safety recovery logic
        if (!safety_check(kalman_angle)) {
            // Try to recover after a pause
            data_logger.logEvent("Robot fallen - attempting recovery");
            pid_controller.reset();  // Reset PID integral term
            k_sleep(K_MSEC(1000));
            consecutive_falls = 0;
            LOG_INF("Attempting to restart balance control...");
            continue;
        }
        
        // Track statistics
        total_iterations++;
        if (!fallen) {
            successful_iterations++;
        }
        
        // Print status every 2 seconds (100 iterations at 50Hz)
        if (total_iterations % 100 == 0) {
            float success_rate = (float)successful_iterations / total_iterations * 100;
            const char* status_mode = use_kalman_filter ? control_mode.getModeName() : "RAW-SENSOR";
            LOG_INF("Mode: %s | Success: %.1f%% | Bias: %.4f rad/s | Filter failures: %d",
                   status_mode, (double)success_rate, (double)kalman_bias, kalman_failure_count);
        }
        
        // Maintain control loop timing
        uint32_t loop_time = k_uptime_get_32() - loop_start_time;
        if (loop_time < CONTROL_LOOP_PERIOD_MS) {
            k_sleep(K_MSEC(CONTROL_LOOP_PERIOD_MS - loop_time));
        }
    }
    
    return 0;
}