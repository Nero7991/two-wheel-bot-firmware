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
#include <stdio.h>
#include <math.h>

// Include your exported model file here
// Examples:
// #include "good_two_wheel_bot_dqn_2025-08-17T19-28-58.cpp"
// #include "model.h"
#include "model.cpp"

#include "Motor.h"  // Motor control

LOG_MODULE_REGISTER(main_inference, CONFIG_APP_LOG_LEVEL);

// MPU6050 IMU sensor
const struct device *const mpu6050 = DEVICE_DT_GET_ONE(invensense_mpu6050);

// Control parameters
#define CONTROL_LOOP_PERIOD_MS 20  // 50Hz control loop
#define ROLLING_AVERAGE_SIZE 4      // Smoothing for sensor readings
#define ANGLE_OFFSET 0.115f         // Calibrated angle offset for vertical position
#define MAX_MOTOR_SPEED 300       // Maximum motor PWM value

// Safety limits
#define MAX_TILT_ANGLE 0.5f         // ~30 degrees in radians
#define SAFETY_TIMEOUT_MS 5000      // Stop if robot falls for this long

// Rolling average buffers
static float angle_buffer[ROLLING_AVERAGE_SIZE] = {0};
static float angular_velocity_buffer[ROLLING_AVERAGE_SIZE] = {0};
static int buffer_index = 0;

// Pre-trained model instance from the included file
static TwoWheelBotDQN bot;

// Safety monitoring
static int consecutive_falls = 0;
static const int MAX_CONSECUTIVE_FALLS = 10;

/**
 * Calculate rolling average
 */
static float rolling_average(float *buffer, int size, float new_value) {
    buffer[buffer_index % size] = new_value;
    
    float sum = 0;
    for (int i = 0; i < size; i++) {
        sum += buffer[i];
    }
    return sum / size;
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
            float raw_angle = sensor_value_to_double(&accel[1]) / 9.81f;
            
            // Clamp to valid range for asin
            if (raw_angle > 1.0f) raw_angle = 1.0f;
            if (raw_angle < -1.0f) raw_angle = -1.0f;
            
            // Calculate angle and apply offset
            float calculated_angle = asinf(raw_angle) - ANGLE_OFFSET;
            
            // Apply rolling average for smoothing
            angle = rolling_average(angle_buffer, ROLLING_AVERAGE_SIZE, calculated_angle);
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
            
            // Apply rolling average for smoothing
            angular_velocity = rolling_average(angular_velocity_buffer, 
                                              ROLLING_AVERAGE_SIZE, raw_velocity);
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
 * Apply motor control based on DQN model output
 */
static void apply_control(float angle, float angular_velocity) {
    // Get action from trained model (using the exported model's interface)
    int action = bot.getAction(angle, angular_velocity);
    float torque = bot.getMotorTorque(action);
    
    // Convert torque to motor speeds
    // For a two-wheel robot, both wheels move together for balance
    int16_t motor_speed = -(int16_t)(torque * MAX_MOTOR_SPEED);
    
    // Apply differential drive for balance
    // Positive torque = forward, negative = backward
    set_motor_speed(motor_speed, motor_speed);
    
    // Debug output (optional)
    #ifdef DEBUG_INFERENCE
    LOG_INF("State: [%.3f, %.3f] Action: %d Torque: %.2f",
            angle, angular_velocity, action, torque);
    #endif
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
static void calibrate_sensors(void) {
    LOG_INF("Calibrating sensors... Keep robot upright and still.");
    
    // Clear buffers
    for (int i = 0; i < ROLLING_AVERAGE_SIZE; i++) {
        angle_buffer[i] = 0;
        angular_velocity_buffer[i] = 0;
    }
    
    // Take initial readings to populate buffers
    float angle, angular_velocity;
    for (int i = 0; i < ROLLING_AVERAGE_SIZE * 2; i++) {
        get_angle(mpu6050, angle);
        get_angular_velocity(mpu6050, angular_velocity);
        k_sleep(K_MSEC(20));
    }
    
    LOG_INF("Calibration complete. Starting control loop.");
}

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
                test_states[i].angle,
                test_states[i].angular_velocity,
                action,
                torque,
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
            (float)missed_deadlines * 100.0f / loop_count);
    LOG_INF("Effective rate: %.1f Hz", 
            (float)loop_count / 10.0f);
    
    LOG_INF("\nSimulation mode complete. System halted.");
    
    // Blink LED to indicate simulation mode
    while (true) {
        k_sleep(K_MSEC(500));
    }
}

int main(void) {
    LOG_INF("Two-Wheel Balancing Robot - Inference Mode");
    LOG_INF("Using pre-trained DQN model");
    
    bool hardware_available = true;
    bool simulation_mode = false;
    
    // Check MPU6050 availability
    if (!device_is_ready(mpu6050)) {
        LOG_WRN("MPU6050 device not ready - will use simulation mode");
        hardware_available = false;
        simulation_mode = true;
    } else {
        LOG_INF("MPU6050 ready");
        
        // Try to read from MPU6050 to verify it's working
        struct sensor_value test_val;
        int rc = sensor_sample_fetch(mpu6050);
        if (rc != 0) {
            LOG_WRN("MPU6050 fetch failed (%d) - will use simulation mode", rc);
            hardware_available = false;
            simulation_mode = true;
        }
    }
    
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
    
    // Calibrate sensors
    calibrate_sensors();
    
    // Main control loop
    LOG_INF("Starting balance control...");
    
    float angle, angular_velocity;
    uint32_t loop_start_time;
    uint32_t total_iterations = 0;
    uint32_t successful_iterations = 0;
    
    while (true) {
        loop_start_time = k_uptime_get_32();
        
        // Read sensors
        int rc = get_angle(mpu6050, angle);
        if (rc != 0) {
            LOG_ERR("Failed to read angle: %d", rc);
            continue;
        }
        
        rc = get_angular_velocity(mpu6050, angular_velocity);
        if (rc != 0) {
            LOG_ERR("Failed to read angular velocity: %d", rc);
            continue;
        }
        
        // Safety check
        if (!safety_check(angle)) {
            // Try to recover after a pause
            k_sleep(K_MSEC(1000));
            consecutive_falls = 0;
            LOG_INF("Attempting to restart balance control...");
            continue;
        }
        
        // Apply control using trained model
        apply_control(angle, angular_velocity);
        
        // Track statistics
        total_iterations++;
        if (!has_fallen(angle)) {
            successful_iterations++;
        }
        
        // Print status every second (50 iterations at 50Hz)
        if (total_iterations % 50 == 0) {
            float success_rate = (float)successful_iterations / total_iterations * 100;
            LOG_INF("Status: Angle=%.3f rad, Vel=%.3f rad/s, Success=%.1f%%",
                   angle, angular_velocity, success_rate);
        }
        
        // Maintain control loop timing
        uint32_t loop_time = k_uptime_get_32() - loop_start_time;
        if (loop_time < CONTROL_LOOP_PERIOD_MS) {
            k_sleep(K_MSEC(CONTROL_LOOP_PERIOD_MS - loop_time));
        }
    }
    
    return 0;
}