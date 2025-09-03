
#include "Motor.h"

#if !DT_NODE_EXISTS(DT_NODELABEL(s_pwm6)) || !DT_NODE_EXISTS(DT_NODELABEL(s_pwm7)) || !DT_NODE_EXISTS(DT_NODELABEL(s_pwm8)) || !DT_NODE_EXISTS(DT_NODELABEL(s_pwm9))
// Fallback PWM specs when PCA9685 PWM LEDs are not available
struct pwm_dt_spec motor_pwm_left_forward = {.dev = NULL, .channel = 6, .period = PWM_MSEC(5), .flags = 0};
struct pwm_dt_spec motor_pwm_left_reverse = {.dev = NULL, .channel = 7, .period = PWM_MSEC(5), .flags = 0};
struct pwm_dt_spec motor_pwm_right_forward = {.dev = NULL, .channel = 9, .period = PWM_MSEC(5), .flags = 0};
struct pwm_dt_spec motor_pwm_right_reverse = {.dev = NULL, .channel = 8, .period = PWM_MSEC(5), .flags = 0};
#endif

int initialize_pwm_driver(){
#if DT_NODE_EXISTS(DT_NODELABEL(pca9685))
    // Initialize PCA9685 PWM controller on I2C bus
    const struct device *const pca9685 = DEVICE_DT_GET(DT_NODELABEL(pca9685));

    if (!device_is_ready(pca9685)) {
        printk("PCA9685 device %s is not ready. Waiting...\n", pca9685->name);
        int timeout = 100; // 10 second timeout
        while(!device_is_ready(pca9685) && --timeout > 0) {
            k_sleep(K_MSEC(100));
        }
        if (timeout <= 0) {
            printk("PCA9685 device timeout - motor control will not work\n");
            return -1;
        }
    }
    printk("PCA9685 %s ready\n", pca9685->name);

    // PWM channels are now configured via device tree - no manual assignment needed

    // Check if PWM channels are ready
    if (motor_pwm_left_forward.dev && !pwm_is_ready_dt(&motor_pwm_left_forward)) {
        printk("Error: PWM device %s is not ready\n",
               motor_pwm_left_forward.dev->name);
        return -1;
    }

    printk("All motor PWM channels ready\n");
    return 0;
#else
    printk("PCA9685 PWM controller not configured in device tree\n");
    printk("Motor control will be in simulation mode\n");
    return -1;
#endif
}

