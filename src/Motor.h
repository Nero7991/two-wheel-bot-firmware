// Motor.h
#ifndef MOTOR_H
#define MOTOR_H

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/pwm.h>
#include <stdio.h>

// Motor control PWM channels - using PCA9685 I2C PWM controller
#if DT_NODE_EXISTS(DT_NODELABEL(s_pwm6)) && DT_NODE_EXISTS(DT_NODELABEL(s_pwm7)) && DT_NODE_EXISTS(DT_NODELABEL(s_pwm8)) && DT_NODE_EXISTS(DT_NODELABEL(s_pwm9))
const struct pwm_dt_spec  motor_pwm_left_forward = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(s_pwm6), 0);
const struct pwm_dt_spec  motor_pwm_left_reverse = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(s_pwm7), 0);
const struct pwm_dt_spec motor_pwm_right_forward = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(s_pwm9), 0);
const struct pwm_dt_spec motor_pwm_right_reverse = PWM_DT_SPEC_GET_BY_IDX(DT_NODELABEL(s_pwm8), 0);
#else
// Fallback PWM specs when PCA9685 PWM LEDs are not available
extern struct pwm_dt_spec motor_pwm_left_forward;
extern struct pwm_dt_spec motor_pwm_left_reverse;
extern struct pwm_dt_spec motor_pwm_right_forward;
extern struct pwm_dt_spec motor_pwm_right_reverse;
#endif

#define PWM_PERIOD PWM_MSEC(5)

int initialize_pwm_driver();

inline int32_t set_motor_speed(int16_t motor_left, int16_t motor_right){
	// Check if PWM controller is available
	if (!motor_pwm_left_forward.dev) {
		// Simulation mode - just log motor speeds occasionally
		static int counter = 0;
		if (counter++ % 50 == 0) {
			printk("Motor simulation - Left: %d, Right: %d\n", motor_left, motor_right);
		}
		return 0;
	}

	// Set the motor speeds using PCA9685 PWM controller
	int ret = 0;
	
	if(motor_left >= 0){
		//Forward direction
		ret = pwm_set_dt(&motor_pwm_left_forward, PWM_PERIOD, PWM_USEC(motor_left * 5));
		if (ret) {
			goto error;
		}
		ret = pwm_set_dt(&motor_pwm_left_reverse, PWM_PERIOD, PWM_USEC(0));
		if (ret) {
			goto error;
		}
	}
	else{
		//Reverse direction
		ret = pwm_set_dt(&motor_pwm_left_reverse, PWM_PERIOD, PWM_USEC(-motor_left * 5));
		if (ret) {
			goto error;
		}
		ret = pwm_set_dt(&motor_pwm_left_forward, PWM_PERIOD, PWM_USEC(0));
		if (ret) {
			goto error;
		}
	}

	if(motor_right >= 0){
		//Forward direction
		ret = pwm_set_dt(&motor_pwm_right_forward, PWM_PERIOD, PWM_USEC(motor_right * 5));
		if (ret) {
			goto error;
		}
		ret = pwm_set_dt(&motor_pwm_right_reverse, PWM_PERIOD, PWM_USEC(0));
		if (ret) {
            goto error;
		}
	}
	else{
		//Reverse direction
		ret = pwm_set_dt(&motor_pwm_right_reverse, PWM_PERIOD, PWM_USEC(-motor_right * 5));
		if (ret) {
			goto error;
		}
		ret = pwm_set_dt(&motor_pwm_right_forward, PWM_PERIOD, PWM_USEC(0));
		if (ret) {
			goto error;
		}
	}
	return 0;
error:
    printk("Error %d: failed to set pulse width\n", ret);
    return -1;
}



#endif