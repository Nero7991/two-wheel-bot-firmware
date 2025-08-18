
/* MIT License
Copyright (c) 2024 Oren Collaco

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

 */

/*
 * Copyright (c) 2021 Nordic Semiconductor ASA
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/devicetree.h>
#include <app_version.h>
#include "stdlib.h"
#include "DQNAgent.h"
#include "Motor.h"
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/pwm.h>

#include <zephyr/logging/log.h>
#include <stdio.h>

LOG_MODULE_REGISTER(main, CONFIG_APP_LOG_LEVEL);

const struct device *const mpu6050 = DEVICE_DT_GET_ONE(invensense_mpu6050);

static const char *now_str(void)
{
	static char buf[16]; /* ...HH:MM:SS.MMM */
	uint32_t now = k_uptime_get_32();
	unsigned int ms = now % MSEC_PER_SEC;
	unsigned int s;
	unsigned int min;
	unsigned int h;

	now /= MSEC_PER_SEC;
	s = now % 60U;
	now /= 60U;
	min = now % 60U;
	now /= 60U;
	h = now;

	snprintf(buf, sizeof(buf), "%u:%02u:%02u.%03u",
		 h, min, s, ms);
	return buf;
}

static int process_mpu6050(const struct device *dev)
{
	struct sensor_value temperature;
	struct sensor_value accel[3];
	struct sensor_value gyro[3];
	int rc = sensor_sample_fetch(dev);

	if(rc == -EBADMSG) {
		printf("Sample fetch error: %d\n", rc);
		return rc;
	}

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
					accel);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
					gyro);
	}
	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP,
					&temperature);
	}
	if (rc == 0) {
		printf("[%s]:%g Cel: "
		       "  accel %f %f %f m/s/s"
		       "  gyro  %f %f %f rad/s\n",
		       now_str(),
		       sensor_value_to_double(&temperature),
		       sensor_value_to_double(&accel[0]),
		       sensor_value_to_double(&accel[1]),
		       sensor_value_to_double(&accel[2]),
		       sensor_value_to_double(&gyro[0]),
		       sensor_value_to_double(&gyro[1]),
		       sensor_value_to_double(&gyro[2]));
	} else {
		printf("sample fetch/get failed: %d\n", rc);
	}

	return rc;
}

/* rolling average function that takes buffer array reference */
float rolling_average(float *buffer, int buffer_size, float new_value) {
	float sum = 0;
	for (int i = 0; i < buffer_size - 1; i++) {
		buffer[i] = buffer[i + 1];
		sum += buffer[i];
	}
	buffer[buffer_size - 1] = new_value;
	sum += new_value;
	return sum / buffer_size;
}

// Function to find the index of the maximum element in an array
int argmax(const float* array, int size) {
    int max_index = 0;
    for (int i = 1; i < size; ++i) {
        if (array[i] > array[max_index]) {
            max_index = i;
        }
    }
    return max_index;
}

// Function to find the maximum element in an array
float max_element(const float* array, int size) {
    float max_value = array[0];
    for (int i = 1; i < size; ++i) {
        if (array[i] > max_value) {
            max_value = array[i];
        }
    }
    return max_value;
}

#define ROLLING_AVERAGE_SIZE 8

int get_angle(const struct device *dev, float& angle){
	// Get the angle from the MPU6050
	struct sensor_value accel[3];
	static float angle_hist[ROLLING_AVERAGE_SIZE];
	int rc = sensor_sample_fetch(dev);

	if(rc == -EBADMSG) {
		printf("MP6050 fetch error: %d\n", rc);
		return rc;
	}

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ,
					accel);
		angle = rolling_average(angle_hist, ROLLING_AVERAGE_SIZE, sensor_value_to_double(&accel[1])/12.0);	
	}
	return 0;

}

int get_angular_velocity(const struct device *dev, float& angle_velocity){
	// Get the angular velocity from the MPU6050
	struct sensor_value gyro[3];
    static float angle_velocity_hist[ROLLING_AVERAGE_SIZE];
	int rc = sensor_sample_fetch(dev);

	if(rc == -EBADMSG) {
		printf("MP6050 fetch error: %d\n", rc);
		return rc;
	}

	if (rc == 0) {
		rc = sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ,
					gyro);
		angle_velocity = rolling_average(angle_velocity_hist, ROLLING_AVERAGE_SIZE, sensor_value_to_double(&gyro[0])/6.0);	
	}
	return 0;
}

void get_next_state(float action[NUM_OUTPUTS], float next_state[NUM_INPUTS]) {
    // Calculate the motor speeds based on the action
    float motor_left = action[0] * 1000;
    float motor_right = action[1] * 1000;

    // Apply the motor speeds to the robot and wait for a short duration
    set_motor_speed((int16_t)motor_left, (int16_t)motor_right);
    k_sleep(K_MSEC(20));

    // Get the updated tilt angle and angular velocity after applying the action
    float updated_tilt_angle, updated_angular_velocity;
    get_angle(mpu6050, updated_tilt_angle);
    get_angular_velocity(mpu6050, updated_angular_velocity);

    // Store the updated state in the next_state array
    next_state[0] = updated_tilt_angle;
    next_state[1] = updated_angular_velocity;
}

void get_state(float state[NUM_INPUTS]) {
    // Get the tilt angle and angular velocity after applying the action
    float updated_tilt_angle, updated_angular_velocity;
    get_angle(mpu6050, updated_tilt_angle);
    get_angular_velocity(mpu6050, updated_angular_velocity);

    // Store the updated state in the next_state array
    state[0] = updated_tilt_angle;
    state[1] = updated_angular_velocity;
}

#ifdef CONFIG_MPU6050_TRIGGER
static struct sensor_trigger trigger;

static void handle_mpu6050_drdy(const struct device *dev,
				const struct sensor_trigger *trig)
{
	int rc = process_mpu6050(dev);

	if (rc != 0) {
		printf("cancelling trigger due to failure: %d\n", rc);
		(void)sensor_trigger_set(dev, trig, NULL);
		return;
	}
}
#endif /* CONFIG_MPU6050_TRIGGER */

const int NUM_EPISODES = 10000000;
const float LEARNING_RATE = 0.00001;
const float DISCOUNT_FACTOR = 0.90;
const int MAX_STEPS = 2;
const int MAX_TRANSITIONS = MAX_STEPS;
const float MIN_TILT_ANGLE = -30.0;
const float MAX_TILT_ANGLE = 30.0;

const float BETA1 = 0.9;
const float BETA2 = 0.999;
const float EPSILON = 0.01;

struct Transition {
	float state[NUM_INPUTS];
	float action[NUM_OUTPUTS];
	float reward;
	float next_state[NUM_INPUTS];
};

Transition transitions[MAX_TRANSITIONS];
DQNAgent agent(LEARNING_RATE, BETA1, BETA2, EPSILON);

int main(void)
{
	int ret;

	printf("Zephyr Adeept 2Wheel %s\n", APP_VERSION_STRING);

	agent.initialize(LEARNING_RATE, BETA1, BETA2, EPSILON);
	agent.print_stats();

	

	if (!device_is_ready(mpu6050)) {
		printf("Device %s is not ready. waiting...\n", mpu6050->name);
		while(!device_is_ready(mpu6050));
	}
	printf(" %s ready\n", mpu6050->name);

	initialize_pwm_driver();
	
	for (int i = 0; i < 20; i++) {
		int rc = 0;

		/* Print angle and angular velocity */
		float angle, angular_velocity;
		get_angle(mpu6050, angle);
		get_angular_velocity(mpu6050, angular_velocity);
		printf("Angle: %f, Angular Velocity: %f\n", angle, angular_velocity);

		set_motor_speed(0, 0);

		if (rc != 0) {
			break;
		}
		k_sleep(K_MSEC(20));
	}

	#ifdef CONFIG_MPU6050_TRIGGER
		trigger = (struct sensor_trigger) {
			.type = SENSOR_TRIG_DATA_READY,
			.chan = SENSOR_CHAN_ALL,
		};
		if (sensor_trigger_set(mpu6050, &trigger,
					handle_mpu6050_drdy) < 0) {
			printf("Cannot configure trigger\n");
			return 0;
		}
		printk("Configured for triggered sampling.\n");
	#endif
	
	int num_transitions = 0;

	///while(1);
	uint32_t start_time, end_time, train_time, inference_time;

	for (int episode = 0; episode < NUM_EPISODES; ++episode) {
        // Reset the environment (e.g., set the robot to a known initial state)
        // reset_robot();

        float tilt_angle, angular_velocity;
        float state[NUM_INPUTS];
        float action[NUM_OUTPUTS];
        float reward;
		bool is_episode_done = false;
		float error, old_error, error_change = 0.0;
		int count = 0;
        // Run the episode until a termination condition is met
        while (!is_episode_done) {
            // Get the current tilt angle and angular velocity from the MPU6050 sensor
            get_angle(mpu6050, tilt_angle);
            get_angular_velocity(mpu6050, angular_velocity);

            // Prepare the state input for the DQNAgent
            state[0] = tilt_angle;
            state[1] = angular_velocity;

            // Get the action from the DQNAgent
            agent.forward(state, action);

            // Scale the action outputs to motor speeds
            float motor_left = action[0] * 1000;
            float motor_right = action[1] * 1000;
        
            // Set the motor speeds and wait for the robot to move
            set_motor_speed((int16_t)motor_left, (int16_t)motor_right);
            k_sleep(K_MSEC(20));

            // Get the next state after taking the action
            float next_state[NUM_INPUTS];
            get_state(next_state);

			// Get the new action for training
			float next_action[NUM_OUTPUTS];
			agent.forward(next_state, next_action);

			float targets[NUM_OUTPUTS];

			// Calculate the reward based on the tilt angle and angular velocity
            reward = 0.0;
			/* calculate the difference in error after 3 iterations using count */
			// if (count == 0) {
			// 	old_error = next_state[0] - 0.115f;
			// 	if(old_error < 0)
			// 		old_error = -old_error;
			// }
			// count++;
			// if (count == 2) {
			// 	count = 0;
			// 	error = next_state[0] - 0.115;
			// 	if(error < 0)
			// 		error = -error;
			// 	error_change = old_error - error;
			// }

			// if(error_change < 0)
			// 	reward = -0.5;
			// else
			// 	reward = 0.2;
            if (abs(next_state[1]) < 0.1) {
				reward += 0.1; // Small positive reward for keeping angular velocity minimal
			} 
			else {
				reward -= 0.1; // Small negative reward for increasing angular velocity
			}
			error = next_state[0] - 0.115f;
			/* if error is +ve and moving forward = bad */
			if (error > 0 && next_action[0] > 0.0) {	
				reward -= 0.1;
				targets[0] = -0.4;
			/* if error is +ve and moving back = good */
			} else if (error > 0 && next_action[0] < 0.0) {
				reward += 0.1;
				targets[0] = next_action[0];
			/* if error is -ve and moving forward = good */
			} else if (error < 0 && next_action[0] > 0.0) {
				reward += 0.1;
				targets[0] = next_action[0];
			/* if error is -ve and moving back = bad */
			} else if (error < 0 && next_action[0] < 0.0) {
				reward -= 0.1;
				targets[0] = 0.4;
			}
			
            if (next_state[0] > 0.10 && next_state[0] < 0.13) {
                reward += 0.1;  // Positive reward for keeping the tilt angle in desired range
            } else if (next_state[0] < 0.10 || next_state[0] > 0.13) {
                reward -= 0.1;  // Negative reward for exceeding the tilt angle range
            }

			/* print input, motor values and reward */
			printf("Angle: %f, Angular Velocity: %f, Motor Left: %f, Motor Right: %f, Reward: %f, Error: %f\n", tilt_angle, angular_velocity, motor_left, motor_right, reward, error);
		
            // // Get the Q-values for the next state
            // float next_q_values[NUM_OUTPUTS];
            // agent.forward(next_state, next_q_values);

            // Calculate the target Q-value for the current state-action pair
            // float target_q_value = reward + DISCOUNT_FACTOR * *std::max_element(next_q_values, next_q_values + NUM_OUTPUTS);

            // Prepare the target Q-values for training
            //float targets[NUM_OUTPUTS];
            // for (int i = 0; i < NUM_OUTPUTS; ++i) {
            //     //targets[i] = reward + current_q_values[i] * DISCOUNT_FACTOR;
            //     targets[i] = current_q_values[i] * DISCOUNT_FACTOR;

            // }

            // Train the DQNAgent with the current state, targets, and reward
            agent.train(state, targets, reward);
        }

        printf("Episode %d completed\n", episode + 1);
    }

	return 0;
}

