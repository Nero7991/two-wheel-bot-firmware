
#include "Motor.h"

#define PCA9685 DT_NODELABEL(pca9685)

int initialize_pwm_driver(){
    const struct device *const pca9685 = DEVICE_DT_GET(PCA9685);

    if (!device_is_ready(pca9685)) {
		printf("Device %s is not ready. waiting...\n", pca9685->name);
		while(!device_is_ready(pca9685));
	}
	printf("%s ready\n", pca9685->name);

	if (!pwm_is_ready_dt(&motor_pwm_left_forward)) {
		printk("Error: PWM device %s is not ready\n",
		       motor_pwm_left_forward.dev->name);
		return -1;
	}

	if (!pwm_is_ready_dt(&motor_pwm_right_forward)) {
		printk("Error: PWM device %s is not ready\n",
		       motor_pwm_right_forward.dev->name);
		return -1;
	}
    return 0;
}

