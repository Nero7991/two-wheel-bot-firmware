#ifndef I2C_MANUAL_INIT_H
#define I2C_MANUAL_INIT_H

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

#ifdef CONFIG_BOARD_ESP32S3_DEVKITC

// ESP32-S3 I2C pins
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9

static inline int manual_i2c_init(void)
{
    int ret;
    
    printk("Attempting manual I2C initialization on GPIO%d (SDA) and GPIO%d (SCL)\n", 
            I2C_SDA_PIN, I2C_SCL_PIN);
    
    // Get I2C device - try multiple approaches
    const struct device *i2c_dev = NULL;
    
    // Try device tree label first
    i2c_dev = device_get_binding("I2C_0");
    if (!i2c_dev) {
        printk("I2C_0 device not found, trying alternative names\n");
        i2c_dev = device_get_binding("i2c0");
    }
    if (!i2c_dev) {
        printk("i2c0 device not found, trying register address\n");
        i2c_dev = device_get_binding("i2c@60013000");
    }
    if (!i2c_dev) {
        printk("i2c@60013000 device not found, trying DEVICE_DT_GET\n");
        // Try direct device tree access
        #if DT_NODE_HAS_STATUS(DT_NODELABEL(i2c0), okay)
        i2c_dev = DEVICE_DT_GET(DT_NODELABEL(i2c0));
        printk("Got device via DT_NODELABEL(i2c0)\n");
        #else
        printk("i2c0 node not okay in device tree\n");
        #endif
    }
    if (!i2c_dev) {
        printk("ERROR: No I2C device found through any method\n");
        return -ENODEV;
    }
    
    // Check if device is ready
    if (!device_is_ready(i2c_dev)) {
        printk("I2C device not ready, attempting to initialize\n");
        
        // Get GPIO device for pin configuration
        const struct device *gpio_dev = device_get_binding("GPIO_0");
        if (!gpio_dev) {
            printk("ERROR: GPIO device not found\n");
            return -ENODEV;
        }
        
        // Configure SDA pin
        ret = gpio_pin_configure(gpio_dev, I2C_SDA_PIN, 
                                 GPIO_PULL_UP | GPIO_OPEN_DRAIN | GPIO_OUTPUT_HIGH);
        if (ret < 0) {
            printk("ERROR: Failed to configure SDA pin: %d\n", ret);
            return ret;
        }
        
        // Configure SCL pin  
        ret = gpio_pin_configure(gpio_dev, I2C_SCL_PIN,
                                 GPIO_PULL_UP | GPIO_OPEN_DRAIN | GPIO_OUTPUT_HIGH);
        if (ret < 0) {
            printk("ERROR: Failed to configure SCL pin: %d\n", ret);
            return ret;
        }
        
        printk("GPIO pins configured for I2C\n");
    }
    
    // Configure I2C speed (100kHz standard)
    uint32_t i2c_cfg = I2C_SPEED_SET(I2C_SPEED_STANDARD) | I2C_MODE_CONTROLLER;
    ret = i2c_configure(i2c_dev, i2c_cfg);
    if (ret < 0) {
        printk("ERROR: Failed to configure I2C: %d\n", ret);
        return ret;
    }
    
    printk("I2C manually initialized successfully\n");
    
    // Test I2C by scanning for devices
    printk("Scanning I2C bus for devices...\n");
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        struct i2c_msg msgs[1];
        uint8_t dst;
        
        msgs[0].buf = &dst;
        msgs[0].len = 0;
        msgs[0].flags = I2C_MSG_WRITE | I2C_MSG_STOP;
        
        if (i2c_transfer(i2c_dev, &msgs[0], 1, addr) == 0) {
            printk("I2C device found at address 0x%02x\n", addr);
        }
    }
    
    return 0;
}

#else

static inline int manual_i2c_init(void)
{
    // Not ESP32, no manual init needed
    return 0;
}

#endif

#endif // I2C_MANUAL_INIT_H