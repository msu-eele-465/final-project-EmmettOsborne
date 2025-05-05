#include "esp_common.h"
#include "i2c_master.h"
#include "gpio.h"
#include "pca9685_interface.h"
#include <stdarg.h>
#include <stdio.h>

// Define missing types for RTOS SDK compatibility
#ifndef uint8_t
#define uint8_t unsigned char
#endif
#ifndef uint16_t
#define uint16_t unsigned short
#endif
#ifndef uint32_t
#define uint32_t unsigned int
#endif
#ifndef bool
#define bool uint8_t
#define true 1
#define false 0
#endif

// OE GPIO Pin (adjust as needed)
#define OE_GPIO_PIN 16

uint8_t pca9685_interface_iic_init(void) {
    i2c_master_gpio_init();
    i2c_master_init();
    return 0; // Success
}

uint8_t pca9685_interface_iic_deinit(void) {
    // No deinit function in i2c_master.h; assume cleanup if needed
    return 0; // Success
}

uint8_t pca9685_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    i2c_master_start();
    
    // Write address with write bit (LSB = 0)
    i2c_master_writeByte((addr << 1) | 0);
    if (!i2c_master_checkAck()) {
        i2c_master_stop();
        return 1; // Error
    }
    
    // Write register
    i2c_master_writeByte(reg);
    if (!i2c_master_checkAck()) {
        i2c_master_stop();
        return 1; // Error
    }
    
    // Write data
    for (uint16_t i = 0; i < len; i++) {
        i2c_master_writeByte(buf[i]);
        if (!i2c_master_checkAck()) {
            i2c_master_stop();
            return 1; // Error
        }
    }
    
    i2c_master_stop();
    return 0; // Success
}

uint8_t pca9685_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len) {
    // Write register address first
    i2c_master_start();
    i2c_master_writeByte((addr << 1) | 0);
    if (!i2c_master_checkAck()) {
        i2c_master_stop();
        return 1; // Error
    }
    
    i2c_master_writeByte(reg);
    if (!i2c_master_checkAck()) {
        i2c_master_stop();
        return 1; // Error
    }
    
    // Restart for reading
    i2c_master_start();
    i2c_master_writeByte((addr << 1) | 1);
    if (!i2c_master_checkAck()) {
        i2c_master_stop();
        return 1; // Error
    }
    
    // Read data
    for (uint16_t i = 0; i < len; i++) {
        buf[i] = i2c_master_readByte();
        if (i < len - 1) {
            i2c_master_send_ack();
        } else {
            i2c_master_send_nack();
        }
    }
    
    i2c_master_stop();
    return 0; // Success
}

uint8_t pca9685_interface_oe_gpio_init(void) {
    // Initialize OE pin (e.g., GPIO 16)
    gpio_output_conf(0, 0, 1 << OE_GPIO_PIN, 0);
    return 0;
}

uint8_t pca9685_interface_oe_gpio_deinit(void) {
    // No specific deinit needed
    return 0;
}

uint8_t pca9685_interface_oe_gpio_write(uint8_t value) {
    // Write to OE pin
    if (value) {
        gpio_output_set(1 << OE_GPIO_PIN, 0, 0, 0);
    } else {
        gpio_output_set(0, 1 << OE_GPIO_PIN, 0, 0);
    }
    return 0;
}

void pca9685_interface_delay_ms(uint32_t ms) {
    os_delay_us(ms * 1000);
}

void pca9685_interface_debug_print(const char *const fmt, ...) {
    va_list args;
    va_start(args, fmt);
    vprintf(fmt, args);
    va_end(args);
}