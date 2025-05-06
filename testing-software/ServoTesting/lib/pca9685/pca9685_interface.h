#ifndef PCA9685_INTERFACE_H
#define PCA9685_INTERFACE_H

#include <stdint.h>

uint8_t pca9685_interface_iic_init(void);
uint8_t pca9685_interface_iic_deinit(void);
uint8_t pca9685_interface_iic_read(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t pca9685_interface_iic_write(uint8_t addr, uint8_t reg, uint8_t *buf, uint16_t len);
uint8_t pca9685_interface_oe_gpio_init(void);
uint8_t pca9685_interface_oe_gpio_deinit(void);
uint8_t pca9685_interface_oe_gpio_write(uint8_t value);
void pca9685_interface_delay_ms(uint32_t ms);
void pca9685_interface_debug_print(const char *const fmt, ...);

#endif