#include "esp_common.h"
#include "freertos/task.h"
#include "gpio.h"
#include "../lib/pca9685/driver_pca9685.h"
#include "../lib/pca9685/pca9685_interface.h"
#include <stdio.h>
#include "espressif/esp8266/uart_register.h"

/******************************************************************************
 * FunctionName : user_rf_cal_sector_set
 * Description  : SDK just reversed 4 sectors, used for rf init data and paramters.
 *                We add this function to force users to set rf cal sector, since
 *                we don't know which sector is free in user's application.
 *                sector map for last several sectors : ABCCC
 *                A : rf cal
 *                B : rf init data
 *                C : sdk parameters
 * Parameters   : none
 * Returns      : rf cal sector
*******************************************************************************/
uint32 user_rf_cal_sector_set(void)
{
    flash_size_map size_map = system_get_flash_size_map();
    uint32 rf_cal_sec = 0;
    switch (size_map) {
        case FLASH_SIZE_4M_MAP_256_256:
            rf_cal_sec = 128 - 5;
            break;

        case FLASH_SIZE_8M_MAP_512_512:
            rf_cal_sec = 256 - 5;
            break;

        case FLASH_SIZE_16M_MAP_512_512:
        case FLASH_SIZE_16M_MAP_1024_1024:
            rf_cal_sec = 512 - 5;
            break;

        case FLASH_SIZE_32M_MAP_512_512:
        case FLASH_SIZE_32M_MAP_1024_1024:
            rf_cal_sec = 1024 - 5;
            break;

        default:
            rf_cal_sec = 0;
            break;
    }

    return rf_cal_sec;
}

void task_blink(void* ignore)
{
    gpio16_output_conf();
    while(true) {
    	gpio16_output_set(0);
        vTaskDelay(1000/portTICK_RATE_MS);
        printf("Blink\n");
    	gpio16_output_set(1);
        vTaskDelay(1000/portTICK_RATE_MS);
    }

    vTaskDelete(NULL);
}

void task_servo(void* ignore)
{
    pca9685_handle_t handle;
    DRIVER_PCA9685_LINK_INIT(&handle, pca9685_handle_t);
    DRIVER_PCA9685_LINK_IIC_INIT(&handle, pca9685_interface_iic_init);
    DRIVER_PCA9685_LINK_IIC_DEINIT(&handle, pca9685_interface_iic_deinit);
    DRIVER_PCA9685_LINK_IIC_READ(&handle, pca9685_interface_iic_read);
    DRIVER_PCA9685_LINK_IIC_WRITE(&handle, pca9685_interface_iic_write);
    DRIVER_PCA9685_LINK_OE_GPIO_INIT(&handle, pca9685_interface_oe_gpio_init);
    DRIVER_PCA9685_LINK_OE_GPIO_DEINIT(&handle, pca9685_interface_oe_gpio_deinit);
    DRIVER_PCA9685_LINK_OE_GPIO_WRITE(&handle, pca9685_interface_oe_gpio_write);
    DRIVER_PCA9685_LINK_DELAY_MS(&handle, pca9685_interface_delay_ms);
    DRIVER_PCA9685_LINK_DEBUG_PRINT(&handle, pca9685_interface_debug_print);

    // Set the address to 0x40 (default address for PCA9685 :D )
    pca9685_set_addr(&handle, 0x40);

    // Initialize the PCA9685
    if (pca9685_init(&handle) != 0) {
        printf("PCA9685 initialization failed\n");
        vTaskDelete(NULL);
    }

    // Calculate prescaler for 50Hz
    uint8_t prescaler;
    uint32_t oscillator = PCA9685_OSCILLATOR_INTERNAL_FREQUENCY; // 25MHz
    uint16_t output_freq = 50;
    if (pca9685_output_frequency_convert_to_register(&handle, oscillator, output_freq, &prescaler) != 0) {
        printf("Failed to convert frequency to prescaler\n");
        pca9685_deinit(&handle);
        vTaskDelete(NULL);
    }

    // Set the prescaler
    if (pca9685_set_prescaler(&handle, prescaler) != 0) {
        printf("Failed to set prescaler\n");
        pca9685_deinit(&handle);
        vTaskDelete(NULL);
    }

    // Set PWM for channel 0 to 1.5ms pulse (approximately 90 degrees)
    uint16_t on_count = 0;
    uint16_t off_count = 307; // approximately 1.5ms at 50Hz with 4096 steps
    if (pca9685_write_channel(&handle, PCA9685_CHANNEL_0, on_count, off_count) != 0) {
        printf("Failed to write PWM channel\n");
        pca9685_deinit(&handle);
        vTaskDelete(NULL);
    }

    printf("Servo set to 90 degrees\n");

    // Keep the task running or perform other operations as needed
    while (true) {
        vTaskDelay(1000 / portTICK_RATE_MS);
    }

    // Deinitialize if necessary
    pca9685_deinit(&handle);
    vTaskDelete(NULL);
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
    // Set UART0 baud rate to 115200
    WRITE_PERI_REG(UART_CLKDIV(0), UART_CLK_FREQ / 115200);
    
    // Test print to verify baud rate
    printf("Start of program\n");
    
    xTaskCreate(&task_blink, "startup", 2048, NULL, 1, NULL);
    xTaskCreate(&task_servo, "servo", 2048, NULL, 1, NULL);
}