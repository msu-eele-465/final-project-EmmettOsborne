// Forward declaration of printAngles - yea I know this isn't the best way to do it
void printAngles(void);

/*************************************************************
 * 1) INCLUSIONS
 *************************************************************/

#include "esp_common.h"
#include "freertos/task.h"
#include "gpio.h"
#include "../lib/pca9685/driver_pca9685.h"
#include "../lib/pca9685/pca9685_interface.h"
#include "../lib/nrf24l01/nrf24l01.h"
#include <stdio.h>
#include "espressif/esp8266/uart_register.h"

/*************************************************************
 * 2) SERVO DRIVER DEFINITIONS
 *************************************************************/

// TCA9548A address
#define TCA9548A_ADDR 0x70

// Leg TCA channels
uint8_t tcaChannelForLeg[3] = {5, 6, 7}; // Legs 0,1,2

// Servo definition struct
typedef struct {
    int servoID;
    int leg;
    int joint;
    int pcaChannel;
} ServoDef_t;

/*************************************************************
 * 3) REMOTE-CONTROL STATE
 *************************************************************/

// Received data struct
typedef struct __attribute__((packed)) {
    uint8_t ch1;   // Joystick Left Y
    uint8_t ch2;   // Joystick Left X
    uint8_t ch3;   // Joystick Right Y
    uint8_t ch4;   // Joystick Right X
    uint8_t ch5;   // Left Joystick push
    uint8_t ch6;   // Right Joystick push
    uint8_t ch7;   // Connection toggling
    uint8_t ch8;   // Left rocker top
    uint8_t ch9;   // Right rocker top
    uint8_t ch10;  // Left rocker bottom
    uint8_t ch11;  // Right rocker bottom
} Received_data_t;

// PCA9685 handles
pca9685_handle_t pwmHandles[3];

// State variables
int currentMode = 1;
int currentJoint = 1;
int angleIndexForLegJoint[3][3] = {
    {4,4,4},
    {4,4,4},
    {4,4,4}
};
int last_ch5 = 0, last_ch6 = 0, last_ch8 = 0, last_ch9 = 0, last_ch10 = 0, last_ch11 = 0;

/*************************************************************
 * 4) RECEIVE DATA FUNCTIONS
 *************************************************************/

void processReceivedData(Received_data_t* data) {
    int ch1_value = map(data->ch1, 0, 255, 0, 255);
    int ch2_value = map(data->ch2, 0, 255, 0, 255);
    int ch3_value = map(data->ch3, 0, 255, 0, 255);
    int ch4_value = map(data->ch4, 0, 255, 0, 255);
    int ch5_value = map(data->ch5, 0, 1, 0, 1);
    int ch6_value = map(data->ch6, 0, 1, 0, 1);
    int ch7_value = map(data->ch7, 0, 1, 0, 1);
    int ch8_value = map(data->ch8, 0, 1, 0, 1);
    int ch9_value = map(data->ch9, 0, 1, 0, 1);
    int ch10_value = map(data->ch10, 0, 1, 0, 1);
    int ch11_value = map(data->ch11, 0, 1, 0, 1);

    static int prevCH7 = 0;
    static int toggleCount = 0;
    if (data->ch7 != prevCH7) {
        toggleCount = 0;
    } else {
        toggleCount++;
    }
    prevCH7 = data->ch7;
    if (toggleCount > 10) {
        printf("NOT CONNECTED\n");
    } else {
        printf("CONNECTED\n");
    }

    checkButtonsAndUpdateState(ch5_value, ch6_value, ch8_value, ch9_value, ch10_value, ch11_value, ch1_value, ch3_value);
}

void task_nrf24_receive(void* ignore) {
    printf("Setting up nRF24L01 as receiver...\n");
    nrf24_device(RECEIVER, RESET);
    printf("nRF24L01 setup complete.\n");
    nrf24_prx_static_payload_width(11, 1);
    // Declare datapipe_address here
    uint8_t datapipe_address[MAXIMUM_NUMBER_OF_DATAPIPES][ADDRESS_WIDTH_DEFAULT];
    uint8_t pipeAddr[5] = {0x00, 0x00, 0x00, 0x00, 0x01};
    memcpy(datapipe_address[0], pipeAddr, 5);
    nrf24_datapipe_address_configuration();
    nrf24_datapipe_enable(1);
    nrf24_rf_datarate(DATARATE_250KBPS);
    nrf24_rf_power(0);

    for (int i = 0; i < 3; i++) {
        DRIVER_PCA9685_LINK_INIT(&pwmHandles[i], pca9685_handle_t);
        DRIVER_PCA9685_LINK_IIC_INIT(&pwmHandles[i], pca9685_interface_iic_init);
        DRIVER_PCA9685_LINK_IIC_DEINIT(&pwmHandles[i], pca9685_interface_iic_deinit);
        DRIVER_PCA9685_LINK_IIC_READ(&pwmHandles[i], pca9685_interface_iic_read);
        DRIVER_PCA9685_LINK_IIC_WRITE(&pwmHandles[i], pca9685_interface_iic_write);
        DRIVER_PCA9685_LINK_OE_GPIO_INIT(&pwmHandles[i], pca9685_interface_oe_gpio_init);
        DRIVER_PCA9685_LINK_OE_GPIO_DEINIT(&pwmHandles[i], pca9685_interface_oe_gpio_deinit);
        DRIVER_PCA9685_LINK_OE_GPIO_WRITE(&pwmHandles[i], pca9685_interface_oe_gpio_write);
        DRIVER_PCA9685_LINK_DELAY_MS(&pwmHandles[i], pca9685_interface_delay_ms);
        DRIVER_PCA9685_LINK_DEBUG_PRINT(&pwmHandles[i], pca9685_interface_debug_print);
        pca9685_set_addr(&pwmHandles[i], 0x40);
        selectI2CBus(tcaChannelForLeg[i]);
        if (pca9685_init(&pwmHandles[i]) != 0) {
            printf("Failed to init PCA9685 for leg %d\n", i + 1);
        }
        uint8_t prescaler;
        uint32_t oscillator = PCA9685_OSCILLATOR_INTERNAL_FREQUENCY;
        uint16_t output_freq = 60;
        if (pca9685_output_frequency_convert_to_register(&pwmHandles[i], oscillator, output_freq, &prescaler) == 0) {
            pca9685_set_prescaler(&pwmHandles[i], prescaler);
        }
    }

    Received_data_t received_data;
    while (true) {
        uint8_t result = nrf24_receive((uint8_t*)&received_data, 11);
        if (result == OPERATION_DONE) {
            processReceivedData(&received_data);
        } else if (result == OPERATION_ERROR) {
            printf("Error in receiving\n");
        }
        vTaskDelay(10 / portTICK_RATE_MS);
    }
    vTaskDelete(NULL);
}

uint32 user_rf_cal_sector_set(void) {
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

/*************************************************************
 * 5) INITIALIZE
 *************************************************************/

void user_init(void) {
    WRITE_PERI_REG(UART_CLKDIV(0), UART_CLK_FREQ / 115200);
    printf("Start of program\n");
    xTaskCreate(&task_nrf24_receive, "nrf24_rx", 2048, NULL, 1, NULL);
}