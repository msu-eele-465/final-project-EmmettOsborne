// Forward declaration of printAngles
void printAngles(void);

/*************************************************************
 * 1) Integrated Remote + Servo Control
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
 * 3) SERVO MAP & ANGLE TABLE
 *************************************************************/

// Servo map
ServoDef_t servoMap[18] = {
    // Servos #01..12 (two-digit)
    {1, 1, 3, 2}, 
    {2, 1, 3, 3}, 
    {3, 1, 2, 4}, 
    {4, 1, 2, 5},
    {5, 2, 3, 2}, 
    {6, 2, 3, 3}, 
    {7, 2, 2, 4}, 
    {8, 2, 2, 5},
    {9, 3, 3, 2}, 
    {10, 3, 3, 3}, 
    {11, 3, 2, 4}, 
    {12, 3, 2, 5},

    // Servos #101..106 (three-digit)
    {101, 1, 1, 0}, 
    {102, 1, 1, 1}, 
    {103, 2, 1, 0}, 
    {104, 2, 1, 1},
    {105, 3, 1, 0}, 
    {106, 3, 1, 1}
};

// Servo angle table
// We have 18 servos, each row has 7 columns => [18][7]
// Index 0..6 => 45°,90°,135°,180°,225°,270°,315°
// -1 => invalid
int servoAngleTable[18][7] = {
    // servo #1 Leg 1 Knee Left (index=0)
    {133, 210, 287, 367, 448, 525, 607},
    // servo #2 Leg 1 Knee Right (1)
    {136, 214, 291, 369, 450, 529, 611},
    // servo #3 Leg 1 Legg Left  (2)
    {125, 205, 285, 366, 446, 524, 602},
    // servo #4 Leg 1 Legg Right (3)
    {128, 206, 282, 360, 439, 518, 601},
    // servo #5 Leg 2 Knee Left (4)
    {140, 220, 299, 377, 458, 535, 615},
    // servo #6 Leg 2 Knee Right (5)
    {136, 216, 293, 373, 454, 532, 614},
    // servo #7 Leg 2 Legg Left (6)
    {125, 206, 285, 363, 442, 516, 593},
    // servo #8 Leg 2 Legg Right (7)
    {128, 206, 282, 360, 439, 518, 601},
    // servo #9 Leg 3 Knee Left (8)
    {131, 211, 291, 370, 450, 526, 606},
    // servo #10 Leg 3 Knee Right (9)
    {125, 205, 285, 366, 446, 524, 602},
    // servo #11 Leg 3 Legg Left (10)
    {127, 207, 284, 362, 442, 518, 598},
    // servo #12 Leg 3 Legg Right (11)
    {133, 214, 294, 376, 456, 532, 610},

    // servo #101 Leg 1 Hip Down (12)
    {-1, -1, 265, 386, 507, -1, -1},
    // servo #102 Leg 1 Hip Up (13)
    {-1, -1, 247, 366, 483, -1, -1},
    // servo #103 Leg 2 Hip Down (14)
    {-1, -1, 251, 371, 490, -1, -1},
    // servo #104 Leg 2 Hip Up (15)
    {-1, -1, 255, 374, 493, -1, -1},
    // servo #105 Leg 3 Hip Down (16)
    {-1, -1, 236, 355, 475, -1, -1},
    // servo #106 Leg 3 Hip Up (17)
    {-1, -1, 242, 357, 480, -1, -1}
};

/*************************************************************
 * 4) REMOTE-CONTROL STATE
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
 * 5) SERVO CONTROL HELPERS
 *************************************************************/

// Helper functions
int map(int x, int in_min, int in_max, int out_min, int out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void selectI2CBus(uint8_t bus) {
    uint8_t data = 1 << bus;
    if (pca9685_interface_iic_write_no_reg(TCA9548A_ADDR, data) != 0) {
        printf("Failed to select I2C bus %d\n", bus);
    }
}

 /*************************************************************
 * 6) SET SERVO ANGLES
 *************************************************************/

void setServoAngles(int leg, int joint, int angleIndex) {
    int normalIndex = angleIndex - 1;
    int mirroredIndex = 6 - normalIndex;
    for (int i = 0; i < 18; i++) {
        if (servoMap[i].leg == leg && servoMap[i].joint == joint) {
            bool isEven = (servoMap[i].servoID % 2 == 0);
            int useIndex = isEven ? normalIndex : mirroredIndex;
            int pwmVal = servoAngleTable[i][useIndex];
            if (pwmVal != -1) {
                int legIndex = leg - 1;
                selectI2CBus(tcaChannelForLeg[legIndex]);
                if (pca9685_write_channel(&pwmHandles[legIndex], servoMap[i].pcaChannel, 0, pwmVal) != 0) {
                    printf("Failed to set PWM for servo %d\n", servoMap[i].servoID);
                }
            }
        }
    }
}

/*************************************************************
 * 7) CHANGE ANGLE LOGIC
 *************************************************************/

void changeAngle(int increment) {
    if (currentMode == 4) {
        for (int legIndex = 0; legIndex < 3; legIndex++) {
            int jIndex = currentJoint - 1;
            angleIndexForLegJoint[legIndex][jIndex] += increment;
            if (angleIndexForLegJoint[legIndex][jIndex] < 1) angleIndexForLegJoint[legIndex][jIndex] = 1;
            if (angleIndexForLegJoint[legIndex][jIndex] > 7) angleIndexForLegJoint[legIndex][jIndex] = 7;
            setServoAngles(legIndex + 1, jIndex + 1, angleIndexForLegJoint[legIndex][jIndex]);
        }
        printf("ANGLE %s for ALL LEGS, joint %d\n", (increment > 0) ? "++" : "--", currentJoint);
    } else {
        int l = currentMode - 1;
        int j = currentJoint - 1;
        angleIndexForLegJoint[l][j] += increment;
        if (angleIndexForLegJoint[l][j] < 1) angleIndexForLegJoint[l][j] = 1;
        if (angleIndexForLegJoint[l][j] > 7) angleIndexForLegJoint[l][j] = 7;
        setServoAngles(currentMode, currentJoint, angleIndexForLegJoint[l][j]);
        printf("ANGLE %s for leg %d, joint %d\n", (increment > 0) ? "++" : "--", currentMode, currentJoint);
    }
    printAngles();
}

/*************************************************************
 * 8) JOYSTICK CONTROL FOR MODE 5
 *************************************************************/

int interpolatePWM(int joystickValue, int servoIdx) {
    const int rangeBounds[] = {0, 36, 72, 108, 145, 181, 218, 255};
    const int angleIndices[] = {6, 5, 4, 3, 2, 1, 0};
    int rangeIdx;
    for (rangeIdx = 0; rangeIdx < 6; rangeIdx++) {
        if (joystickValue >= rangeBounds[rangeIdx] && joystickValue <= rangeBounds[rangeIdx + 1]) {
            break;
        }
    }
    if (rangeIdx >= 6) rangeIdx = 6;
    bool isEven = (servoMap[servoIdx].servoID % 2 == 0);
    int startIdx = angleIndices[rangeIdx];
    int endIdx = (rangeIdx < 6) ? angleIndices[rangeIdx + 1] : angleIndices[rangeIdx];
    if (!isEven) {
        startIdx = 6 - startIdx;
        endIdx = 6 - endIdx;
    }
    int pwmStart = servoAngleTable[servoIdx][startIdx];
    int pwmEnd = servoAngleTable[servoIdx][endIdx];
    if (pwmStart == -1 || pwmEnd == -1) {
        return (pwmStart != -1) ? pwmStart : (pwmEnd != -1) ? pwmEnd : 0;
    }
    float t = (float)(joystickValue - rangeBounds[rangeIdx]) / (rangeBounds[rangeIdx + 1] - rangeBounds[rangeIdx]);
    int pwmValue = pwmStart + t * (pwmEnd - pwmStart);
    return pwmValue;
}

void updateJoystickControl(int ch1, int ch3) {
    for (int i = 0; i < 18; i++) {
        if (servoMap[i].joint == 2) {
            int pwm = interpolatePWM(ch1, i);
            int legIndex = servoMap[i].leg - 1;
            selectI2CBus(tcaChannelForLeg[legIndex]);
            pca9685_write_channel(&pwmHandles[legIndex], servoMap[i].pcaChannel, 0, pwm);
        } else if (servoMap[i].joint == 3) {
            int pwm = interpolatePWM(ch3, i);
            int legIndex = servoMap[i].leg - 1;
            selectI2CBus(tcaChannelForLeg[legIndex]);
            pca9685_write_channel(&pwmHandles[legIndex], servoMap[i].pcaChannel, 0, pwm);
        }
    }
    for (int legIndex = 0; legIndex < 3; legIndex++) {
        angleIndexForLegJoint[legIndex][1] = map(ch1, 0, 255, 7, 1);
        angleIndexForLegJoint[legIndex][2] = map(ch3, 0, 255, 7, 1);
    }
    printAngles();
}

/*************************************************************
 * 9) PRINT ANGLE STATES
 *************************************************************/

void printAngles(void) {
    printf("Current angle states (1..7 => 45..315°):\n");
    for (int leg = 0; leg < 3; leg++) {
        printf(" Leg %d: ", leg + 1);
        for (int jt = 0; jt < 3; jt++) {
            printf("Joint%d=%d ", jt + 1, angleIndexForLegJoint[leg][jt]);
        }
        printf("\n");
    }
    printf("\n");
}

/*************************************************************
 * 10) REMOTE UI LOGIC
 *************************************************************/

void checkButtonsAndUpdateState(int ch5, int ch6, int ch8, int ch9, int ch10, int ch11, int ch1, int ch3) {
    if (ch5 == 1 && last_ch5 == 0) {
        currentMode--;
        if (currentMode < 1) currentMode = 5;
        printf("MODE changed => %d\n", currentMode);
        if (currentMode == 5) {
            for (int legIndex = 0; legIndex < 3; legIndex++) {
                angleIndexForLegJoint[legIndex][0] = 4;
                setServoAngles(legIndex + 1, 1, 4);
            }
            printf("Mode 5: Joint 1 set to 180° for all legs\n");
        }
    }
    if (ch6 == 1 && last_ch6 == 0) {
        currentMode++;
        if (currentMode > 5) currentMode = 1;
        printf("MODE changed => %d\n", currentMode);
        if (currentMode == 5) {
            for (int legIndex = 0; legIndex < 3; legIndex++) {
                angleIndexForLegJoint[legIndex][0] = 4;
                setServoAngles(legIndex + 1, 1, 4);
            }
            printf("Mode 5: Joint 1 set to 180° for all legs\n");
        }
    }
    if (currentMode != 5) {
        if (ch8 == 1 && last_ch8 == 0) {
            currentJoint++;
            if (currentJoint > 3) currentJoint = 3;
            printf("JOINT changed => %d\n", currentJoint);
        }
        if (ch10 == 1 && last_ch10 == 0) {
            currentJoint--;
            if (currentJoint < 1) currentJoint = 1;
            printf("JOINT changed => %d\n", currentJoint);
        }
    }
    if (currentMode != 5) {
        if (ch9 == 1 && last_ch9 == 0) {
            changeAngle(-1);
        }
        if (ch11 == 1 && last_ch11 == 0) {
            changeAngle(1);
        }
    } else {
        updateJoystickControl(ch1, ch3);
    }
    last_ch5 = ch5;
    last_ch6 = ch6;
    last_ch8 = ch8;
    last_ch9 = ch9;
    last_ch10 = ch10;
    last_ch11 = ch11;
}

/*************************************************************
 * 11) RECEIVE DATA FUNCTIONS
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

    for (int i = 0; i < 18; i++) {
        int leg = servoMap[i].leg;
        int chan = servoMap[i].pcaChannel;
        int pwm = servoAngleTable[i][3];
        if (pwm != -1) {
            int legIndex = leg - 1;
            selectI2CBus(tcaChannelForLeg[legIndex]);
            pca9685_write_channel(&pwmHandles[legIndex], chan, 0, pwm);
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
 * 12) INITIALIZE
 *************************************************************/

void user_init(void) {
    WRITE_PERI_REG(UART_CLKDIV(0), UART_CLK_FREQ / 115200);
    printf("Start of program\n");
    xTaskCreate(&task_nrf24_receive, "nrf24_rx", 2048, NULL, 1, NULL);
}