#include "nrf24l01.h"
#include "spi_interface.h"
#include "gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp8266/pin_mux_register.h"
#include "esp8266/eagle_soc.h"

/*start of low level functions, specific to the mcu and compiler*/

/*delay in miliseconds*/
void delay_function(uint32_t duration_ms)
{
    vTaskDelay(duration_ms / portTICK_RATE_MS);
}

/*contains all SPI configuations, such as pins and control registers*/
/*SPI control: master, interrupts disabled, clock polarity low when idle, clock phase falling edge, clock up tp 1 MHz*/
void SPI_Initializer()
{
    // Set pin functions for HSPI
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTDI_U, FUNC_HSPIQ_MISO);  // GPIO12 - MISO
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTCK_U, FUNC_HSPID_MOSI);  // GPIO13 - MOSI
    PIN_FUNC_SELECT(PERIPHS_IO_MUX_MTMS_U, FUNC_HSPI_CLK);     // GPIO14 - SCK

    // Initialize SPI
    SpiAttr spiAttr = {
        .mode = SpiMode_Master,
        .subMode = SpiSubMode_0,
        .speed = 7,  // 80 MHz / (7 + 1) = 10 MHz
        .bitOrder = SpiBitOrder_MSBFirst
    };
    SPIInit(SpiNum_HSPI, &spiAttr);
}

/*contains all CSN and CE pins gpio configurations, including setting them as gpio outputs and turning SPI off and CE '1'*/
void pinout_Initializer()
{
    GPIO_ConfigTypeDef io_conf;
    io_conf.GPIO_Pin = (1 << 2) | (1 << 15);  // GPIO2 (CE), GPIO15 (CSN)
    io_conf.GPIO_Mode = GPIO_Mode_Output;     // Set as output
    io_conf.GPIO_Pullup = 0;                  // No pull-up
    io_conf.GPIO_IntrType = 0;                // Disable interrupts (assuming 0 is disable)
    gpio_config(&io_conf);

    // Set initial states: CSN high, CE low
    gpio_output_conf(1 << 15, 1 << 2, 0, 0);  // Set GPIO15 high, GPIO2 low
}

/*CSN pin manipulation to high or low (SPI on or off)*/
void nrf24_SPI(uint8_t input)
{
    if (input) {
        gpio_output_conf(1 << 15, 0, 0, 0);  // Set CSN high
    } else {
        gpio_output_conf(0, 1 << 15, 0, 0);  // Set CSN low
    }
}

/*1 byte SPI shift register send and receive routine*/
uint8_t SPI_send_command(uint8_t command)
{
    while (READ_PERI_REG(SPI_CMD(SpiNum_HSPI)) & SPI_USR);  // Wait for SPI to be idle

    // Set command and address lengths to 0
    CLEAR_PERI_REG_MASK(SPI_USER(SpiNum_HSPI), SPI_USR_COMMAND);
    SET_PERI_REG_BITS(SPI_USER1(SpiNum_HSPI), SPI_USR_ADDR_BITLEN, 0, SPI_USR_ADDR_BITLEN_S);
    CLEAR_PERI_REG_MASK(SPI_USER(SpiNum_HSPI), SPI_USR_ADDR);

    // Set data length to 8 bits for full-duplex transfer
    SET_PERI_REG_MASK(SPI_USER(SpiNum_HSPI), SPI_USR_MOSI | SPI_USR_MISO);
    SET_PERI_REG_BITS(SPI_USER1(SpiNum_HSPI), SPI_USR_MOSI_BITLEN, 7, SPI_USR_MOSI_BITLEN_S);  // 7 means 8 bits
    SET_PERI_REG_BITS(SPI_USER1(SpiNum_HSPI), SPI_USR_MISO_BITLEN, 7, SPI_USR_MISO_BITLEN_S);

    // Load data to send
    WRITE_PERI_REG(SPI_W0(SpiNum_HSPI), command);

    // Start SPI transfer
    SET_PERI_REG_MASK(SPI_CMD(SpiNum_HSPI), SPI_USR);

    // Wait for transfer to complete
    while (READ_PERI_REG(SPI_CMD(SpiNum_HSPI)) & SPI_USR);

    // Read received data
    uint8_t received = READ_PERI_REG(SPI_W0(SpiNum_HSPI)) & 0xFF;
    return received;
}

/*CE pin maniplation to high or low*/
void nrf24_CE(uint8_t input)
{
    if (input) {
        gpio_output_conf(1 << 2, 0, 0, 0);   // Set CE high
    } else {
        gpio_output_conf(0, 1 << 2, 0, 0);   // Set CE low
    }
}