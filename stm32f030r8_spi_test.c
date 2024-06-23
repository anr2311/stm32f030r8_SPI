/*
 * stm32f030r8_spi_test.c
 *
 *  Created on: Jun 17, 2024
 *      Author: Aditya
 */

/* PA4 - SPI1_NSS
 * PA5 - SPI1_SCLK
 * PA6 - SPI1_MISO
 * PA7 - SPI1_MOSI
 * ALT FN Mode - 0
 */

#include "stm32f030xx.h"
#include "stm32f030xx_spi_driver.h"
#include "stm32f030xx_gpio_driver.h"
#include "string.h"

void SPI1_GPIO_Init(void)
{
	GPIO_Handle_t SPI_Pins;

	SPI_Pins.pGPIOx = GPIOA;

	SPI_Pins.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;

	SPI_Pins.GPIO_PinConfig.GPIO_PinAltFuncMode = 0;

	SPI_Pins.GPIO_PinConfig.GPIO_PinOPType = GPIO_PUSH_PULL;

	SPI_Pins.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NOPUPD;

	SPI_Pins.GPIO_PinConfig.GPIO_PinSpeed = GPIO_MED_SPEED;

	// SCLK
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN5;
	GPIO_Init(&SPI_Pins);

	// MISO
//	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN6;
//	GPIO_Init(&SPI_Pins);

	// MOSI
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN7;
	GPIO_Init(&SPI_Pins);

	// NSS
	SPI_Pins.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN4;
	GPIO_Init(&SPI_Pins);
}

void SPI1_Init(void)
{
	SPI_Handle_t SPI1Handle;

	SPI1Handle.pSPIx = SPI1;

	SPI1Handle.SPIConfig.SPI_BusConfig = SPI_BUS_CONFIG_FD;

	SPI1Handle.SPIConfig.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;

	SPI1Handle.SPIConfig.SPI_SclkSpeed = SPI_SCL_SPEED_DIV2;

	SPI1Handle.SPIConfig.SPI_DFF = SPI_DFF_8BIT;

	SPI1Handle.SPIConfig.SPI_CPOL = SPI_CPOL_LOW;

	SPI1Handle.SPIConfig.SPI_CPHA = SPI_CPHA_LOW;

	SPI1Handle.SPIConfig.SPI_SSM = SPI_SSM_DISABLE;

	SPI1Handle.SPIConfig.SPI_SSI = SPI_SSI_DISABLE; // makes NSS signal internally high, avoids MODF error

	SPI1Handle.SPIConfig.SPI_SSOE = SPI_SSOE_ENABLE;

	SPI_Init(&SPI1Handle);
}

int main(void)
{
	char user_data[] = "Hello World";

	SPI_Handle_t SPI1Handle;
	SPI1Handle.pSPIx = SPI1;

	uint8_t dataLen = strlen(user_data);

	// Function to initialize GPIO pins as SPI1 pins
	SPI1_GPIO_Init();

	// Easily forgotten --> Enable GPIO peripheral clock
	GPIO_PClockControl(GPIOA, ENABLE);

	SPI1_Init();

	// Easily forgotten --> Enable SPI peripheral clock
	SPI_PClockControl(SPI1, ENABLE);

	// First send the length (number of bytes) info
	SPI_SendData(SPI1, &dataLen, 1);

	// Then send the actual data
	SPI_SendData(SPI1, (uint8_t*)user_data, strlen(user_data));

	// Check if SPI shift registers are still at work (BUSY flag from SR register)
	// Before disabling the clock to the SPIx peripheral
//	while (SPI1Handle.pSPIx->SPI_SR && (1 << 7));

	// Disable clock once data is transmitted
	SPI_PClockControl(SPI1, DISABLE);

	while(1);

	return 0;
}


