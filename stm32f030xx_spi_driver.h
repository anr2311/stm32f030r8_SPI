/*
 * stm32f030xx_gpio_driver.h
 *
 *  Created on: Jan 1, 2024
 *      Author: Aditya
 */

#ifndef INC_STM32F030XX_SPI_DRIVER_H_
#define INC_STM32F030XX_SPI_DRIVER_H_

#include "stm32f030xx.h"

/* Device Mode MACROs */
#define SPI_DEVICE_MODE_MASTER  1
#define SPI_DEVICE_MODE_SLAVE   0

/* Bus Config MACROs */
#define SPI_BUS_CONFIG_FD	    			1  // Full duplex
#define SPI_BUS_CONFIG_HD   				2  // Half duplex
#define SPI_BUS_CONFIG_SIMPLEX_RX_ONLY   	3

/* Clock speed MACROs (this relates to the baud rates */
#define SPI_SCL_SPEED_DIV2    0  // PCLK will be divided by 2
#define SPI_SCL_SPEED_DIV4    1  // PCLK will be divided by 4
#define SPI_SCL_SPEED_DIV8    2  // PCLK will be divided by 8
#define SPI_SCL_SPEED_DIV16   3  // PCLK will be divided by 16
#define SPI_SCL_SPEED_DIV32   4  // PCLK will be divided by 32
#define SPI_SCL_SPEED_DIV64   5  // PCLK will be divided by 64
#define SPI_SCL_SPEED_DIV128  6  // PCLK will be divided by 128
#define SPI_SCL_SPEED_DIV256  7  // PCLK will be divided by 256

/* DFFs MACROs */
// DFF is DS (4 bit length) in case of STM32F030R8
#define SPI_DFF_8BIT   7
#define SPI_DFF_16BIT  15

/* CPOL MACROs */
#define SPI_CPOL_HIGH   1
#define SPI_CPOL_LOW    0

/* CPHA MACROs */
#define SPI_CPHA_HIGH   1
#define SPI_CPHA_LOW    0

/* SSM MACROs */
#define SPI_SSM_ENABLE   1
#define SPI_SSM_DISABLE  0

/* SSI MACROs */
#define SPI_SSI_ENABLE   1
#define SPI_SSI_DISABLE  0

/* SSOE MACROs */
#define SPI_SSOE_ENABLE   1
#define SPI_SSOE_DISABLE  0

/* Configuration structure */
typedef struct
{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;  // Data Frame Format
	uint8_t SPI_CPHA; // Clock Phase
	uint8_t SPI_CPOL; // Clock Polarity
	uint8_t SPI_SSM;  // Software Slave Management
	uint8_t SPI_SSI;  // Internal Slave Select
	uint8_t SPI_SSOE; // SS output enable
}SPI_Config_t;

/* handle structure for SPIx peripheral */
typedef struct
{
	SPI_RegDef_t *pSPIx;  // Base address of the SPIx peripheral (x:1,2)
	SPI_Config_t SPIConfig;
}SPI_Handle_t;

/* APIs supported by this driver */

void SPI_PClockControl (SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

void SPI_Init (SPI_Handle_t *pSPIHandle);
void SPI_DeInit (SPI_RegDef_t *pSPIx);

void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t Len);
void SPI_ReceiveData (SPI_RegDef_t *pSPIx, uint8_t* pRxBuffer, uint32_t Len);

void SPI_IRQConfig (uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnOrDi);
void SPI_IRQHandling (SPI_Handle_t *pSPIHandle);

#endif /* INC_STM32F030XX_SPI_DRIVER_H_ */
