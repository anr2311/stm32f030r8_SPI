/*
 * stm32f030xx_spi_driver.c
 *
 *  Created on: Jun 16, 2024
 *      Author: Aditya
 */

#include "stm32f030xx_spi_driver.h"

/********************************************************************
 * Function Name 	 : SPI_PClockControl
 * Brief		 	 : Configures the peripheral clock for SPIx peripheral
 * Input Parameters  : Configuration parameters for a SPI peripheral
 * Return Parameters : None
 * Note / Remarks	 : None
 ********************************************************************/
void SPI_PClockControl (SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if 		(pSPIx == SPI1) SPI1_PCLK_EN();
		else if (pSPIx == SPI2) SPI2_PCLK_EN();
		else	{ /* Do nothing */ }
	}
	else
	{
		if 		(pSPIx == SPI1) SPI1_PCLK_DI();
		else if (pSPIx == SPI2) SPI2_PCLK_DI();
		else	{ /* Do nothing */ }
	}
}

/********************************************************************
 * Function Name 	 : SPI_Init
 * Brief		 	 : Configures the user inputs for SPIx peripheral
 * Input Parameters  : Configuration parameters for a SPI peripheral
 * Return Parameters : None
 * Note / Remarks	 : None
 ********************************************************************/
void SPI_Init (SPI_Handle_t *pSPIHandle)
{
	uint32_t tempreg = 0;

	// Configure the device mode
	tempreg = pSPIHandle->SPIConfig.SPI_DeviceMode << SPIx_CR1_MSTR;

	// Configure the bus config
	if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		// BiDi mode to be cleared
		tempreg &= ~(1 << SPIx_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		// BiDi mode to be enabled
		tempreg |= (1 << SPIx_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RX_ONLY)
	{
		// BiDi mode to be cleared and RxOnly bit to be set
		tempreg &= ~(1 << SPIx_CR1_BIDIMODE);
		tempreg |= (1 << SPIx_CR1_RXONLY);
	}
	else
	{
		// Do nothing
	}

	// Configurations for clock speed
	tempreg |= (pSPIHandle->SPIConfig.SPI_SclkSpeed) << SPIx_CR1_BR;

	// Configure CPOL and CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPIx_CR1_CPOL;
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPIx_CR1_CPHA;

	// Configure SSM
	if (pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_ENABLE)
	{
		tempreg |= (1 << SPIx_CR1_SSM);
	}
	else if (pSPIHandle->SPIConfig.SPI_SSM == SPI_SSM_DISABLE)
	{
		tempreg &= ~(1 << SPIx_CR1_SSM);
	}
	else
	{
		// Do nothing
	}

	// Configure SSI
	if (pSPIHandle->SPIConfig.SPI_SSI == SPI_SSI_ENABLE)
	{
		tempreg |= (1 << SPIx_CR1_SSI);
	}
	else if (pSPIHandle->SPIConfig.SPI_SSI == SPI_SSI_DISABLE)
	{
		tempreg &= ~(1 << SPIx_CR1_SSI);
	}
	else
	{
		// Do nothing
	}

	// Update the CR1 register
	pSPIHandle->pSPIx->SPI_CR1 = tempreg;

	// Update the SPI data size in CR2 register
	if (pSPIHandle->SPIConfig.SPI_DFF == SPI_DFF_8BIT || pSPIHandle->SPIConfig.SPI_DFF == SPI_DFF_16BIT)
	{
		pSPIHandle->pSPIx->SPI_CR2 |= (pSPIHandle->SPIConfig.SPI_DFF << SPIx_CR2_DS);
	}
	else
	{
		/* Data size by default is configured to 8-bits on POR
		   Other data size is not supported by this driver */
	}

	if (pSPIHandle->SPIConfig.SPI_SSOE == SPI_SSOE_ENABLE)
	{
		pSPIHandle->pSPIx->SPI_CR2 |= (1 << SPIx_CR2_SSOE);
	}
	else if (pSPIHandle->SPIConfig.SPI_SSOE == SPI_SSOE_DISABLE)
	{
		pSPIHandle->pSPIx->SPI_CR2 &= ~(1 << SPIx_CR2_SSOE);
	}
	else
	{
		// Do nothing
	}

	// Enable SPI
	pSPIHandle->pSPIx->SPI_CR1 |= (1 << SPIx_CR1_SPE);
}

/********************************************************************
 * Function Name 	 : SPI_Init
 * Brief		 	 : De-initializes the given SPIx peripheral
 * Input Parameters  : Base address of SPIx peripheral
 * Return Parameters : None
 * Note / Remarks	 : None
 ********************************************************************/
void SPI_DeInit (SPI_RegDef_t *pSPIx)
{
	if (pSPIx == SPI1)
	{
		SPI1_REG_RESET();
		SPI1_REG_RESET_CLEAR();
	}
	else if (pSPIx == SPI2)
	{
		SPI2_REG_RESET();
		SPI2_REG_RESET_CLEAR();
	}
	else
	{
		/* Do nothing */
	}
}

/********************************************************************
 * Function Name 	 : SPI_SendData
 * Brief		 	 : Sends the data out of the SPIx peripheral
 * Input Parameters  : Base address of SPIx peripheral, data to be sent and its length
 * Return Parameters : None
 * Note / Remarks	 : Blocking Send
 ********************************************************************/
void SPI_SendData (SPI_RegDef_t *pSPIx, uint8_t* pTxBuffer, uint32_t Len)
{
	while (Len > 0)
	{
		// wait till Tx buffer is empty
		// There is a possibility that this while loop can block permanently
		// Can be overcome using a watchdog
		while ( !(pSPIx->SPI_SR && (1 << 1)));

		if (((pSPIx->SPI_CR2 & (0x0F00u)) >> 16) == SPI_DFF_8BIT)
		{
			// 8-bit data format
			pSPIx->SPI_DR = *pTxBuffer;

			pTxBuffer++;
		}
		else if (((pSPIx->SPI_CR2 & (0x0F00u)) >> 16) == SPI_DFF_16BIT)
		{
			// 16-bit data format
			pSPIx->SPI_DR = *((uint16_t*) pTxBuffer);
			Len--; // since 2 bytes of data is being sent

			(uint16_t*)pTxBuffer++;
		}
		Len--;
	}
}
