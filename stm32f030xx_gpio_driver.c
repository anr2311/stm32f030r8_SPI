/*
 * stm32f030xx_gpio_driver.c
 *
 *  Created on: Jan 1, 2024
 *      Author: Aditya
 */

#include "stm32f030xx_gpio_driver.h"

/* GPIO initialization and de-initialization */
/********************************************************************
 * Function Name 	 : GPIO_Init
 * Brief		 	 : Initializes a particular GPIO port / pin
 * Input Parameters  : Configuration parameters for a GPIO port
 * Return Parameters : None
 * Note / Remarks	 : None
 ********************************************************************/
void GPIO_Init (GPIO_Handle_t *pGPIOHandle)
{
	uint32_t TempReg = 0u;

	// Configure the mode of the GPIO pin
	if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) /* Non-interrupt modes */
	{
		TempReg = pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

		pGPIOHandle -> pGPIOx->MODER |= TempReg;

		// Clear TempReg
		TempReg = 0u;
	}
	else
	{
		if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			EXTI->EXTI_FTSR |= (1u << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding RTSR bit
			EXTI->EXTI_RTSR &= ~(1u << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			EXTI->EXTI_RTSR |= (1u << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
			// Clear the corresponding FTSR bit
			EXTI->EXTI_FTSR &= ~(1u << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}
		else if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			EXTI->EXTI_FTSR |= (1u << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

			EXTI->EXTI_RTSR |= (1u << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
		}

		// Configure the GPIO port selection in SYSCFG_EXTICR
		SYSCFG_PCLK_EN(); /* Enable PCLK for SYSCFG peripheral */

		uint8_t temp1, temp2, portcode;

		temp1 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber / 4;
		temp2 = pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber % 4;
		portcode = GPIO_BASEADDR_TO_PR(pGPIOHandle->pGPIOx);
		SYSCFG ->SYSCFG_EXTICR[temp1] = portcode << (4 * temp2);

		// Enable interrupt delivery in the Interrupt Mask Register
		EXTI->EXTI_IMR |= (1u << pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);
	}

	// Configure the speed of the GPIO pin
	TempReg = pGPIOHandle -> GPIO_PinConfig.GPIO_PinSpeed << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle -> pGPIOx->OSPEEDR |= TempReg;

	TempReg = 0u;

	// Configure the PUPD settings
	TempReg = pGPIOHandle -> GPIO_PinConfig.GPIO_PinPuPdControl << (2 * pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle -> pGPIOx->PUPDR |= TempReg;

	TempReg = 0u;

	// Configure the output type
	TempReg = pGPIOHandle -> GPIO_PinConfig.GPIO_PinOPType << (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber);

	pGPIOHandle -> pGPIOx->OTYPER |= TempReg;

	TempReg = 0u;

	// Configure the alternate functionality (if required)
	if (pGPIOHandle -> GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		uint8_t temp1 = 0u, temp2 = 0u;

		temp1 = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber) / 8;
		temp2 = (pGPIOHandle -> GPIO_PinConfig.GPIO_PinNumber) % 8;

		TempReg = pGPIOHandle -> GPIO_PinConfig.GPIO_PinAltFuncMode << (4u * temp2);

		if (temp1 == 0)
		{
			pGPIOHandle ->pGPIOx->AFRL |= TempReg;
		}
		else
		{
			pGPIOHandle ->pGPIOx->AFRH |= TempReg;
		}
	}
}

/********************************************************************
 * Function Name 	 : GPIO_DeInit
 * Brief		 	 : Deinitializes the GPIO peripheral back to its Reset values
 * Input Parameters  : Base address of the GPIO peripheral
 * Return Parameters : None
 * Note / Remarks	 : None
 ********************************************************************/
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx) /* de-init will be done using the RCC peripheral reset register */
{
	if (pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
		GPIOA_REG_RESET_CLEAR();
	}
	else if (pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
		GPIOB_REG_RESET_CLEAR();
	}
	else if (pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
		GPIOC_REG_RESET_CLEAR();
	}
	else if (pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
		GPIOD_REG_RESET_CLEAR();
	}
	else if (pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
		GPIOF_REG_RESET_CLEAR();
	}
}

/* Peripheral clock setup */
/********************************************************************
 * Function Name 	 : GPIO_PClockControl
 * Brief		 	 : Enables or disables the peripheral clock for a GPIO port
 * Input Parameters  : Base address of the GPIO peripheral; ENABLE / DISABLE
 * Return Parameters : None
 * Note / Remarks	 : None
 ********************************************************************/
void GPIO_PClockControl (GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		if 		(pGPIOx == GPIOA) GPIOA_PCLK_EN();
		else if (pGPIOx == GPIOB) GPIOB_PCLK_EN();
		else if (pGPIOx == GPIOC) GPIOC_PCLK_EN();
		else if (pGPIOx == GPIOD) GPIOD_PCLK_EN();
		else if (pGPIOx == GPIOF) GPIOF_PCLK_EN();
	}
	else
	{
		if 		(pGPIOx == GPIOA) GPIOA_PCLK_DI();
		else if (pGPIOx == GPIOB) GPIOB_PCLK_DI();
		else if (pGPIOx == GPIOC) GPIOC_PCLK_DI();
		else if (pGPIOx == GPIOD) GPIOD_PCLK_DI();
		else if (pGPIOx == GPIOF) GPIOF_PCLK_DI();
	}
}

/* GPIO input operations */
/********************************************************************
 * Function Name 	 : GPIO_ReadFromInputPin
 * Brief		 	 : Returns the input data value of the selected pin of GPIO port
 * Input Parameters  : Base address of the GPIO peripheral; PinNumber of the pin to be read
 * Return Parameters : Returns the value on the input pin of the GPIO peripheral
 * Note / Remarks	 : None
 ********************************************************************/
uint8_t GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;

	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);

	return value;
}

/********************************************************************
 * Function Name 	 : GPIO_ReadFromInputPort
 * Brief		 	 : Returns the input data value of the selected GPIO port
 * Input Parameters  : Base address of the GPIO peripheral
 * Return Parameters : Returns the value on the GPIO port
 * Note / Remarks	 : None
 ********************************************************************/
uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;

	value = (uint16_t)(pGPIOx->IDR & 0x0000FFFF);

	return value;
}

/* GPIO output operations */
/********************************************************************
 * Function Name 	 : GPIO_WriteToOutputPin
 * Brief		 	 : Writes the input value to the output data register for corresponding pin
 * Input Parameters  : Base address of the GPIO peripheral; pin number of the GPIO port; Value to be written
 * Return Parameters : None
 * Note / Remarks	 : None
 ********************************************************************/
void GPIO_WriteToOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if (Value == GPIO_PIN_SET)
	{
		pGPIOx ->ODR |= (1u << PinNumber);
	}
	else
	{
		pGPIOx ->ODR &= ~(1u << PinNumber);
	}
}

/********************************************************************
 * Function Name 	 : GPIO_WriteToOutputPort
 * Brief		 	 : Writes the input value to the output data register for corresponding port
 * Input Parameters  : Base address of the GPIO peripheral; Value to be written
 * Return Parameters : None
 * Note / Remarks	 : None
 ********************************************************************/
void GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx, uint8_t Value)
{
	pGPIOx ->ODR = Value;
}

/********************************************************************
 * Function Name 	 : GPIO_ToggleOutputPin
 * Brief		 	 : Toggles the bit field of the given GPIO port
 * Input Parameters  : Base address of the peripheral; Pin number of the pin to be toggled
 * Return Parameters : None
 * Note / Remarks	 : None
 ********************************************************************/
void GPIO_ToggleOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR = pGPIOx->ODR ^ (1u << PinNumber);
}

/* GPIO interrupt config and handling */
/********************************************************************
 * Function Name 	 : GPIO_IRQConfig
 * Brief		 	 :
 * Input Parameters  :
 * Return Parameters :
 * Note / Remarks	 :
 ********************************************************************/
void GPIO_IRQConfig (uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnOrDi)
{
	if (EnOrDi == ENABLE)
	{
		// Program ISER register to enable the interrupt
		(*NVIC_ISER) |= (1u << IRQNumber);
	}
	else
	{
		// Program ICER register to disable the interrupt
		(*NVIC_ICER) |= (1u << IRQNumber);
	}

	// Code to set the interrupt priority - lower the value higher the priority of the interrupt
	uint8_t temp = IRQNumber % 4;

	if (IRQNumber <= 3)
	{
		// Left shifting by 6 is required as Cortex M0 reads only bits [7:6] and ignores the bits [5:0]
		(*NVIC_IPR0) |= ((IRQPriority) << (8 * temp)) << 6;
	}
	else if (IRQNumber <= 7)
	{
		(*NVIC_IPR1) |= ((IRQPriority) << (8 * temp)) << 6;
	}
	else if (IRQNumber <= 11)
	{
		(*NVIC_IPR2) |= ((IRQPriority) << (8 * temp)) << 6;
	}
	else if (IRQNumber <= 15)
	{
		(*NVIC_IPR3) |= ((IRQPriority) << (8 * temp)) << 6;
	}
	else if (IRQNumber <= 19)
	{
		(*NVIC_IPR4) |= ((IRQPriority) << (8 * temp)) << 6;
	}
	else if (IRQNumber <= 23)
	{
		(*NVIC_IPR5) |= ((IRQPriority) << (8 * temp)) << 6;
	}
	else if (IRQNumber <= 27)
	{
		(*NVIC_IPR6) |= ((IRQPriority) << (8 * temp)) << 6;
	}
	else if (IRQNumber <= 31)
	{
		(*NVIC_IPR7) |= ((IRQPriority) << (8 * temp)) << 6;
	}
	else {} // do nothing
}

/********************************************************************
 * Function Name 	 : GPIO_IRQHandling
 * Brief		 	 :
 * Input Parameters  :
 * Return Parameters :
 * Note / Remarks	 :
 ********************************************************************/
void GPIO_IRQHandling (uint8_t PinNumber)
{
	if (EXTI->EXTI_PR & (1 << PinNumber))
	{
		// Clear the pending IRQ
		EXTI ->EXTI_PR |= (1u << PinNumber);
	}
}
