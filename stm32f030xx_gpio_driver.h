/*
 * stm32f030xx_gpio_driver.h
 *
 *  Created on: Jan 1, 2024
 *      Author: Aditya
 */

#ifndef INC_STM32F030XX_GPIO_DRIVER_H_
#define INC_STM32F030XX_GPIO_DRIVER_H_

#include "stm32f030xx.h"

/* defining all GPIO pin numbers */

#define GPIO_PIN0			0
#define GPIO_PIN1			1
#define GPIO_PIN2			2
#define GPIO_PIN3			3
#define GPIO_PIN4			4
#define GPIO_PIN5			5
#define GPIO_PIN6			6
#define GPIO_PIN7			7
#define GPIO_PIN8			8
#define GPIO_PIN9			9
#define GPIO_PIN10			10
#define GPIO_PIN11			11
#define GPIO_PIN12			12
#define GPIO_PIN13			13
#define GPIO_PIN14			14
#define GPIO_PIN15			15

/* defining all possible GPIO modes */

#define GPIO_MODE_INPUT 	0
#define GPIO_MODE_OUTPUT 	1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

/* defining GPIO possible output types */

#define GPIO_PUSH_PULL		0
#define GPIO_OPEN_DRAIN		1

/* defining GPIO speeds */

#define GPIO_LOW_SPEED1		0
#define GPIO_LOW_SPEED2		2
#define GPIO_MED_SPEED		1
#define GPIO_HI_SPEED		3

/* defining all possible GPIO pull up pull down configurations */

#define GPIO_NOPUPD			0
#define GPIO_PU_ONLY		1
#define GPIO_PD_ONLY		2

/* defining the GPIO base address to Port Code converter */
#define GPIO_BASEADDR_TO_PR(x)	(x == GPIOA) ? 0 : \
								(x == GPIOB) ? 1 : \
								(x == GPIOC) ? 2 : \
								(x == GPIOD) ? 3 : \
								(x == GPIOF) ? 5 : 0

typedef struct {
	uint8_t GPIO_PinNumber;
	uint8_t GPIO_PinMode;
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PinPuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFuncMode;
} GPIOPinConfig_t;


typedef struct {
	// Pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef_t *pGPIOx;

	// GPIO pin configuration structure
	GPIOPinConfig_t GPIO_PinConfig;

} GPIO_Handle_t;

/* APIs supported by this driver */

/* GPIO initialization and de-initialization */
void GPIO_Init (GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit (GPIO_RegDef_t *pGPIOx); /* de-init will be done using the RCC peripheral reset register */

/* Peripheral clock setup */
void GPIO_PClockControl (GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/* GPIO input operations */
uint8_t GPIO_ReadFromInputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadFromInputPort (GPIO_RegDef_t *pGPIOx);

/* GPIO output operations */
void GPIO_WriteToOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_WriteToOutputPort (GPIO_RegDef_t *pGPIOx, uint8_t Value);
void GPIO_ToggleOutputPin (GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/* GPIO interrupt config and handling */
void GPIO_IRQConfig (uint8_t IRQNumber, uint32_t IRQPriority, uint8_t EnOrDi);
void GPIO_IRQHandling (uint8_t PinNumber);

#endif /* INC_STM32F030XX_GPIO_DRIVER_H_ */
