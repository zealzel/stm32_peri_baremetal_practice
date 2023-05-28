/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: May 28, 2023
 *      Author: zealzel
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/* Handle structure for GPIO pin */
typedef struct {
    uint8_t GPIO_PinNumber;      /* possible values from @GPIO_PIN_NUMBERS */
    uint8_t GPIO_PinMode;        /* possible values from @GPIO_PIN_MODES */
    uint8_t GPIO_PinSpeed;       /* possible values from @GPIO_PIN_SPEED */
    uint8_t GPIO_PinPuPdControl; /* possible values from @GPIO_PIN_PUPD */
    uint8_t GPIO_PinOPType;      /* possible values from @GPIO_PIN_OPTYPE */
    uint8_t GPIO_PinAltFunMode;  /* possible values from @GPIO_PIN_ALTFN */
} GPIO_PinConfig_t;

/* Configuration structure for GPIOx peripheral */
typedef struct {
    GPIO_RegDef_t* pGPIOx; /* This holds the base address of the GPIO port to
                              which the pin belongs */
    GPIO_PinConfig_t
        GPIO_PinConfig; /* This holds GPIO pin configuration settings */
} GPIO_Handle_t;

/******************************************
 * APIs supported by this driver
 * For more information about the APIs check the function definitions
 ******************************************/

/* Peripheral Clock setup */
void GPIO_PeriClockControl(void);

/* Init and De-init */
void GPIO_init(void);
void GPIO_DeInit(void);

/* Data read and write */
void GPIO_ReadFromInputPin(void);
void GPIO_ReadFromInputPort(void);
void GPIO_WriteToOutputPin(void);
void GPIO_WriteToOutputPort(void);
void GPIO_ToggleOutputPin(void);

/* IRQ configuration and ISR handling */
void GPIO_IRQConfig(void);
void GPIO_IRQHandling(void);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
