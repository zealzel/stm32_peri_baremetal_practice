/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: May 28, 2023
 *      Author: zealzel
 */

#include "stm32f407xx_gpio_driver.h"

/* Peripheral Clock setup */

/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for
 * the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_PeriClockControl(GPIO_RegDef_t* pGPIOx, uint8_t EnorDi) {
    if (EnorDi == ENABLE) {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_EN();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_EN();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_EN();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_EN();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_EN();
        } else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_EN();
        } else if (pGPIOx == GPIOG) {
            GPIOG_PCLK_EN();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_EN();
        } else if (pGPIOx == GPIOI) {
            GPIOI_PCLK_EN();
        }
    } else {
        if (pGPIOx == GPIOA) {
            GPIOA_PCLK_DI();
        } else if (pGPIOx == GPIOB) {
            GPIOB_PCLK_DI();
        } else if (pGPIOx == GPIOC) {
            GPIOC_PCLK_DI();
        } else if (pGPIOx == GPIOD) {
            GPIOD_PCLK_DI();
        } else if (pGPIOx == GPIOE) {
            GPIOE_PCLK_DI();
        } else if (pGPIOx == GPIOF) {
            GPIOF_PCLK_DI();
        } else if (pGPIOx == GPIOG) {
            GPIOG_PCLK_DI();
        } else if (pGPIOx == GPIOH) {
            GPIOH_PCLK_DI();
        } else if (pGPIOx == GPIOI) {
            GPIOI_PCLK_DI();
        }
    }
}

/* Init and De-init */

/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_init(GPIO_Handle_t* pGPIOHandle) {
    uint32_t temp = 0; // temp register

    // 1. Configure the mode of GPIO pin
    if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG) {
        // The non interrupt mode
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode
                << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->MODER &=
            ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
        pGPIOHandle->pGPIOx->MODER |= temp;
        temp = 0;

        // 2. Configure the speed
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed
                << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->OSPEEDR &=
            ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
        pGPIOHandle->pGPIOx->OSPEEDR |= temp;
        temp = 0;

        // 3. Configure the pupd settings
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl
                << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
        pGPIOHandle->pGPIOx->PUPDR &=
            ~(0x3 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
        pGPIOHandle->pGPIOx->PUPDR |= temp;
        temp = 0;

        // 4. Configure the output type
        temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType
                << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
        pGPIOHandle->pGPIOx->OTYPER &=
            ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // clearing
        pGPIOHandle->pGPIOx->OTYPER |= temp;
        temp = 0;
        // 5. Configure the alternate functionality
        if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN) {
            // Configure the alternate function register
            uint8_t temp1, temp2;
            temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
            temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
            pGPIOHandle->pGPIOx->AFR[temp1] &=
                ~(0xF << (4 * temp2)); // clearing
            pGPIOHandle->pGPIOx->AFR[temp1] |=
                (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4 * temp2));
        }
    } else {
        // This part will code later. (Interrupt mode)
    }
}

/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function de-initializes the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_DeInit(GPIO_RegDef_t* pGPIOx) {
    if (pGPIOx == GPIOA) {
        GPIOA_REG_RESET();
    } else if (pGPIOx == GPIOB) {
        GPIOB_REG_RESET();
    } else if (pGPIOx == GPIOC) {
        GPIOC_REG_RESET();
    } else if (pGPIOx == GPIOD) {
        GPIOD_REG_RESET();
    } else if (pGPIOx == GPIOE) {
        GPIOE_REG_RESET();
    } else if (pGPIOx == GPIOF) {
        GPIOF_REG_RESET();
    } else if (pGPIOx == GPIOG) {
        GPIOG_REG_RESET();
    } else if (pGPIOx == GPIOH) {
        GPIOH_REG_RESET();
    } else if (pGPIOx == GPIOI) {
        GPIOI_REG_RESET();
    }
}

/* Data read and write */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber) {
    // Implementation
    uint8_t value;
    value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
    return value;
}

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t* pGPIOx) {
    uint16_t value;
    value = (uint16_t)pGPIOx->IDR;
    return value;
}

void GPIO_WriteToOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber,
                           uint8_t Value) {
    if (Value == GPIO_PIN_SET) {
        // Write 1 to the output data register at the bit field corresponding to
        // the pin number
        pGPIOx->ODR |= (1 << PinNumber);
    } else {
        // Write 0
        pGPIOx->ODR &= ~(1 << PinNumber);
    }
}
void GPIO_WriteToOutputPort(GPIO_RegDef_t* pGPIOx, uint16_t Value) {
    pGPIOx->ODR = Value;
}

void GPIO_ToggleOutputPin(GPIO_RegDef_t* pGPIOx, uint8_t PinNumber) {
    pGPIOx->ODR ^= (1 << PinNumber);
}

/* IRQ configuration and ISR handling */
void GPIO_IRQConfig(uint8_t IRQNumber, uint8_t IRQPriority, uint8_t EnorDi) {}
void GPIO_IRQHandling(uint8_t PinNumber) {}
