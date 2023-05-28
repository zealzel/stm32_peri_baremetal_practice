#include <stdint.h>

#define __vo volatile

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#define FLASH_BASEADDR      0x08000000U
#define SRAM1_BASEADDR      0x20000000U // 112kB (112KiB)
#define SRAM2_BASEADDR      0x2001C000U // 16kB (16KiB)
#define ROM_BASEADDR        0x1FFF0000U // System memory
#define SRAM                SRAM1_BASEADDR

/*
 * AHBx and APBx Bus Peripheral base addresses
 */
#define PERIPH_BASEADDR         0x40000000U // Base address of peripherals
#define APB1PERIGH_BASEADDR     PERIPH_BASEADDR // Base address of APB1 peripherals
#define APB2PERIGH_BASEADDR     0x40010000U // Base address of APB2 peripherals
#define AHB1PERIGH_BASEADDR     0x40020000U //  Base address of AHB1 peripherals
#define AHB2PERIGH_BASEADDR     0x50000000U // Base address of AHB2 peripherals
#define RCC_BASEADDR            (AHB1PERIGH_BASEADDR + 0x3800) // RCC base address

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */
#define GPIOA_BASEADDR          (AHB1PERIGH_BASEADDR + 0x0000)
#define GPIOB_BASEADDR          (AHB1PERIGH_BASEADDR + 0x0400)
#define GPIOC_BASEADDR          (AHB1PERIGH_BASEADDR + 0x0800)
#define GPIOD_BASEADDR          (AHB1PERIGH_BASEADDR + 0x0c00)
#define GPIOE_BASEADDR          (AHB1PERIGH_BASEADDR + 0x1000)
#define GPIOF_BASEADDR          (AHB1PERIGH_BASEADDR + 0x1400)
#define GPIOG_BASEADDR          (AHB1PERIGH_BASEADDR + 0x1800)
#define GPIOH_BASEADDR          (AHB1PERIGH_BASEADDR + 0x1c00)
#define GPIOI_BASEADDR          (AHB1PERIGH_BASEADDR + 0x2000)

/* Base addresses of peripherals which are hanging on APB1 bus */
#define I2C1_BASEADDR           (APB1PERIGH_BASEADDR + 0x5400)
#define I2C2_BASEADDR           (APB1PERIGH_BASEADDR + 0x5800)
#define I2C3_BASEADDR           (APB1PERIGH_BASEADDR + 0x5c00)
#define SPI2_BASEADDR           (APB1PERIGH_BASEADDR + 0x3800)
#define SPI3_BASEADDR           (APB1PERIGH_BASEADDR + 0x3c00)
#define USART2_BASEADDR         (APB1PERIGH_BASEADDR + 0x4400)
#define USART3_BASEADDR         (APB1PERIGH_BASEADDR + 0x4800)
#define UART4_BASEADDR          (APB1PERIGH_BASEADDR + 0x4c00)
#define UART5_BASEADDR          (APB1PERIGH_BASEADDR + 0x5000)

/* Base addresses of peripherals which are hanging on APB2 bus */
#define SPI1_BASEADDR           (APB2PERIGH_BASEADDR + 0x3000)
#define USART1_BASEADDR         (APB2PERIGH_BASEADDR + 0x1000)
#define USART6_BASEADDR         (APB2PERIGH_BASEADDR + 0x1400)
#define EXTI_BASEADDR           (APB2PERIGH_BASEADDR + 0x3c00)
#define SYSCFG_BASEADDR         (APB2PERIGH_BASEADDR + 0x3800)


/* Peripheral register definition structures */
typedef struct {
    __vo uint32_t MODER;         // GPIO port mode register
    __vo uint32_t OTYPER;        // GPIO port output type register
    __vo uint32_t OSPEEDR;       // GPIO port output speed register
    __vo uint32_t PUPDR;         // GPIO port pull-up/pull-down register
    __vo uint32_t IDR;           // GPIO port input data register
    __vo uint32_t ODR;           // GPIO port output data register
    __vo uint32_t BSRR;          // GPIO port bit set/reset register
    __vo uint32_t LCKR;          // GPIO port configuration lock register
    __vo uint32_t AFR[2];        // GPIO alternate function register, AF[0] is low, AF[1] is high
} GPIO_RegDef_t;


/* peripheral definition (peripheral base addresses typecasted to xxx_RegDef_t) */
#define GPIOA       ((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB       ((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC       ((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD       ((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE       ((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF       ((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG       ((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH       ((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI       ((GPIO_RegDef_t*)GPIOI_BASEADDR)


/* Peripheral register definition structures for RCC */
typedef struct {
    __vo uint32_t CR;            // RCC clock control register, offset: 0x00
    __vo uint32_t PLLCFGR;       // RCC PLL configuration register, offset: 0x04
    __vo uint32_t CFGR;          // RCC clock configuration register, offset: 0x08
    __vo uint32_t CIR;           // RCC clock interrupt register, offset: 0x0C
    __vo uint32_t AHB1RSTR;      // RCC AHB1 peripheral reset register, offset: 0x10
    __vo uint32_t AHB2RSTR;      // RCC AHB2 peripheral reset register, offset: 0x14
    __vo uint32_t AHB3RSTR;      // RCC AHB3 peripheral reset register, offset: 0x18
    uint32_t      RESERVED0;     // Reserved
    __vo uint32_t APB1RSTR;      // RCC APB1 peripheral reset register, offset: 0x20
    __vo uint32_t APB2RSTR;      // RCC APB2 peripheral reset register, offset: 0x24
    uint32_t      RESERVED1[2];  // Reserved
    __vo uint32_t AHB1ENR;       // RCC AHB1 peripheral clock enable register, offset: 0x30
    __vo uint32_t AHB2ENR;       // RCC AHB2 peripheral clock enable register, offset: 0x34
    __vo uint32_t AHB3ENR;       // RCC AHB3 peripheral clock enable register, offset: 0x38
    uint32_t      RESERVED2;     // Reserved
    __vo uint32_t APB1ENR;       // RCC APB1 peripheral clock enable register, offset: 0x40
    __vo uint32_t APB2ENR;       // RCC APB2 peripheral clock enable register, offset: 0x44
    uint32_t      RESERVED3[2];  // Reserved
    __vo uint32_t AHB1LPENR;     // RCC AHB1 peripheral clock enable in low power mode register, offset: 0x50
    __vo uint32_t AHB2LPENR;     // RCC AHB2 peripheral clock enable in low power mode register, offset: 0x54
    __vo uint32_t AHB3LPENR;     // RCC AHB3 peripheral clock enable in low power mode register, offset: 0x58
    uint32_t      RESERVED4;     // Reserved
    __vo uint32_t APB1LPENR;     // RCC APB1 peripheral clock enable in low power mode register, offset: 0x60
    __vo uint32_t APB2LPENR;     // RCC APB2 peripheral clock enable in low power mode register, offset: 0x64
    uint32_t      RESERVED5[2];  // Reserved
    __vo uint32_t BDCR;          // RCC Backup domain control register, offset: 0x70
    __vo uint32_t CSR;           // RCC clock control & status register, offset: 0x74
    uint32_t      RESERVED6[2];  // Reserved
    __vo uint32_t SSCGR;         // RCC spread spectrum clock generation register, offset: 0x80
    __vo uint32_t PLLI2SCFGR;    // RCC PLLI2S configuration register, offset: 0x84
    __vo uint32_t PLLSAICFGR;    // RCC PLL configuration register, offset: 0x88
    __vo uint32_t DCKCFGR;       // RCC Dedicated Clock Configuration Register, offset: 0x8C
    __vo uint32_t CKGATENR;      // RCC clocks gated enable register, offset: 0x90
    __vo uint32_t DCKCFGR2;      // RCC dedicated clocks configuration register 2, offset: 0x94
} RCC_RegDef_t;

/* peripheral definition (peripheral base addresses typecasted to xxx_RegDef_t) */
#define RCC         ((RCC_RegDef_t*)RCC_BASEADDR)

/* Clock Enable Macros for GPIOx peripherals */
#define GPIOA_PCLK_EN()         (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()         (RCC->AHB1ENR |= (1 << 1))

/* Clock Enable Macros for I2Cx peripherals */
#define I2C1_PCLK_EN()          (RCC->APB1ENR |= (1 << 21))

/* Clock Enable Macros for SPIx peripherals */
#define SPI1_PCLK_EN()          (RCC->APB2ENR |= (1 << 12))

/* Clock Enable Macros for USARTx peripherals */
#define USART1_PCLK_EN()        (RCC->APB2ENR |= (1 << 4))

/* Clock Enable Macros for SYSCFG peripherals */
#define SYSCFG_PCLK_EN()        (RCC->APB2ENR |= (1 << 14))

/* Clock Disable Macros for GPIOx peripherals */
#define GPIOA_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 0))

/* Clock Disable Macros for I2Cx peripherals */
#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 21))

/* Clock Disable Macros for SPIx peripherals */
#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 12))

/* Clock Disable Macros for USARTx peripherals */
#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 4))

/* Clock Disable Macros for SYSCFG peripherals */
#define SYSCFG_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 14))

#endif /* INC_STM32F407XX_H_ */
