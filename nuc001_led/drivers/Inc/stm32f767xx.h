#include <stdint.h>

#define __vo volatile

/* processor specific details */
/* ARM Cortex Mx processor NVIC ISERx register addresses */
#define NVIC_ISER0          ((__vo uint32_t*)0xE000E100)
#define NVIC_ISER1          ((__vo uint32_t*)0xE000E104)
#define NVIC_ISER2          ((__vo uint32_t*)0xE000E108)
#define NVIC_ISER3          ((__vo uint32_t*)0xE000E10C)

/* ARM Cortex Mx processor NVIC ICERx register addresses */
#define NVIC_ICER0          ((__vo uint32_t*)0xE000E180)
#define NVIC_ICER1          ((__vo uint32_t*)0xE000E184)
#define NVIC_ICER2          ((__vo uint32_t*)0xE000E188)
#define NVIC_ICER3          ((__vo uint32_t*)0xE000E18C)

/* ARM Cortex Mx processor priority register address calculation */
#define NVIC_PR_BASE_ADDR   ((__vo uint32_t*)0xE000E400)
#define NO_PR_BITS_IMPLEMENTED  4


#ifndef INC_STM32F767XX_H_
#define INC_STM32F767XX_H_

/* memory base addresses map to stm32f767xx.h */
#define FLASH_BASEADDR      0x08000000U // 1MB (1MiB)
#define SRAM1_BASEADDR      0x20020000U // 368kB (368KiB)
#define SRAM2_BASEADDR      0x2007C000U // 16kB (16KiB)
#define ROM_BASEADDR        0x1FFF0000U // System memory
#define SRAM                SRAM1_BASEADDR

// #define FLASH_BASEADDR      0x08000000U
// #define SRAM1_BASEADDR      0x20000000U // 112kB (112KiB)
// #define SRAM2_BASEADDR      0x2001C000U // 16kB (16KiB)
// #define ROM_BASEADDR        0x1FFF0000U // System memory
// #define SRAM                SRAM1_BASEADDR

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
#define RCC         ((RCC_RegDef_t*)RCC_BASEADDR)
#define EXTI        ((EXTI_RegDef_t*)EXTI_BASEADDR)
#define SYSCFG      ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)



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


/* Peripheral register definition structures for EXTI */
typedef struct {
    __vo uint32_t EXTI_IMR;   // Interrupt mask register
    __vo uint32_t EXTI_EMR;   // Event mask register
    __vo uint32_t EXTI_RTSR;  // Rising trigger selection register
    __vo uint32_t EXTI_FTSR;  // Falling trigger selection register
    __vo uint32_t EXTI_SWIER; // Software interrupt event register
    __vo uint32_t EXTI_PR;    // Pending register
} EXTI_RegDef_t;

/* peripheral register definition structures for SYSCFG */
typedef struct {
    __vo uint32_t MEMRMP;       // SYSCFG memory remap register, offset: 0x00
    __vo uint32_t PMC;          // SYSCFG peripheral mode configuration register, offset: 0x04
    __vo uint32_t EXTICR[4];    // SYSCFG external interrupt configuration register 1, offset: 0x08
    uint32_t      RESERVED1[2]; // Reserved
    __vo uint32_t CMPCR;        // Compensation cell control register, offset: 0x20
    uint32_t      RESERVED2[2]; // Reserved
    __vo uint32_t CFGR;         // SYSCFG configuration register, offset: 0x2C
} SYSCFG_RegDef_t;

/* NVIC register definition structure */
typedef struct {
    __vo uint32_t ISER[8];  // Interrupt Set Enable Register
    __vo uint32_t ICER[8];  // Interrupt Clear Enable Register
    __vo uint32_t ISPR[8];  // Interrupt Set Pending Register
    __vo uint32_t ICPR[8];  // Interrupt Clear Pending Register
    __vo uint32_t IABR[8];  // Interrupt Active bit Register
    __vo uint32_t IPR[60];  // Interrupt Priority Register
} NVIC_RegDef_t;



/* Clock Enable Macros for GPIOx peripherals */
#define GPIOA_PCLK_EN()         (RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()         (RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()         (RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()         (RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()         (RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()         (RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()         (RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()         (RCC->AHB1ENR |= (1 << 7))
#define GPIOI_PCLK_EN()         (RCC->AHB1ENR |= (1 << 8))


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
#define GPIOB_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 7))
#define GPIOI_PCLK_DI()         (RCC->AHB1ENR &= ~(1 << 8))


/* Clock Disable Macros for I2Cx peripherals */
#define I2C1_PCLK_DI()          (RCC->APB1ENR &= ~(1 << 21))

/* Clock Disable Macros for SPIx peripherals */
#define SPI1_PCLK_DI()          (RCC->APB2ENR &= ~(1 << 12))

/* Clock Disable Macros for USARTx peripherals */
#define USART1_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 4))

/* Clock Disable Macros for SYSCFG peripherals */
#define SYSCFG_PCLK_DI()        (RCC->APB2ENR &= ~(1 << 14))

/* Macros to reset GPIOx peripherals */
#define GPIOA_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0));}while(0)
#define GPIOB_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1));}while(0)
#define GPIOC_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2));}while(0)
#define GPIOD_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3));}while(0)
#define GPIOE_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4));}while(0)
#define GPIOF_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5));}while(0)
#define GPIOG_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6));}while(0)
#define GPIOH_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7));}while(0)
#define GPIOI_REG_RESET() do{(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8));}while(0)


/* Returns port code for given GPIOx base address */
#define GPIO_BASEADDR_TO_CODE(x)   ((x == GPIOA) ? 0 :\
                                     (x == GPIOB) ? 1 :\
                                     (x == GPIOC) ? 2 :\
                                     (x == GPIOD) ? 3 :\
                                     (x == GPIOE) ? 4 :\
                                     (x == GPIOF) ? 5 :\
                                     (x == GPIOG) ? 6 :\
                                     (x == GPIOH) ? 7 :\
                                     (x == GPIOI) ? 8 :0)

/* IRQ(Interrupt Request) Numbers of STM32F407x MCU */
#define IRQ_NO_EXTI0            6
#define IRQ_NO_EXTI1            7
#define IRQ_NO_EXTI2            8
#define IRQ_NO_EXTI3            9
#define IRQ_NO_EXTI4            10
#define IRQ_NO_EXTI9_5          23
#define IRQ_NO_EXTI15_10        40

/* IRQ(Interrupt Request) Priority Numbers of STM32F407x MCU */
#define NVIC_IRQ_PRI0           0
#define NVIC_IRQ_PRI15           15


/* Some generic macros */
#define ENABLE                  1
#define DISABLE                 0
#define SET                     ENABLE
#define RESET                   DISABLE
#define GPIO_PIN_SET            SET
#define GPIO_PIN_RESET          RESET

#endif /* INC_STM32F767XX_H_ */
