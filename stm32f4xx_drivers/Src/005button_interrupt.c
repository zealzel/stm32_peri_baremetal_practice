#include <string.h>
#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(void) {
    for (uint32_t i = 0; i < 500000 / 2; i++) {
    }
}

int main(void) {
    GPIO_Handle_t GpioLed, GpioBtn;
    memset(&GpioLed, 0, sizeof(GpioLed));
    memset(&GpioBtn, 0, sizeof(GpioBtn));

    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_init(&GpioLed);

    // GpioBtn.pGPIOx = GPIOD; GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    // GpioBtn.pGPIOx = GPIOD; GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_1;
    GpioBtn.pGPIOx = GPIOC; GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;

    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;
    // GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    // GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_PeriClockControl(GPIOC, ENABLE);
    GPIO_init(&GpioBtn);

    // IRQ Configurations
    // GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE); GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRI15);
    // GPIO_IRQPriorityConfig(IRQ_NO_EXTI1, NVIC_IRQ_PRI15); GPIO_IRQInterruptConfig(IRQ_NO_EXTI1, ENABLE);
    GPIO_IRQPriorityConfig(IRQ_NO_EXTI15_10, NVIC_IRQ_PRI15); GPIO_IRQInterruptConfig(IRQ_NO_EXTI15_10, ENABLE);

    // GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, GPIO_PIN_SET);
    while (1);
}

// void EXTI9_5_IRQHandler(void) {
// void EXTI1_IRQHandler(void) {
void EXTI15_10_IRQHandler(void) {

    delay();
    // GPIO_IRQHandling(GPIO_PIN_NO_5);
    // GPIO_IRQHandling(GPIO_PIN_NO_1);
    GPIO_IRQHandling(GPIO_PIN_NO_15);

    GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);

}


