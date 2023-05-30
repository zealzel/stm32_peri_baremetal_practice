#include "stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"

void delay(void) {
    for (uint32_t i = 0; i < 500000/2; i++) {
    }
}

int main(void) {
    GPIO_Handle_t GpioLed, GpioBtn;

    GpioLed.pGPIOx = GPIOD;
    GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
    GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
    GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
    GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;
    // GPIO_PeriClockControl(GPIOD, ENABLE);
    // GPIO_init(&GpioLed);

    GpioBtn.pGPIOx = GPIOD;
    GpioBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
    GpioBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IN;
    GpioBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPEED_FAST;
    GpioBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

    GPIO_PeriClockControl(GPIOD, ENABLE);
    GPIO_init(&GpioLed);
    GPIO_init(&GpioBtn);

    GPIO_WriteToOutputPin(GPIOD, GPIO_PIN_NO_12, GPIO_PIN_RESET);

    while (1) {
        if (GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_5) == BTN_PRESSED) {
            delay();
            GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
        }
    }
}
