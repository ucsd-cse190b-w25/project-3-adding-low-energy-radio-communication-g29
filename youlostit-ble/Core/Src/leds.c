/*
 * leds.c
 *
 *  Created on: Oct 3, 2023
 *      Author: schulman
 */


/* Include memory map of our MCU */
#include <stm32l475xx.h>

void leds_init()
{
  /* Configure PA5 as an output by clearing all bits and setting the mode */
  RCC->AHB2ENR |= (RCC_AHB2ENR_GPIOAEN | RCC_AHB2ENR_GPIOBEN);
  GPIOA->MODER &= ~GPIO_MODER_MODE5;
  GPIOA->MODER |= GPIO_MODER_MODE5_0;

  /* Configure the GPIO output as push pull (transistor for high and low) */
  GPIOA->OTYPER &= ~GPIO_OTYPER_OT5;

  /* Disable the internal pull-up and pull-down resistors */
  GPIOA->PUPDR &= GPIO_PUPDR_PUPD5;

  /* Configure the GPIO to use low speed mode */
  GPIOA->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED5_Pos);

  /* Turn off the LED 1 */
  GPIOA->ODR &= ~GPIO_ODR_OD5;
  //For led2
  // Clear the mode bits for PB14
  GPIOB->MODER &= ~GPIO_MODER_MODE14;

  // Set PB14 as output (01)
  GPIOB->MODER |= GPIO_MODER_MODE14_0;

  // Set output type to push-pull
  GPIOB->OTYPER &= ~GPIO_OTYPER_OT14;

  // Disable internal pull-up/down resistors
  GPIOB->PUPDR &= ~GPIO_PUPDR_PUPD14;

  // Optionally set a speed (e.g., very fast)
  GPIOB->OSPEEDR |= (0x3 << GPIO_OSPEEDR_OSPEED14_Pos);

  // Turn off LED2 initially
  GPIOB->ODR &= ~GPIO_ODR_OD14;
}

void leds_set(uint8_t led)
{
    // LED1: connected to PA5, controlled by bit 0
    if (led & 0x01)
    {
        // If bit 0 is set, turn LED1 on
        GPIOA->ODR |= GPIO_ODR_OD5;
    }
    else
    {
        // Otherwise, turn LED1 off
        GPIOA->ODR &= ~GPIO_ODR_OD5;
    }

    // LED2: connected to PB14, controlled by bit 1
    if (led & 0x02)
    {
        // If bit 1 is set, turn LED2 on
        GPIOB->ODR |= GPIO_ODR_OD14;
    }
    else
    {
        // Otherwise, turn LED2 off
        GPIOB->ODR &= ~GPIO_ODR_OD14;
    }
}
