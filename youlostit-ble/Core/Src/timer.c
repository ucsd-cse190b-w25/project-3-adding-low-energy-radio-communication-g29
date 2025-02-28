/*
 * timer.c
 *
 *  Created on: Oct 5, 2023
 *      Author: schulman
 */

#include "timer.h"


void timer_init(TIM_TypeDef* timer)
{
  // TODO implement this
  //Enable the clock for TIM2
  RCC->APB1ENR1 |= RCC_APB1ENR1_TIM2EN;
  //Disable the timer while configuring
  timer->CR1 &= ~TIM_CR1_CEN;
  //Reset the counter and interrupt flags.
  timer->CNT = 0;    // Current timer count
  timer->SR  = 0;
  //Set up timer to auto-reload. By default, ARR is used as the reload.
  timer->ARR = 0xFFFF;
  timer->DIER |= TIM_DIER_UIE; // Update interrupt enable

  //Configure the NVIC to enable the TIM2 global interrupt.
  NVIC_SetPriority(TIM2_IRQn, 2);  // Priority level can be changed as needed
  NVIC_EnableIRQ(TIM2_IRQn);

  //Set the prescaler to divide the timer clock.
  // The default system clock after reset is ~4 MHz on STM32L475.
  //set PSC = 3, the timer clock becomes 4 MHz / (3+1) = 1 MHz,
  //so each timer tick is 1 microsecond.
  timer->PSC = 3;

     // enable the timer
  timer->CR1 |= TIM_CR1_CEN;

}

void timer_reset(TIM_TypeDef* timer)
{
  // TODO implement this
  timer->CNT = 0;
}

void timer_set_ms(TIM_TypeDef* timer, uint16_t period_ms)
{
  // TODO implement this
	//Disable the timer while updating the ARR register to avoid mid-update issues.
	timer->CR1 &= ~TIM_CR1_CEN;

	    // Compute the auto-reload value for the desired period.
	    //    With PSC = 3, the timer ticks at 1 MHz (1 tick = 1 microsecond).
	    //    So for 'period_ms' milliseconds, the total ticks = period_ms * 1000.
	    //    The timer counts from 0 up to ARR. set ARR = (ticks - 1).
	uint32_t ticks = period_ms * 1000UL; // Convert ms to microseconds
	if (ticks > 0xFFFF)
		{
	        ticks = 0xFFFF; // clamp to max if needed
	    }

	timer->ARR = ticks - 1; // Because counting starts at 0

	//Reset the counter to ensure a fresh start for the new period
	timer->CNT = 0;

	//Re-enable the timer now that ARR is updated
	timer->CR1 |= TIM_CR1_CEN;
}
