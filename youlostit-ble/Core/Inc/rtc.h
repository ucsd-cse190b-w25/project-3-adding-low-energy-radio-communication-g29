#ifndef RTC_H
#define RTC_H

#include "stm32l4xx_hal.h"

/* Initialize the RTC for wakeup functionality using the LSE oscillator */
HAL_StatusTypeDef RTC_Init(void);

/* Start the RTC wakeup timer in interrupt mode.
   wakeupCounter: the counter value to achieve the desired wakeup interval (~50ms when set to 102).
   Returns HAL_OK on success. */
HAL_StatusTypeDef RTC_Start(uint32_t wakeupCounter);

/* Stop (disable) the RTC wakeup timer */
void RTC_Stop(void);

#endif // RTC_H
