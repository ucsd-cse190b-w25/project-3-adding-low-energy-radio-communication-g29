#include "rtc.h"

/* Define the RTC wakeup clock divider value for DIV16 */
#define RTC_WAKEUPCLOCK_RTCCLK_DIV16   3

/* Define bit positions and masks for RTC_CR register */
#define RTC_CR_WCKSEL_Pos   8
#define RTC_CR_WCKSEL       (0x7 << RTC_CR_WCKSEL_Pos)

/* RTC_Init: Initializes the RTC using the LSI clock and sets prescaler values.
   This routine enables the RTC clock, configures the RTC registers, and sets up the NVIC for the RTC wakeup IRQ.
*/
HAL_StatusTypeDef RTC_Init(void)
{
    /* Enable power clock and allow access to the backup domain */
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();

    /* Enable the LSI oscillator */
    __HAL_RCC_LSI_ENABLE();
    while (__HAL_RCC_GET_FLAG(RCC_FLAG_LSIRDY) == RESET);

    /* Select LSI as RTC clock source and enable the RTC clock */
    __HAL_RCC_RTC_CONFIG(RCC_RTCCLKSOURCE_LSI);
    __HAL_RCC_RTC_ENABLE();

    /* Disable write protection for RTC registers */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    /* Enter initialization mode */
    RTC->ISR |= RTC_ISR_INIT;
    while ((RTC->ISR & RTC_ISR_INITF) == 0);

    /* Set prescalers:
         - Asynchronous prescaler: 127
         - Synchronous prescaler: 255
       These values assume an LSI frequency near 32 kHz.
       (If your LSI frequency differs significantly, adjust these values accordingly.) */
    RTC->PRER = (127 << 16) | 255;

    /* Exit initialization mode */
    RTC->ISR &= ~RTC_ISR_INIT;

    /* Re-enable write protection */
    RTC->WPR = 0xFF;

    /* Setup NVIC for the RTC wakeup interrupt */
    HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

    return HAL_OK;
}

/* RTC_Start: Configures and starts the RTC wakeup timer.
   It disables any previous wakeup, waits for the write flag,
   sets the auto-reload counter, configures the clock divider,
   enables the wakeup interrupt, and starts the timer.
*/
HAL_StatusTypeDef RTC_Start(uint32_t wakeupCounter)
{
    /* Disable write protection */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    /* Disable wakeup timer */
    RTC->CR &= ~RTC_CR_WUTE;
    /* Wait for the wakeup timer write flag to be set */
    while (!(RTC->ISR & RTC_ISR_WUTWF)) {}

    /* Set wakeup auto-reload value */
    RTC->WUTR = wakeupCounter;

    /* Clear any pending wakeup flag */
    RTC->ISR &= ~RTC_ISR_WUTF;

    /* Configure wakeup clock selection to DIV16 */
    RTC->CR &= ~RTC_CR_WCKSEL;
    RTC->CR |= (RTC_WAKEUPCLOCK_RTCCLK_DIV16 << RTC_CR_WCKSEL_Pos);

    /* Enable wakeup timer interrupt and the wakeup timer itself */
    RTC->CR |= RTC_CR_WUTIE;
    RTC->CR |= RTC_CR_WUTE;

    /* Re-enable write protection */
    RTC->WPR = 0xFF;

    return HAL_OK;
}

/* RTC_Stop: Disables the RTC wakeup timer and its interrupt. */
void RTC_Stop(void)
{
    /* Disable write protection */
    RTC->WPR = 0xCA;
    RTC->WPR = 0x53;

    /* Disable wakeup timer and its interrupt */
    RTC->CR &= ~(RTC_CR_WUTIE | RTC_CR_WUTE);

    /* Re-enable write protection */
    RTC->WPR = 0xFF;
}

/* RTC Wakeup IRQ Handler.
   This handler is invoked when the RTC wakeup timer expires.
   It clears the RTC wakeup flag and also clears the EXTI pending flag.
*/
void RTC_WKUP_IRQHandler(void)
{
    /* If the wakeup flag is set, clear it */
    if (RTC->ISR & RTC_ISR_WUTF)
    {
        RTC->ISR &= ~RTC_ISR_WUTF;
    }
    /* Clear the EXTI line 20 pending flag (associated with RTC wakeup)
       Note: On STM32L4 devices, use EXTI->PR1 instead of EXTI->PR */
    EXTI->PR1 = (1 << 20);
}
