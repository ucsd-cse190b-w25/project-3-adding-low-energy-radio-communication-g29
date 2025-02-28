#include "i2c.h"
#include <stm32l475xx.h>

void i2c_init(void)
{
    // Enable clocks for GPIOB and I2C2
    RCC->AHB2ENR |= RCC_AHB2ENR_GPIOBEN;     // Enable GPIOB clock (for STM32L4)
    RCC->APB1ENR1 |= RCC_APB1ENR1_I2C2EN;     // Enable I2C2 clock (for STM32L4)
    // Clear mode bits for PB10 and PB11
    GPIOB->MODER &= ~((3U << (10 * 2)) | (3U << (11 * 2)));
    // Set mode to Alternate Function (10b)
    GPIOB->MODER |= ((2U << (10 * 2)) | (2U << (11 * 2)));

    GPIOB->AFR[1] &= ~((0xFU << ((10 - 8) * 4)) | (0xFU << ((11 - 8) * 4)));
    GPIOB->AFR[1] |= ((4U << ((10 - 8) * 4)) | (4U << ((11 - 8) * 4)));
    GPIOB->OTYPER |= (1U << 10) | (1U << 11);

    GPIOB->OSPEEDR |= ((3U << (10 * 2)) | (3U << (11 * 2)));

    // Enable internal pull-ups
    GPIOB->PUPDR &= ~((3U << (10 * 2)) | (3U << (11 * 2)));
    GPIOB->PUPDR |= ((1U << (10 * 2)) | (1U << (11 * 2)));

    // Reset I2C2
    I2C2->CR1 |= I2C_CR1_SWRST;
    I2C2->CR1 &= ~I2C_CR1_SWRST;

    // Configure the TIMINGR register.
    I2C2->TIMINGR = 0x10707DBC;  // Adjust this value based on your system clock and desired speed

    // Enable I2C2 peripheral
    I2C2->CR1 |= I2C_CR1_PE;
}

uint8_t i2c_transaction(uint8_t address, uint8_t dir, uint8_t* data, uint8_t len)
{
    uint32_t timeout;

    // Wait until I2C bus is not busy
    timeout = 100000;
    while (I2C2->ISR & I2C_ISR_BUSY)
    {
        if (--timeout == 0)
            return 1;  // timeout error
    }

    // Configure the transfer in the CR2 register.
    I2C2->CR2 = 0;  // Clear previous settings
    I2C2->CR2 |= ((uint32_t)address << 1)              // 7-bit slave address
                | ((uint32_t)len << I2C_CR2_NBYTES_Pos)  // Number of bytes
                | I2C_CR2_AUTOEND;                       // Enable autoend

    if (dir == 0)   // Write operation
    {
        // Write mode: ensure the transfer direction is write (RD_WRN = 0)
        // Initiate the transfer by setting the START bit
        I2C2->CR2 &= ~I2C_CR2_RD_WRN;
        I2C2->CR2 |= I2C_CR2_START;

        for (uint8_t i = 0; i < len; i++)
        {
            // Wait until TXDR is empty (TXIS flag set)
            timeout = 100000;
            while (!(I2C2->ISR & I2C_ISR_TXIS))
            {
                if (--timeout == 0)
                    return 2;  // timeout error during TXIS wait
            }
            I2C2->TXDR = data[i];
        }
    }
    else    // Read operation
    {
        // Read mode: set the RD_WRN bit before starting the transfer.
        I2C2->CR2 |= I2C_CR2_RD_WRN;
        I2C2->CR2 |= I2C_CR2_START;

        for (uint8_t i = 0; i < len; i++)
        {
            // Wait until a byte has been received (RXNE flag set)
            timeout = 100000;
            while (!(I2C2->ISR & I2C_ISR_RXNE))
            {
                if (--timeout == 0)
                    return 3;  // timeout error during RXNE wait
            }
            data[i] = I2C2->RXDR;
        }
    }

    // Wait for the transfer to complete (STOPF flag set)
    timeout = 100000;
    while (!(I2C2->ISR & I2C_ISR_STOPF))
    {
        if (--timeout == 0)
            return 4;  // timeout waiting for STOPF
    }
    // Clear the STOP flag by writing to ICR
    I2C2->ICR = I2C_ICR_STOPCF;

    return 0;
}
