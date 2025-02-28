#include "lsm6dsl.h"
#include "i2c.h"
#include <stdio.h>
#include <stdint.h>

#define LSM6DSL_ADDR          0x6A

#define WHO_AM_I_REG          0x0F   // Sensor identification register
#define WHO_AM_I_EXPECTED     0x6A   // Expected value for WHO_AM_I (verify against datasheet)
#define CTRL1_XL              0x10   // Accelerometer control register
#define CTRL3_C               0x12   // Control register 3 (contains IF_INC bit)
#define OUTX_L_XL             0x28   // Starting register for acceleration data

static uint8_t lsm6dsl_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t data[2];
    data[0] = reg;
    data[1] = value;
    return i2c_transaction(LSM6DSL_ADDR, 0, data, 2);
}


static uint8_t lsm6dsl_read_reg(uint8_t reg, uint8_t* value)
{
    uint8_t status;
    // write the register address
    status = i2c_transaction(LSM6DSL_ADDR, 0, &reg, 1);
    if (status != 0)
        return status;

    // read one byte from register
    return i2c_transaction(LSM6DSL_ADDR, 1, value, 1);
}
void lsm6dsl_init(void)
{
    uint8_t status;
    uint8_t who_am_i = 0;

    // Verify sensor identity
    status = lsm6dsl_read_reg(WHO_AM_I_REG, &who_am_i);
    if (status != 0)
    {
        printf("LSM6DSL: Error reading WHO_AM_I register (err %d)\n", status);
        return;
    }
    if (who_am_i != WHO_AM_I_EXPECTED)
    {
        printf("LSM6DSL: Unexpected WHO_AM_I value: 0x%02X\n", who_am_i);
        // Depending on your application, you might want to halt further operation here
    }
    else
    {
        printf("LSM6DSL: WHO_AM_I = 0x%02X\n", who_am_i);
    }

    // Enable register address auto-increment by setting IF_INC in CTRL3_C.
    status = lsm6dsl_write_reg(CTRL3_C, 0x04);
    if (status != 0)
    {
        printf("LSM6DSL: Error configuring CTRL3_C (err %d)\n", status);
    }
    else
    {
        printf("LSM6DSL: CTRL3_C configured (auto-increment enabled)\n");
    }

    //Configure the accelerometer.
    status = lsm6dsl_write_reg(CTRL1_XL, 0x50);
    if (status != 0)
    {
        printf("LSM6DSL: Error writing CTRL1_XL (err %d)\n", status);
    }
    else
    {
        printf("LSM6DSL: CTRL1_XL configured (0x50)\n");
    }
}

void lsm6dsl_read_xyz(int16_t* x, int16_t* y, int16_t* z)
{
    uint8_t status;
    uint8_t data[6];
    /* Use the base register address.
       With auto-increment enabled (via CTRL3_C), the sensor will automatically
       increment the register pointer after each byte.
    */
    uint8_t reg = OUTX_L_XL; 

    //register address
    status = i2c_transaction(LSM6DSL_ADDR, 0, &reg, 1);
    if (status != 0)
    {
        printf("LSM6DSL: Error setting up read address (err %d)\n", status);
        return;
    }

    // Read 6 consecutive bytes: X_L, X_H, Y_L, Y_H, Z_L, Z_H
    status = i2c_transaction(LSM6DSL_ADDR, 1, data, 6);
    if (status != 0)
    {
        printf("LSM6DSL: Error reading acceleration data (err %d)\n", status);
        return;
    }

    // Combine low and high bytes for each axis (little-endian format)
    *x = (int16_t)((data[1] << 8) | data[0]);
    *y = (int16_t)((data[3] << 8) | data[2]);
    *z = (int16_t)((data[5] << 8) | data[4]);
}
