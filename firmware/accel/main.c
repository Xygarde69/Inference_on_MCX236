#include "fsl_debug_console.h"
#include "fsl_fxls.h"
#include "board.h"
#include "app.h"

/* SENS_CONFIG1[2:1] FSR: 00 = +/-2 g, 01 = +/-4 g, 10 = +/-8 g, 11 = +/-16 g */
#define SENS_CONFIG1_FSR_MASK  0x06U
#define SENS_CONFIG1_FSR_SHIFT 1U

static void delay_ms(uint32_t ms)
{
    SDK_DelayAtLeastUs(ms * 1000U, SystemCoreClock);
}

/* 12-bit signed output: +/-2048 counts spans the full-scale range */
static int32_t raw_to_mg(int16_t raw, uint32_t range_g)
{
    return ((int32_t)raw * (int32_t)range_g * 1000) / 2048;
}

int main(void)
{
    fxls_handle_t fxls    = {0};
    fxls_config_t config  = {0};
    fxls_accel_data_t accel;
    uint8_t sens_config1;

    BOARD_InitHardware();
    BOARD_Accel_I2C_Init();

    PRINTF("\r\nFXLS8974CF accelerometer (I2C 0x%02X)\r\n", ACCEL_I2C_ADDR);

    config.I2C_SendFunc    = BOARD_Accel_I2C_Send;
    config.I2C_ReceiveFunc = BOARD_Accel_I2C_Receive;
    config.slaveAddress    = ACCEL_I2C_ADDR;

    /* Checks WHO_AM_I (0x86) and puts the sensor in active mode */
    if (FXLS_Init(&fxls, &config) != kStatus_Success)
    {
        PRINTF("Sensor init failed: no response or wrong WHO_AM_I\r\n");
        while (1)
        {
        }
    }

    if (FXLS_ReadReg(&fxls, SENS_CONFIG1_REG, &sens_config1, 1) != kStatus_Success)
    {
        PRINTF("Failed to read SENS_CONFIG1\r\n");
        while (1)
        {
        }
    }
    uint32_t range_g = 2U << ((sens_config1 & SENS_CONFIG1_FSR_MASK) >> SENS_CONFIG1_FSR_SHIFT);

    PRINTF("Range: +/-%u g, printing every %u ms (values in mg)\r\n\r\n", range_g, ACCEL_SAMPLE_PERIOD_MS);

    while (1)
    {
        if (FXLS_ReadAccelData(&fxls, &accel) == kStatus_Success)
        {
            PRINTF("X: %6d mg   Y: %6d mg   Z: %6d mg\r\n", raw_to_mg(accel.accelX, range_g),
                   raw_to_mg(accel.accelY, range_g), raw_to_mg(accel.accelZ, range_g));
        }
        else
        {
            PRINTF("I2C read failed\r\n");
        }

        delay_ms(ACCEL_SAMPLE_PERIOD_MS);
    }
}
