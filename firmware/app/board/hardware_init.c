#include "fsl_common.h"
#include "fsl_gpio.h"
#include "fsl_port.h"
#include "pin_mux.h"
#include "board.h"
#include "app.h"

static void i2c_release_bus_delay(void)
{
    for (uint32_t i = 0U; i < I2C_RELEASE_BUS_COUNT; i++)
    {
        __NOP();
    }
}

/* If the MCU resets mid-transfer, the sensor can be left holding SDA low.
 * Clock SCL nine times and send a STOP so it lets go of the bus. */
static void BOARD_I2C_ReleaseBus(void)
{
    gpio_pin_config_t pin_config;
    port_pin_config_t i2c_pin_config = {0};

    i2c_pin_config.pullSelect = kPORT_PullUp;
    i2c_pin_config.mux        = kPORT_MuxAsGpio;

    pin_config.pinDirection = kGPIO_DigitalOutput;
    pin_config.outputLogic  = 1U;
    CLOCK_EnableClock(kCLOCK_Port4);
    PORT_SetPinConfig(I2C_RELEASE_SCL_PORT, I2C_RELEASE_SCL_PIN, &i2c_pin_config);
    PORT_SetPinConfig(I2C_RELEASE_SDA_PORT, I2C_RELEASE_SDA_PIN, &i2c_pin_config);

    GPIO_PinInit(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, &pin_config);
    GPIO_PinInit(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, &pin_config);

    /* Drive SDA low first to simulate a start */
    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    /* Send 9 pulses on SCL and keep SDA high */
    for (uint8_t i = 0U; i < 9U; i++)
    {
        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
        i2c_release_bus_delay();

        GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
        i2c_release_bus_delay();
        i2c_release_bus_delay();
    }

    /* Send stop */
    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 0U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SCL_GPIO, I2C_RELEASE_SCL_PIN, 1U);
    i2c_release_bus_delay();

    GPIO_PinWrite(I2C_RELEASE_SDA_GPIO, I2C_RELEASE_SDA_PIN, 1U);
    i2c_release_bus_delay();
}

void BOARD_InitHardware(void)
{
    /* LPI2C2 (accelerometer) runs from FRO 12M; matches BOARD_ACCEL_I2C_CLOCK_FREQ in board.h */
    CLOCK_AttachClk(kFRO12M_to_FLEXCOMM2);
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom2Clk, 1u);

    BOARD_InitPins();
    BOARD_InitBootClocks();

    /* Debug console / policy link on LP_FLEXCOMM4 (MCU-Link VCOM, BOARD_DEBUG_UART_BAUDRATE 8N1) */
    CLOCK_AttachClk(BOARD_DEBUG_UART_CLK_ATTACH);
    CLOCK_SetClkDiv(kCLOCK_DivFlexcom4Clk, 1u);

    BOARD_I2C_ReleaseBus();
    BOARD_I2C_ConfigurePins();
    BOARD_InitDebugConsole();
}
