#ifndef APP_H_
#define APP_H_

#include "board.h"

/* The policy link is the debug console UART itself: LP_FLEXCOMM4 (P1_8 RX, P1_9 TX),
 * which the MCU-Link bridges to USB (J10, /dev/ttyACM0). debug_console_lite polls,
 * so the protocol can read and write LPUART4 directly. */
#define POLICY_UART ((LPUART_Type *)BOARD_DEBUG_UART_BASEADDR)

/* FXLS8974CF (U10) on LPI2C2 (FLEXCOMM2): P4_0 = SDA, P4_1 = SCL. 7-bit address 0x18 (SA0 = 0). */
#define ACCEL_I2C_ADDR 0x18U

/* Bus recovery: bit-bang the I2C lines as GPIO before handing them to LPI2C */
#define I2C_RELEASE_SDA_PORT  PORT4
#define I2C_RELEASE_SCL_PORT  PORT4
#define I2C_RELEASE_SDA_GPIO  GPIO4
#define I2C_RELEASE_SDA_PIN   0U
#define I2C_RELEASE_SCL_GPIO  GPIO4
#define I2C_RELEASE_SCL_PIN   1U
#define I2C_RELEASE_BUS_COUNT 100U

void BOARD_InitHardware(void);

#endif /* APP_H_ */
