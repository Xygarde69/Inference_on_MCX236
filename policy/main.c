#include <string.h>
#include "fsl_debug_console.h"
#include "fsl_lpuart.h"
#include "board.h"
#include "app.h"
#include "policy_model.h"
#include "tilt.h"

/* Frame protocol, little-endian (see host/nxp_policy.py):
 *   PC -> board  0xA5 cmd payload xor
 *   board -> PC  0x5A cmd payload xor
 * xor covers cmd and payload. */
#define SYNC_REQUEST 0xA5U
#define SYNC_REPLY   0x5AU
#define CMD_INFO     'I' /* -> u32 model CRC, u16 n_obs, u16 n_act, u32 core Hz, u8 accel ok */
#define CMD_POLICY   'P' /* n_obs f32 -> n_act f32 actions, f32 forward, f32 left, u32 cycles */
#define CMD_COMMAND  'C' /* -> f32 forward, f32 left (tilt command only, no inference) */
#define CMD_ZERO     'Z' /* -> u8 ok: current pose becomes the no-command pose */
#define CMD_ERROR    'E' /* u8 code */

#define ERR_CHECKSUM 1U
#define ERR_COMMAND  2U
#define ERR_UART     3U

static float s_obs[POLICY_N_OBS];
static float s_actions[POLICY_N_ACT];
static uint8_t s_frame[4U * POLICY_N_OBS + 1U];
static bool s_accel_ok;

static uint8_t xor_bytes(uint8_t check, const uint8_t *data, size_t n)
{
    for (size_t i = 0U; i < n; i++)
    {
        check ^= data[i];
    }
    return check;
}

static void reply(uint8_t cmd, const uint8_t *payload, size_t n)
{
    const uint8_t head[2] = {SYNC_REPLY, cmd};
    const uint8_t check   = xor_bytes(cmd, payload, n);

    (void)LPUART_WriteBlocking(POLICY_UART, head, sizeof(head));
    (void)LPUART_WriteBlocking(POLICY_UART, payload, n);
    (void)LPUART_WriteBlocking(POLICY_UART, &check, 1U);
}

static void reply_error(uint8_t code)
{
    reply(CMD_ERROR, &code, 1U);
}

static void reply_info(void)
{
    uint8_t out[13];
    const uint32_t crc    = POLICY_MODEL_CRC;
    const uint16_t n_obs  = POLICY_N_OBS;
    const uint16_t n_act  = POLICY_N_ACT;
    const uint32_t clk_hz = SystemCoreClock;

    memcpy(&out[0], &crc, 4U);
    memcpy(&out[4], &n_obs, 2U);
    memcpy(&out[6], &n_act, 2U);
    memcpy(&out[8], &clk_hz, 4U);
    out[12] = s_accel_ok ? 1U : 0U;
    reply(CMD_INFO, out, sizeof(out));
}

static void reply_command(void)
{
    uint8_t out[8];
    const tilt_command_t command = tilt_update();

    memcpy(&out[0], &command.forward, 4U);
    memcpy(&out[4], &command.left, 4U);
    reply(CMD_COMMAND, out, sizeof(out));
}

static void reply_zero(void)
{
    const uint8_t ok = tilt_zero() ? 1U : 0U;

    reply(CMD_ZERO, &ok, 1U);
}

/* The command goes back to the PC, which feeds it into the next observation */
static void run_policy(void)
{
    uint8_t out[4U * POLICY_N_ACT + 12U];

    memcpy(s_obs, s_frame, sizeof(s_obs));
    const tilt_command_t command = tilt_update();
    const uint32_t start         = DWT->CYCCNT;
    policy_forward(s_obs, s_actions);
    const uint32_t cycles = DWT->CYCCNT - start;

    memcpy(out, s_actions, sizeof(s_actions));
    memcpy(&out[sizeof(s_actions)], &command.forward, 4U);
    memcpy(&out[sizeof(s_actions) + 4U], &command.left, 4U);
    memcpy(&out[sizeof(s_actions) + 8U], &cycles, 4U);
    reply(CMD_POLICY, out, sizeof(out));
}

/* Read one request after its sync byte; reply to it or with an error */
static void handle_request(void)
{
    uint8_t cmd;
    size_t n = 0U;

    if (LPUART_ReadBlocking(POLICY_UART, &cmd, 1U) != kStatus_Success)
    {
        reply_error(ERR_UART);
        return;
    }
    switch (cmd)
    {
        case CMD_INFO:
        case CMD_COMMAND:
        case CMD_ZERO:
            break;
        case CMD_POLICY:
            n = 4U * POLICY_N_OBS;
            break;
        default:
            reply_error(ERR_COMMAND);
            return;
    }
    /* payload + checksum */
    if (LPUART_ReadBlocking(POLICY_UART, s_frame, n + 1U) != kStatus_Success)
    {
        reply_error(ERR_UART);
        return;
    }
    if (xor_bytes(cmd, s_frame, n) != s_frame[n])
    {
        reply_error(ERR_CHECKSUM);
        return;
    }
    switch (cmd)
    {
        case CMD_INFO:
            reply_info();
            break;
        case CMD_COMMAND:
            reply_command();
            break;
        case CMD_ZERO:
            reply_zero();
            break;
        default:
            run_policy();
            break;
    }
}

int main(void)
{
    BOARD_InitHardware();

    /* Cycle counter for the inference time reported with every action */
    DCB->DEMCR |= DCB_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    s_accel_ok = tilt_init();

    PRINTF("\r\nOnePolicy on FRDM-MCXN236: %u -> %u MLP, model 0x%08X, core %u MHz\r\n", POLICY_N_OBS,
           POLICY_N_ACT, POLICY_MODEL_CRC, SystemCoreClock / 1000000U);
    PRINTF("Tilt command: %s\r\n", s_accel_ok ? "FXLS8974CF ready" : "accelerometer not found, command stays 0");
    PRINTF("Waiting for observations (host/play_nxp.py)\r\n");

    while (1)
    {
        uint8_t byte;

        /* Anything before a sync byte (line noise, a terminal) is skipped */
        if (LPUART_ReadBlocking(POLICY_UART, &byte, 1U) == kStatus_Success && byte == SYNC_REQUEST)
        {
            handle_request();
        }
    }
}
