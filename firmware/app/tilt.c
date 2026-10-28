#include <math.h>
#include "fsl_fxls.h"
#include "board.h"
#include "app.h"
#include "tilt.h"

#define DEG_TO_RAD    0.017453293f
#define ZERO_SAMPLES  16U

static fxls_handle_t s_fxls;
static bool s_ready;
static float s_neutral[3] = {0.0f, 0.0f, 1.0f};
static tilt_command_t s_command;

/* Gravity direction as a unit vector; the length is dropped, which also cancels
 * the part's ~8 % sensitivity error. */
static bool read_up(float up[3])
{
    fxls_accel_data_t raw;

    if (!s_ready || FXLS_ReadAccelData(&s_fxls, &raw) != kStatus_Success)
    {
        return false;
    }
    const float x = raw.accelX, y = raw.accelY, z = raw.accelZ;
    const float norm = sqrtf(x * x + y * y + z * z);
    if (norm < 1.0f)
    {
        return false;
    }
    up[0] = x / norm;
    up[1] = y / norm;
    up[2] = z / norm;
    return true;
}

/* sin(tilt) past the dead zone, scaled to -1..1 at TILT_FULL_DEG */
static float tilt_fraction(float s)
{
    const float dead = sinf((float)TILT_DEADZONE_DEG * DEG_TO_RAD);
    const float full = sinf((float)TILT_FULL_DEG * DEG_TO_RAD);
    const float mag  = (fabsf(s) - dead) / (full - dead);

    if (mag <= 0.0f)
    {
        return 0.0f;
    }
    return copysignf(mag > 1.0f ? 1.0f : mag, s);
}

bool tilt_init(void)
{
    fxls_config_t config = {0};

    BOARD_Accel_I2C_Init();
    config.I2C_SendFunc    = BOARD_Accel_I2C_Send;
    config.I2C_ReceiveFunc = BOARD_Accel_I2C_Receive;
    config.slaveAddress    = ACCEL_I2C_ADDR;

    /* Checks WHO_AM_I (0x86) and puts the sensor in active mode */
    s_ready = (FXLS_Init(&s_fxls, &config) == kStatus_Success);
    return s_ready && tilt_zero();
}

bool tilt_zero(void)
{
    float sum[3] = {0.0f, 0.0f, 0.0f};
    float up[3];

    for (uint32_t i = 0U; i < ZERO_SAMPLES; i++)
    {
        if (!read_up(up))
        {
            return false;
        }
        sum[0] += up[0];
        sum[1] += up[1];
        sum[2] += up[2];
        SDK_DelayAtLeastUs(2000U, SystemCoreClock);
    }
    const float norm = sqrtf(sum[0] * sum[0] + sum[1] * sum[1] + sum[2] * sum[2]);
    for (uint32_t i = 0U; i < 3U; i++)
    {
        s_neutral[i] = sum[i] / norm;
    }
    s_command.forward = 0.0f;
    s_command.left    = 0.0f;
    return true;
}

tilt_command_t tilt_update(void)
{
    float up[3];
    tilt_command_t target = {0.0f, 0.0f};

    if (read_up(up))
    {
        /* Dipping an edge moves "up" away from that edge, so the dipped axis reads less than neutral */
        const float forward = -(float)TILT_FORWARD_SIGN * (up[TILT_FORWARD_AXIS] - s_neutral[TILT_FORWARD_AXIS]);
        const float left    = -(float)TILT_LEFT_SIGN * (up[1U - TILT_FORWARD_AXIS] - s_neutral[1U - TILT_FORWARD_AXIS]);
        const float f       = tilt_fraction(forward);

        target.forward = f * 0.001f * (float)(f >= 0.0f ? TILT_MAX_FORWARD_MMS : TILT_MAX_BACKWARD_MMS);
        target.left    = tilt_fraction(left) * 0.001f * (float)TILT_MAX_LATERAL_MMS;
    }

    /* First-order low-pass, one step per control period */
    const float alpha = (float)TILT_SMOOTHING_PCT * 0.01f;
    s_command.forward += alpha * (target.forward - s_command.forward);
    s_command.left += alpha * (target.left - s_command.left);
    return s_command;
}
