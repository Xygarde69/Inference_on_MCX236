#ifndef TILT_H_
#define TILT_H_

#include <stdbool.h>

/* Velocity command from the board's tilt, in Go2 units (m/s; play.py scales them to the robot) */
typedef struct
{
    float forward;
    float left;
} tilt_command_t;

/* Init the FXLS8974CF and zero at the current pose. False if the sensor doesn't answer. */
bool tilt_init(void);

/* Take the current pose as neutral (no command) */
bool tilt_zero(void);

/* Read the sensor and update the smoothed command. On a read error the command
 * eases to zero. Call once per control step. */
tilt_command_t tilt_update(void);

#endif /* TILT_H_ */
