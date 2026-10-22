#ifndef SYS_STEERING_H
#define SYS_STEERING_H

/**
 * @brief Start the STEER task: on each CTRL (0x010), map the steer value to
 *        the servo PWM pulse (R2.3, S3). Keeps tracking during the system
 *        error state (R3); holds position only if N2 itself is self-failed.
 */
void steering_init(void);

#endif // SYS_STEERING_H
