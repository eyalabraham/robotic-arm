/*****************************************************************************
* config.h
*
* Header file containing robotic arm configuration parameters
*
* Created: June 4, 2018
*
*****************************************************************************/

#ifndef __CONFIG_H__
#define __CONFIG_H__

/****************************************************************************
  Motor IDs
****************************************************************************/
#define     TURRET_STEPPER      0
#define     ARM_STEPPER         1
#define     BOOM_STEPPER        2
#define     GRIP_ROTATE_SERVO   3
#define     GRIP_SERVO          4

#define     STEPPER_LIM_HIGH    1535
#define     STEPPER_LIM_LOW     0

#define     TURRET_MAX_STEP     1500
#define     ARM_MAX_STEP        750
#define     BOOM_MAX_STEP       950

#define     STEPPER_HOMING_RATE 150 // Hz
#define     MAX_STEP_RATE       300

/****************************************************************************
  Limit switch masks
****************************************************************************/
#define     TURRET_LIMIT_SWITCH 0x04
#define     ARM_LIMIT_SWITCH    0x10
#define     BOOM_LIMIT_SWITCH   0x20

/****************************************************************************
  RC servo position ranges
****************************************************************************/
#define     SERVO_MID_POINT 165
#define     SERVO_MIN_POINT 80      // abs. min 78
#define     SERVO_MAX_POINT 250     // abs. max 275

#define     GRIP_ROTATE_MAX_CW  SERVO_MIN_POINT
#define     GRIP_ROTATE_MAX_CCW SERVO_MAX_POINT
#define     GRIP_OPEN_MAX       SERVO_MID_POINT
#define     GRIP_CLOSE_MAX      (SERVO_MIN_POINT+15)

#endif /* __CONFIG_H__ */
