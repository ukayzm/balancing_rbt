#ifndef __BOARD_H__
#define __BOARD_H__

#include <inttypes.h>
#include "motor.h"

extern int16_t count_m0, count_m1;
extern unsigned long total_count_m0, total_count_m1;

extern void setup_board(void);
extern void setDivisorTimer1(int divisor);
extern uint16_t getPwmFrequencyTimer1(void);

extern Motor motor_left, motor_right;

/* VNH5019 pin */
#define M1INA			13	/* 2 -> 13 */
#define M1INB			4
#define M1EN_DIAG		6
#define M1PWM			9
#define M2INA			7
#define M2INB			8
#define M2EN_DIAG		12
#define M2PWM			10

#define M1CS			A0
#define M2CS			A1

/* arduino pin */
#define M0_INTR_PIN		2
#define M0_DIR_PIN 		11
#define M1_INTR_PIN		3
#define M1_DIR_PIN		5

#define M0_PWM_PIN		M1PWM	/* 9 */
#define M0_CTRL0_PIN	M1INA	/* 2 -> 13 */
#define M0_CTRL1_PIN	M1INB	/* 4 */
#define M0_CURRENT_PIN	M1CS	/* A0 */
#define M1_PWM_PIN		M2PWM	/* 10 */
#define M1_CTRL0_PIN	M2INA	/* 7 */
#define M1_CTRL1_PIN	M2INB	/* 8 */
#define M1_CURRENT_PIN	M2CS	/* A1 */


#define IR_PIN			A3


/* msec */
#define INTERVAL_MPU6050		10
#define INTERVAL_IR				10
#define INTERVAL_MOTOR_UPDATE	10
#define INTERVAL_BALANCING		10


#endif // __BOARD_H__
