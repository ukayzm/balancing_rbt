#ifndef __BOARD_H__
#define __BOARD_H__

#include <inttypes.h>

extern int16_t count_m0, count_m1;
extern unsigned long total_count_m0, total_count_m1;

extern void setup_board(void);

#define M0_INTR_PIN		2
#define M0_DIR_PIN 		4
#define M1_INTR_PIN		3
#define M1_DIR_PIN		5

#define M0_PWM_PIN		6
#define M0_CTRL0_PIN	7
#define M0_CTRL1_PIN	8
#define M1_PWM_PIN		9
#define M1_CTRL0_PIN	12
#define M1_CTRL1_PIN	13

#define IR_PIN			A3

#endif // __BOARD_H__
