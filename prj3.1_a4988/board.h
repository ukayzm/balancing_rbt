#ifndef __BOARD_H__
#define __BOARD_H__

#include <inttypes.h>

extern void setup_board(void);
extern void setup_IR(void);
extern uint32_t recv_IR(void);


/* arduino pin */
#define EN0		5
#define STEP0	6
#define DIR0	7
#define EN1		8
#define STEP1	9
#define DIR1	10


#define IR_PIN			A3


#define LOOP_USEC     6000


#define SPEED_KP	0.00
#define SPEED_KI	0.0
#define SPEED_KD	0.0
#define ANGLE_KP	25.0
#define ANGLE_KI	0.0
#define ANGLE_KD	0.1

#define SPEED_KP_STEP	0.01
#define SPEED_KI_STEP	0.1
#define SPEED_KD_STEP	0.1
#define ANGLE_KP_STEP	1
#define ANGLE_KI_STEP	0.1
#define ANGLE_KD_STEP	0.01

#define SHARE_TIMER2_WITH_IRREMOTE
//#define USE_MOTOR_TIMER1


#endif // __BOARD_H__
