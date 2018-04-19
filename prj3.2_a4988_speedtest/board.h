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


/* msec */
#define LOOP_MS     6
//#define INTERVAL_MPU6050	LOOP_MS
//#define INTERVAL_IR			LOOP_MS
#define INTERVAL_BALANCING	LOOP_MS


#endif // __BOARD_H__
