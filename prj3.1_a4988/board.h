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


#endif // __BOARD_H__
