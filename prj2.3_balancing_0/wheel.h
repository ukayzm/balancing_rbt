#ifndef __WHEEL_H__
#define __WHEEL_H__

#include <inttypes.h>
#include <AFMotor.h>
#include <PID_v1.h>

extern int16_t count_m3, count_m4;
extern unsigned long total_count_m3, total_count_m4;

extern void init_encoders(void);

#define WHEEL_RADIUS 42 /* mm */
#define MM_PER_INTR  0.4 /* mm */
#define MS_0_255     50  /* ms (0 mm/s -> max speed 320 mm/s) */
#define MAX_MMPS     320.0 /* mm per second */
#define MIN_PWM		 70  /* the minimum PWM value to spin wheel */

#define NUM_INTR_SAVE	2

class Wheel
{
public:
	Wheel(int16_t *counter, unsigned long *total_counter);
	void AttachMotor(AF_DCMotor *motor);
	void SetPwm(int16_t pwm);			/* PWM -255 ~ +255 */
	void Loop(void);
	void Print(void);
	uint8_t bDiag;
	int16_t GetCurPwm(void);
	float GetCurSpeed(void);
	int32_t GetAccIntr(void);
	void ResetAccIntr(void);
private:
	unsigned long ulLastLoopUs;
	int16_t *pCounter;
	unsigned long *pulTotalCounter;
	int16_t anIntr[NUM_INTR_SAVE];
	unsigned long aulInterval[NUM_INTR_SAVE];
	unsigned long ulIntervalUs;
	unsigned long ulTotalIntr;
	int32_t nAccIntr;
	AF_DCMotor *pMotor;
	int16_t nCurPwm;
	void setMotorPwm(int16_t pwm);
	float fCurSpeed;	/* current speed in mm per second */
};

#endif // __WHEEL_H__
