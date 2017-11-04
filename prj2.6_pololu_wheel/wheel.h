#ifndef __WHEEL_H__
#define __WHEEL_H__

#include <inttypes.h>

extern int16_t count_m0, count_m1;
extern unsigned long total_count_m0, total_count_m1;

extern void init_motors(void);


#define WHEEL_RADIUS 60 /* mm */
#define MM_PER_INTR  0.924 /* mm / intr */

/* the initial PWM value to start wheel spinning */
#define INITIAL_PWM_M0	 100
#define INITIAL_PWM_M1	 110

/* the minimum PWM value to keep wheel spinning */
#define MIN_PWM_M0	 40
#define MIN_PWM_M1	 40

#define NUM_INTR_SAVE	2

class Wheel
{
public:
	Wheel(int16_t *counter, unsigned long *total_counter, int pwm_pin, int ctrl0_pin, int ctrl1_pin, uint16_t init_pwm, uint16_t min_pwm);
	void SetPwm(int16_t pwm);			/* PWM -255 ~ +255 */
	void Loop(void);
	void Print(void);
	uint8_t bDiag;
	int16_t GetCurPwm(void);
	float GetCurSpeed(void);
	int32_t GetAccIntr(void);
	void ResetAccIntr(void);
	uint16_t GetInitPwm(void);
	uint16_t GetMinPwm(void);
private:
	int pin_pwm, pin_ctrl0, pin_ctrl1;
	unsigned long ulLastLoopUs;
	int16_t *pCounter;
	unsigned long *pulTotalCounter;
	int16_t anIntr[NUM_INTR_SAVE];
	unsigned long aulInterval[NUM_INTR_SAVE];
	unsigned long ulIntervalUs;
	unsigned long ulTotalIntr;
	int32_t nAccIntr;
	int16_t nCurPwm;
	uint16_t unInitPwm;
	uint16_t unMinPwm;
	void setMotorPwm(int16_t pwm);
	float fCurSpeed;	/* current speed in mm per second */
};

#endif // __WHEEL_H__
