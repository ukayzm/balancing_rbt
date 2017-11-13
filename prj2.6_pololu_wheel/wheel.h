#ifndef __WHEEL_H__
#define __WHEEL_H__

#include <inttypes.h>

extern int16_t count_m0, count_m1;
extern unsigned long total_count_m0, total_count_m1;

extern void init_motors(void);


#define WHEEL_RADIUS 60 /* mm */
#define ENCODER_CPR  12	/* interrupt count per motor revolution */
#define GEAR_RATIO   34	/* motor rev per wheel rev */
#define MM_PER_INTR  (WHEEL_RADIUS * 2 * 3.141592 / (ENCODER_CPR * GEAR_RATIO)) /* mm / intr */

/* the initial PWM value to start wheel spinning */
#define INITIAL_PWM_M0	 120
#define INITIAL_PWM_M1	 113

/* the minimum PWM value to keep wheel spinning */
#define MIN_PWM_M0	 40
#define MIN_PWM_M1	 32

/* speed (mm/sec) at the minimum PWM */
#define MIN_SPEED_M0	200
#define MIN_SPEED_M1	200

#define NUM_INTR_SAVE	1

class Wheel
{
public:
	Wheel(int16_t *counter, unsigned long *total_counter, int pwm_pin, int ctrl0_pin, int ctrl1_pin, uint16_t init_pwm, uint16_t min_pwm, float min_speed);
	void SetPwm(int16_t pwm);			/* PWM -255 ~ +255 */
	void Update(void);
	void Print(void);
	uint8_t bDiag;
	int16_t GetCurPwm(void);
	float GetCurSpeed(void);
	int32_t GetAccIntr(void);
	void ResetAccIntr(void);
	uint16_t GetInitPwm(void);
	uint16_t GetMinPwm(void);
	float GetMinSpeed(void);
private:
	int pin_pwm, pin_ctrl0, pin_ctrl1;
	unsigned long ulLastUpdateUs;
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
	float fMinSpeed;
	void setMotorDir(int16_t pwm);
	void setMotorPwm(int16_t pwm);
	float fCurSpeed;	/* current speed in mm per second */
};

#endif // __WHEEL_H__
