#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <inttypes.h>

extern int16_t count_m0, count_m1;
extern unsigned long total_count_m0, total_count_m1;

extern void init_motors(void);

#define INTR_PER_REV	(12 * 34)	/* interrupt count per motor revolution */

/* the initial PWM value to start wheel spinning */
#define INITIAL_PWM_M0	 40
#define INITIAL_PWM_M1	 42

/* the minimum PWM value to keep wheel spinning */
#define MIN_PWM_M0	 5
#define MIN_PWM_M1	 5

/* speed (mm/sec) at the minimum PWM */
#define MIN_RPM_M0	42
#define MIN_RPM_M1	42

#define NUM_INTR_SAVE	3

class Motor
{
public:
	Motor(int16_t *counter, unsigned long *total_counter);
	void SetPIN(int pwm_pin, int ctrl0_pin, int ctrl1_pin, int cs_pin);
	void SetCharacteristics(uint16_t init_pwm, uint16_t min_pwm, uint16_t min_rpm);
	void SetPwm(int16_t pwm);			/* PWM -255 ~ +255 */
	void Update(void);
	void Print(void);
	uint8_t bDiag;
	int16_t GetCurPwm(void);
	int16_t GetTgtPwm(void);
	int16_t GetCurRpm(void);
	int32_t GetAccIntr(void);
	void ResetAccIntr(void);
	uint16_t GetInitPwm(void);
	uint16_t GetMinPwm(void);
	uint16_t GetMinRpm(void);
	uint16_t GetCurrent(void);
private:
	int pin_pwm, pin_ctrl0, pin_ctrl1, pin_cs;
	unsigned long ulLastUpdateUs;
	int16_t *pCounter;
	unsigned long *pulTotalCounter;
	int16_t anIntr[NUM_INTR_SAVE];
	unsigned long aulInterval[NUM_INTR_SAVE];
	unsigned long ulIntervalUs;
	unsigned long ulTotalIntr;
	int32_t nAccIntr;
	int16_t nCurPwm;
	int16_t nTgtPwm;
	int16_t nCurRpm;	/* current speed in mm per second */
	uint16_t unInitPwm;
	uint16_t unMinPwm;
	uint16_t unMinRpm;
	void setMotorDir(int16_t pwm);
	void setMotorPwm(int16_t pwm);
	int calibratePwmAtLowSpeed(int pwm);
};

#endif // __MOTOR_H__
