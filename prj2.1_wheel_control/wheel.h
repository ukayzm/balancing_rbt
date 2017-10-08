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

#define KP			1.8
#define KI			1.2
#define KD			0.0

#define NUM_CDATA	(26*2+1)

typedef struct {
	int16_t pwm;
	float speed;	/* mm per second */
} CalibData;

extern CalibData calib_data_m3[NUM_CDATA], calib_data_m4[NUM_CDATA];

class Wheel
{
public:
	Wheel(int16_t *counter, unsigned long *total_counter, CalibData cdata[NUM_CDATA]);
	~Wheel(void);
	void AttachMotor(AF_DCMotor *motor);
	void SetSpeed(int16_t mm_per_sec);	/* in mm per second */
	void SetPwm(int16_t pwm);			/* PWM -255 ~ +255 */
	void Loop(void);
	void Print(void);
	void PrintPid(void);
	void Calibrate(void);
	void Calibrate2(void);
	uint8_t bDiag;
	void IncKp(void);
	void IncKi(void);
	void IncKd(void);
	void DecKp(void);
	void DecKi(void);
	void DecKd(void);
private:
	unsigned long ulLastLoopUs;
	int16_t *pCounter;
	unsigned long *pulTotalCounter;
	int16_t anIntr[NUM_INTR_SAVE];
	unsigned long aulInterval[NUM_INTR_SAVE];
	unsigned long ulIntervalUs;
	unsigned long ulTotalCounter;
	AF_DCMotor *pMotor;
	int16_t nCurPwm;
	int16_t nTgtPwm;
	void setMotorPwm(int16_t pwm);
	PID *pPid;
	double fCurSpeed;	/* current speed in mm per second */
	double fPidOutput;
	double fTgtSpeed;
	double nPwmBias;
	unsigned long nextDelay(unsigned long micro);
	double measure(void);
	CalibData *pstCdata;
	int nCdata;
};

#endif // __WHEEL_H__
