#include <Arduino.h>
#include "board.h"
#include "mpu6050.h"
#include "pid.h"


#define SPEED_KP		0.0
#define SPEED_KI		0.0
#define SPEED_KD		0.0

//#define ZIEGLER_NICHOLS_P
//#define ZIEGLER_NICHOLS_PI
//#define ZIEGLER_NICHOLS_PID
#define KU			70.0
#define TU			(1.0/12.0)
#if defined(ZIEGLER_NICHOLS_P)
#define ANGLE_KP	(0.5*KU)
#define ANGLE_KI	0
#define ANGLE_KD	0
#elif defined(ZIEGLER_NICHOLS_PI)
#define ANGLE_KP	(0.45*KU)
#define ANGLE_KI	(1.2*ANGLE_KP/TU)
#define ANGLE_KD	0
#elif defined(ZIEGLER_NICHOLS_PID)
#define ANGLE_KP	(0.60*KU)
#define ANGLE_KI	(2.0*ANGLE_KP/TU)
#define ANGLE_KD	(KP*TU/8.0)
#else
#define ANGLE_KP	20.0
#define ANGLE_KI	0.0
#define ANGLE_KD	0.0
#endif

float fTgtSpeed, fTgtAngle;
float fCurAngle, fSetSpeed;
int32_t nDir;

Pid AnglePid(ANGLE_KP, ANGLE_KI, ANGLE_KD, 200, LOOP_USEC);
Pid SpeedPid(SPEED_KP, SPEED_KI, SPEED_KD, 200, LOOP_USEC);

extern void motor_set_rpm(int16_t rpm);

void balancing_print_k(void)
{
	Serial.print("K,"); Serial.print(fTgtSpeed, 2);
	Serial.print(",\t"); Serial.print(fTgtAngle, 2);
	Serial.print(",\t"); Serial.print(SpeedPid.getKp(), 1);
	Serial.print(",\t"); Serial.print(SpeedPid.getKi(), 1);
	Serial.print(",\t"); Serial.print(SpeedPid.getKd(), 1);
	Serial.print(",\t"); Serial.print(AnglePid.getKp(), 1);
	Serial.print(",\t"); Serial.print(AnglePid.getKi(), 1);
	Serial.print(",\t"); Serial.print(AnglePid.getKd(), 1);
	Serial.println("");
}

void balancing_print(void)
{
	static unsigned long last_ms;
	unsigned long cur_ms = millis();

	if (last_ms / 1000 != cur_ms / 1000) {
		balancing_print_k();
	}
	Serial.print("P,"); Serial.print(cur_ms - last_ms);
	Serial.print(",\t"); Serial.print(AnglePid.getKp(), 1);
	Serial.print(",\t"); Serial.print(AnglePid.getKi(), 1);
	Serial.print(",\t"); Serial.print(AnglePid.getKd(), 1);
	Serial.print(",\t"); Serial.print(fCurAngle, 2);
	Serial.print(",\t"); Serial.print(fSetSpeed, 0);
	Serial.println("");
	last_ms = cur_ms;
}

void balancing_setup()
{
}

void balancing_loop()
{
	fCurAngle = get_pitch_angle();
	if (fCurAngle < -30 || 30 < fCurAngle) {
		motor_set_rpm(0);
		return;
	}

#if 0
	fTgtAngle = SpeedPid.updatePID(fTgtSpeed, fSetSpeed);
#else
	fTgtAngle = 0;
#endif
	fSetSpeed = AnglePid.updatePID(fTgtAngle, fCurAngle);
	motor_set_rpm(fSetSpeed);

	balancing_print();
}

void balancing_inc_tgt(void)
{
	fTgtSpeed += 1;
	balancing_print_k();
}

void balancing_dec_tgt(void)
{
	fTgtSpeed -= 1;
	balancing_print_k();
}

void balancing_inc_dir(void)
{
	nDir += 1;
	balancing_print_k();
}

void balancing_dec_dir(void)
{
	nDir -= 1;
	balancing_print_k();
}

void balancing_reset_tgtdir(void)
{
	nDir = 0;
	fTgtSpeed = 0;
	balancing_print_k();
}

