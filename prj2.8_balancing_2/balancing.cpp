#include <Arduino.h>
#include <PID_v1.h>
#include "board.h"
#include "mpu6050.h"
#include "motor.h"


#define SPEED_KP		0.0
#define SPEED_KI		0.0
#define SPEED_KD		0.0

#define ANGLE_KP	15.0
#define ANGLE_KI	0.0
#define ANGLE_KD	0.0

double fTgtSpeed, fInSpeed;
double fTgtAngle, fInAngle, fPwm;
int32_t nDir;

PID gPidSpeed(&fInSpeed, &fTgtAngle, &fTgtSpeed, SPEED_KP, SPEED_KI, SPEED_KD, REVERSE);
PID gPidAngle(&fInAngle, &fPwm, &fTgtAngle, ANGLE_KP, ANGLE_KI, ANGLE_KD, DIRECT);

void balancing_print(void)
{
	static unsigned long last_ms;
	unsigned long cur_ms = millis();
	Serial.print(cur_ms - last_ms);
#if 1
	Serial.print(","); Serial.print(fTgtSpeed, 2);
	Serial.print(","); Serial.print(fTgtAngle, 2);
	Serial.print(","); Serial.print(get_pitch_angle(), 2);
	Serial.print(","); Serial.print(fPwm, 0);
	Serial.print(","); Serial.print(motor_left.GetCurPwm());
	Serial.print(","); Serial.print(motor_right.GetCurPwm());
	//Serial.print(","); Serial.print(motor_left.GetAccIntr());
	//Serial.print(","); Serial.print(motor_right.GetAccIntr());
	Serial.print(","); Serial.print(motor_left.GetCurRpm());
	Serial.print(","); Serial.print(motor_right.GetCurRpm());
	Serial.print(","); Serial.print(motor_left.GetCurrent());
	Serial.print(","); Serial.print(motor_right.GetCurrent());
	Serial.print(","); Serial.print(gPidSpeed.GetKp(), 1);
	Serial.print(","); Serial.print(gPidSpeed.GetKi(), 1);
	Serial.print(","); Serial.print(gPidSpeed.GetKd(), 1);
	Serial.print(","); Serial.print(gPidAngle.GetKp(), 1);
	Serial.print(","); Serial.print(gPidAngle.GetKi(), 1);
	Serial.print(","); Serial.print(gPidAngle.GetKd(), 1);
#endif
	Serial.println("");
	last_ms = cur_ms;
}

void balancing_setup()
{
	//gPidSpeed.SetSampleTime(INTERVAL_BALANCING);
	gPidSpeed.SetMode(AUTOMATIC);
	gPidSpeed.SetOutputLimits(-10, 10);

	//gPidAngle.SetSampleTime(INTERVAL_BALANCING);
	gPidAngle.SetMode(AUTOMATIC);
	gPidAngle.SetOutputLimits(-255, 255);
}

/*
 * PID control inspired by https://www.jjrobots.com/projects-2/b-robot/
 */
void compute_pid(float angle_pitch)
{
	int32_t count_l, count_r;
	int32_t cur_count;

	count_l = motor_left.GetAccIntr();
	motor_left.ResetAccIntr();
	count_r = motor_right.GetAccIntr();
	motor_right.ResetAccIntr();

	fInSpeed = count_l + count_r;	/* current speed * a */
	gPidSpeed.Compute();

	fInAngle = angle_pitch;
	gPidAngle.Compute();
}

void balancing_loop()
{
	float angle_pitch;
	int nPwmL, nPwmR;

	motor_left.Update();
	motor_right.Update();

	angle_pitch = get_pitch_angle();

	if (angle_pitch < -30 || 30 < angle_pitch) {
		motor_left.SetPwm(0);
		motor_right.SetPwm(0);

		if (motor_left.GetCurRpm() || motor_right.GetCurRpm()) {
			balancing_print();
		}
		return;
	}

	compute_pid(angle_pitch);

	//fPwm = fPwm * abs(fPwm);

	nPwmL = fPwm + nDir;
	nPwmR = fPwm - nDir;

#if 1
#if 1
	if (nPwmL == 0) {
		motor_left.SetPwm(0);
	} else if (nPwmL > 0) {
		motor_left.SetPwm(nPwmL);
	} else {
		motor_left.SetPwm(nPwmL);
	}
	if (nPwmR == 0) {
		motor_right.SetPwm(0);
	} else if (nPwmR > 0) {
		motor_right.SetPwm(nPwmR);
	} else {
		motor_right.SetPwm(nPwmR);
	}
#else
	motor_left.SetCharacteristics(0, 0, 0);
	motor_right.SetCharacteristics(0, 0, 0);
	if (nPwmL == 0) {
		motor_left.SetPwm(0);
	} else if (nPwmL > 0) {
		motor_left.SetPwm(nPwmL + INITIAL_PWM_M0);
	} else {
		motor_left.SetPwm(nPwmL - INITIAL_PWM_M0);
	}
	if (nPwmR == 0) {
		motor_right.SetPwm(0);
	} else if (nPwmR > 0) {
		motor_right.SetPwm(nPwmR + INITIAL_PWM_M1);
	} else {
		motor_right.SetPwm(nPwmR - INITIAL_PWM_M1);
	}
#endif
#endif

	balancing_print();
}

void balancing_inc_kp(void)
{
	double Kp, Ki, Kd;

	Kp = gPidSpeed.GetKp();
	Ki = gPidSpeed.GetKi();
	Kd = gPidSpeed.GetKd();
	Kp += 0.01;
	gPidSpeed.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_dec_kp(void)
{
	double Kp, Ki, Kd;

	Kp = gPidSpeed.GetKp();
	Ki = gPidSpeed.GetKi();
	Kd = gPidSpeed.GetKd();
	Kp -= 0.01;
	gPidSpeed.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_inc_ki(void)
{
	double Kp, Ki, Kd;

	Kp = gPidSpeed.GetKp();
	Ki = gPidSpeed.GetKi();
	Kd = gPidSpeed.GetKd();
	Ki += 0.01;
	gPidSpeed.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_dec_ki(void)
{
	double Kp, Ki, Kd;

	Kp = gPidSpeed.GetKp();
	Ki = gPidSpeed.GetKi();
	Kd = gPidSpeed.GetKd();
	Ki -= 0.01;
	gPidSpeed.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_inc_kd(void)
{
	double Kp, Ki, Kd;

	Kp = gPidSpeed.GetKp();
	Ki = gPidSpeed.GetKi();
	Kd = gPidSpeed.GetKd();
	Kd += 0.01;
	gPidSpeed.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_dec_kd(void)
{
	double Kp, Ki, Kd;

	Kp = gPidSpeed.GetKp();
	Ki = gPidSpeed.GetKi();
	Kd = gPidSpeed.GetKd();
	Kd -= 0.01;
	gPidSpeed.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_inc_angle_kp(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Kp += 0.1;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_dec_angle_kp(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Kp -= 0.1;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_inc_angle_ki(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Ki += 0.01;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_dec_angle_ki(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Ki -= 0.01;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_inc_angle_kd(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Kd += 0.01;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_dec_angle_kd(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Kd -= 0.01;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_inc_tgt(void)
{
	fTgtSpeed += 1;
	balancing_print();
}

void balancing_dec_tgt(void)
{
	fTgtSpeed -= 1;
	balancing_print();
}

void balancing_inc_dir(void)
{
	nDir += 1;
	balancing_print();
}

void balancing_dec_dir(void)
{
	nDir -= 1;
	balancing_print();
}

void balancing_reset_tgtdir(void)
{
	nDir = 0;
	fTgtSpeed = 0;
	balancing_print();
}

