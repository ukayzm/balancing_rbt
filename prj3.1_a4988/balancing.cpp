#include <Arduino.h>
#include "board.h"
#include "mpu6050.h"


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
#define ANGLE_KP	30.0
#define ANGLE_KI	0.0
#define ANGLE_KD	0.0
#endif

double fAngleKp = ANGLE_KP;
double fAngleKi = ANGLE_KI;
double fAngleKd = ANGLE_KD;
double fSpeedKp = SPEED_KP;
double fSpeedKi = SPEED_KI;
double fSpeedKd = SPEED_KD;
double fTgtSpeed, fInSpeed;
double fTgtAngle, fInAngle, fPwm;
double pTerm, iTerm, dTerm;
float errSum = 0;
int32_t nDir;

void balancing_print_k(void)
{
	Serial.print("K,"); Serial.print(fTgtSpeed, 2);
	Serial.print(",\t"); Serial.print(fTgtAngle, 2);
	Serial.print(",\t"); Serial.print(fSpeedKp, 1);
	Serial.print(",\t"); Serial.print(fSpeedKi, 1);
	Serial.print(",\t"); Serial.print(fSpeedKd, 1);
	Serial.print(",\t"); Serial.print(fAngleKp, 1);
	Serial.print(",\t"); Serial.print(fAngleKi, 1);
	Serial.print(",\t"); Serial.print(fAngleKd, 1);
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
	Serial.print(",\t"); Serial.print(get_pitch_angle(), 2);
	Serial.print(",\t"); Serial.print(pTerm, 2);
	Serial.print(",\t"); Serial.print(errSum, 2);
	Serial.print(",\t"); Serial.print(iTerm, 2);
	Serial.print(",\t"); Serial.print(dTerm, 2);
	Serial.print(",\t"); Serial.print(fPwm, 0);
	Serial.println("");
	last_ms = cur_ms;
}

void balancing_setup()
{
	gPidSpeed.SetSampleTime(INTERVAL_BALANCING);
	gPidSpeed.SetMode(AUTOMATIC);
	gPidSpeed.SetOutputLimits(-10, 10);

	gPidAngle.SetSampleTime(INTERVAL_BALANCING);
	gPidAngle.SetMode(AUTOMATIC);
	gPidAngle.SetOutputLimits(-255, 255);
}

float last_error = 0;
unsigned long lastTime;
#define GUARD_GAIN	30

float last_input;

float compute_pid_adr(float angle_pitch)
{
	float Kp = gPidAngle.GetKp();
	float Ki = gPidAngle.GetKi();
	float Kd = gPidAngle.GetKd();
	unsigned long now = millis();
	float dt = (float)(now - lastTime) / 1000.0;
	float error;

	error = fTgtAngle - angle_pitch;

	pTerm = Kp * error;

	errSum += Ki * error * dt;
	iTerm = constrain(errSum, -GUARD_GAIN, GUARD_GAIN);

	float dInput = angle_pitch - last_input;
	dTerm = Kd * (-dInput / dt);

	last_input = angle_pitch;
	last_error = error;
	lastTime = now;

	return constrain((pTerm + iTerm + dTerm), -255, 255);
}


float compute_pid_Kas(float angle_pitch)
{
	float Kp = gPidAngle.GetKp();
	float Ki = gPidAngle.GetKi();
	float Kd = gPidAngle.GetKd();
	unsigned long now = millis();
	unsigned long dt = (now - lastTime);
	float error;
#if 0
	int count_l, count_r;
	int cur_count;
	float pTerm_Wheel, dTerm_Wheel;
	float Kp_wheel = gPidSpeed.GetKp();
	float Kd_wheel = gPidSpeed.GetKd();

	count_l = motor_left.GetAccIntr();
	motor_left.ResetAccIntr();
	count_r = motor_right.GetAccIntr();
	motor_right.ResetAccIntr();
	cur_count = count_l + count_r;

	pTerm_Wheel = Kp_Wheel * cur_count;           //  -(Kxp/100) * count;
	dTerm_Wheel = Kd_Wheel * (cur_count - last_count);
	last_count = cur_count;
#else
	fTgtAngle = 0.0;
#endif
	error = fTgtAngle - angle_pitch;

	pTerm = Kp * error;

	errSum += error * dt;
	iTerm = Ki * constrain(errSum, -GUARD_GAIN, GUARD_GAIN);

	float dErr = (error - last_error) / dt;
	dTerm = Kd * dErr;

	last_error = error;
	lastTime = now;

	return constrain((pTerm + iTerm + dTerm), -255, 255);
}

void balancing_loop()
{
	float angle_pitch;
	int nPwmL, nPwmR;

	angle_pitch = get_pitch_angle();

	if (angle_pitch < -30 || 30 < angle_pitch) {
		motor_set_rpm(0);

		return;
	}

	//fPwm = compute_pid(angle_pitch);
	//fPwm = compute_pid_Kas(angle_pitch);
	fPwm = compute_pid_adr(angle_pitch);

	//fPwm = fPwm * abs(fPwm);


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

