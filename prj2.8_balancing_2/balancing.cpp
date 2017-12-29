#include <Arduino.h>
#include "pid_v2.h"
#include "board.h"
#include "mpu6050.h"
#include "motor.h"


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
#define ANGLE_KP	70.0
#define ANGLE_KI	5.0
#define ANGLE_KD	0.0
#endif

double fTgtSpeed, fInSpeed;
double fTgtAngle, fInAngle, fPwm;
int32_t nDir;

PID gPidSpeed(&fInSpeed, &fTgtAngle, &fTgtSpeed, SPEED_KP, SPEED_KI, SPEED_KD, REVERSE);
PID gPidAngle(&fInAngle, &fPwm, &fTgtAngle, ANGLE_KP, ANGLE_KI, ANGLE_KD, DIRECT);

void balancing_print_k(void)
{
	Serial.print("K,"); Serial.print(fTgtSpeed, 2);
	Serial.print(",\t"); Serial.print(fTgtAngle, 2);
	Serial.print(",\t"); Serial.print(gPidSpeed.GetKp(), 1);
	Serial.print(",\t"); Serial.print(gPidSpeed.GetKi(), 1);
	Serial.print(",\t"); Serial.print(gPidSpeed.GetKd(), 1);
	Serial.print(",\t"); Serial.print(gPidAngle.GetKp(), 1);
	Serial.print(",\t"); Serial.print(gPidAngle.GetKi(), 1);
	Serial.print(",\t"); Serial.print(gPidAngle.GetKd(), 1);
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
	Serial.print(",\t"); Serial.print(fPwm, 0);
	Serial.print(",\t"); Serial.print(motor_left.GetCurPwm());
	Serial.print(",\t"); Serial.print(motor_right.GetCurPwm());
	//Serial.print(",\t"); Serial.print(motor_left.GetAccIntr());
	//Serial.print(",\t"); Serial.print(motor_right.GetAccIntr());
	Serial.print(",\t"); Serial.print(motor_left.GetCurRpm());
	Serial.print(",\t"); Serial.print(motor_right.GetCurRpm());
	Serial.print(",\t"); Serial.print(motor_left.GetCurrent());
	Serial.print(",\t"); Serial.print(motor_right.GetCurrent());
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

#if 0
	fTgtAngle = 0;
#endif
	fInAngle = angle_pitch;
	gPidAngle.Compute();
}

float last_error = 0;
float errSum = 0;
unsigned long lastTime;
#define GUARD_GAIN	100

void compute_pid_Kas(float angle_pitch)
{
	float Kp = gPidAngle.GetKp();
	float Ki = gPidAngle.GetKi();
	float Kd = gPidAngle.GetKd();
	unsigned long now = millis();
	unsigned long dt = (now - lastTime);
	float error;
	float pTerm, iTerm, dTerm;
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
	float dErr = (error - last_error) / dt;

	errSum += error * dt;
	pTerm = Kp * error;
	iTerm = Ki * constrain(errSum, -GUARD_GAIN, GUARD_GAIN);
	dTerm = Kd * dErr;

	last_error = error;
	lastTime = now;

	fPwm = constrain((pTerm + iTerm + dTerm), -255, 255);
}

float last_input;

void compute_pid_adr(float angle_pitch)
{
	float Kp = gPidAngle.GetKp();
	float Ki = gPidAngle.GetKi();
	float Kd = gPidAngle.GetKd();
	unsigned long now = millis();
	unsigned long dt = (now - lastTime);
	float error;
	float pTerm, iTerm, dTerm;

	fTgtAngle = 0.0;

	error = fTgtAngle - angle_pitch;

	pTerm = Kp * error;
	errSum += Ki * error * dt;
	iTerm = constrain(errSum, -GUARD_GAIN, GUARD_GAIN);
	float dInput = angle_pitch - last_input;
	dTerm = -(Kd * dInput / dt);

	last_input = angle_pitch;
	last_error = error;
	lastTime = now;

	fPwm = constrain((pTerm + iTerm + dTerm), -255, 255);
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

	//compute_pid(angle_pitch);
	//compute_pid_Kas(angle_pitch);
	compute_pid_adr(angle_pitch);

	//fPwm = fPwm * abs(fPwm);

	nPwmL = fPwm + nDir;
	nPwmR = fPwm - nDir;

#if 1
#if 1
#define SHRINK_TOO_LOW_PWM	5
	if (abs(nPwmL) <= SHRINK_TOO_LOW_PWM) {
		motor_left.SetPwm(0);
	} else if (nPwmL > 0) {
		motor_left.SetPwm(nPwmL);
	} else {
		motor_left.SetPwm(nPwmL);
	}
	if (abs(nPwmR) <= SHRINK_TOO_LOW_PWM) {
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
	balancing_print_k();
}

void balancing_dec_kp(void)
{
	double Kp, Ki, Kd;

	Kp = gPidSpeed.GetKp();
	Ki = gPidSpeed.GetKi();
	Kd = gPidSpeed.GetKd();
	Kp -= 0.01;
	gPidSpeed.SetTunings(Kp, Ki, Kd);
	balancing_print_k();
}

void balancing_inc_ki(void)
{
	double Kp, Ki, Kd;

	Kp = gPidSpeed.GetKp();
	Ki = gPidSpeed.GetKi();
	Kd = gPidSpeed.GetKd();
	Ki += 0.01;
	gPidSpeed.SetTunings(Kp, Ki, Kd);
	balancing_print_k();
}

void balancing_dec_ki(void)
{
	double Kp, Ki, Kd;

	Kp = gPidSpeed.GetKp();
	Ki = gPidSpeed.GetKi();
	Kd = gPidSpeed.GetKd();
	Ki -= 0.01;
	gPidSpeed.SetTunings(Kp, Ki, Kd);
	balancing_print_k();
}

void balancing_inc_kd(void)
{
	double Kp, Ki, Kd;

	Kp = gPidSpeed.GetKp();
	Ki = gPidSpeed.GetKi();
	Kd = gPidSpeed.GetKd();
	Kd += 0.01;
	gPidSpeed.SetTunings(Kp, Ki, Kd);
	balancing_print_k();
}

void balancing_dec_kd(void)
{
	double Kp, Ki, Kd;

	Kp = gPidSpeed.GetKp();
	Ki = gPidSpeed.GetKi();
	Kd = gPidSpeed.GetKd();
	Kd -= 0.01;
	gPidSpeed.SetTunings(Kp, Ki, Kd);
	balancing_print_k();
}

void balancing_inc_angle_kp(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Kp += 1;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print_k();
}

void balancing_dec_angle_kp(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Kp -= 1;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print_k();
}

void balancing_inc_angle_ki(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Ki += 0.1;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print_k();
}

void balancing_dec_angle_ki(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Ki -= 0.1;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print_k();
}

void balancing_inc_angle_kd(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Kd += 0.1;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print_k();
}

void balancing_dec_angle_kd(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Kd -= 0.1;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print_k();
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

