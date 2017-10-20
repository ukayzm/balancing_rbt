#include <Arduino.h>
#include <AFMotor.h>
#include <PID_v1.h>
#include "wheel.h"
#include "mpu6050.h"

AF_DCMotor m3(3, MOTOR34_1KHZ);
AF_DCMotor m4(4, MOTOR34_1KHZ);

Wheel wheel_left(&count_m3, &total_count_m3);
Wheel wheel_right(&count_m4, &total_count_m4);

#define SPEED_KP		0.10
#define SPEED_KI		0.02
#define SPEED_KD		0.0

#define ANGLE_KP	50.0
#define ANGLE_KI	0.0
#define ANGLE_KD	0.30

double fTgtSpeed, fInSpeed;
double fTgtAngle, fInAngle, fPwm;
int32_t nDir;

PID gPidSpeed(&fInSpeed, &fTgtAngle, &fTgtSpeed, SPEED_KP, SPEED_KI, SPEED_KD, REVERSE);
PID gPidAngle(&fInAngle, &fPwm, &fTgtAngle, ANGLE_KP, ANGLE_KI, ANGLE_KD, DIRECT);

void balancing_print(void)
{
	Serial.print(millis()); Serial.print(" ms");
	Serial.print(" Kspeed "); Serial.print(gPidSpeed.GetKp(), 2);
	Serial.print(" "); Serial.print(gPidSpeed.GetKi(), 2);
	Serial.print(" "); Serial.print(gPidSpeed.GetKd(), 2);
	Serial.print(" Kangle "); Serial.print(gPidAngle.GetKp(), 2);
	Serial.print(" "); Serial.print(gPidAngle.GetKi(), 2);
	Serial.print(" "); Serial.print(gPidAngle.GetKd(), 2);
	Serial.print(" TgtSpeed "); Serial.print(fTgtSpeed, 2); Serial.print(" deg");
	Serial.print(" TgtAngle "); Serial.print(fTgtAngle, 2); Serial.print(" deg");
	Serial.print(" CurAngle "); Serial.print(fInAngle, 2); Serial.print(" deg");
	Serial.print(" PWM "); Serial.print(fPwm, 2);
	Serial.print(" AccIntr "); Serial.print(wheel_left.GetAccIntr());
	Serial.print(" "); Serial.print(wheel_right.GetAccIntr());
	Serial.print(" "); Serial.print(wheel_left.GetCurSpeed(), 2);
	Serial.print(" "); Serial.print(wheel_right.GetCurSpeed(), 2);
	Serial.println(" mm/s");
}

void balancing_setup()
{
	init_encoders();
	mpu6050_setup();

	wheel_left.AttachMotor(&m3);
	wheel_right.AttachMotor(&m4);

	gPidSpeed.SetSampleTime(10);
	gPidSpeed.SetMode(AUTOMATIC);
	gPidSpeed.SetOutputLimits(-10, 10);

	gPidAngle.SetSampleTime(10);
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

	count_l = wheel_left.GetAccIntr();
	wheel_left.ResetAccIntr();
	count_r = wheel_right.GetAccIntr();
	wheel_right.ResetAccIntr();

	fInSpeed = count_l + count_r;	/* current speed * a */
	gPidSpeed.Compute();

	fInAngle = angle_pitch;
	gPidAngle.Compute();
}

void balancing_loop()
{
	float angle_pitch;
	int nPwmL, nPwmR;

	mpu6050_loop();

	angle_pitch = get_pitch_angle();

	if (angle_pitch < -20 || 20 < angle_pitch) {
		wheel_left.SetPwm(0);
		wheel_right.SetPwm(0);

		wheel_left.Loop();
		wheel_right.Loop();

		return;
	}

	compute_pid(angle_pitch);

	nPwmL = fPwm + nDir;
	nPwmR = fPwm - nDir;

	if (nPwmL == 0) {
		wheel_left.SetPwm(0);
	} else if (nPwmL > 0) {
		wheel_left.SetPwm(nPwmL + MIN_PWM);
	} else {
		wheel_left.SetPwm(nPwmL - MIN_PWM);
	}
	if (nPwmR == 0) {
		wheel_right.SetPwm(0);
	} else if (nPwmR > 0) {
		wheel_right.SetPwm(nPwmR + MIN_PWM);
	} else {
		wheel_right.SetPwm(nPwmR - MIN_PWM);
	}

	wheel_left.Loop();
	wheel_right.Loop();

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
	Kp += 1;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_dec_angle_kp(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Kp -= 1;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_inc_angle_ki(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Ki += 0.1;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_dec_angle_ki(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Ki -= 0.1;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_inc_angle_kd(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Kd += 0.1;
	gPidAngle.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_dec_angle_kd(void)
{
	double Kp, Ki, Kd;

	Kp = gPidAngle.GetKp();
	Ki = gPidAngle.GetKi();
	Kd = gPidAngle.GetKd();
	Kd -= 0.1;
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

