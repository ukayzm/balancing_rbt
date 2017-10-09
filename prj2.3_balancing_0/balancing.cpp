#include <Arduino.h>
#include <AFMotor.h>
#include <PID_v1.h>
#include "wheel.h"
#include "mpu6050.h"

AF_DCMotor m3(3, MOTOR34_1KHZ);
AF_DCMotor m4(4, MOTOR34_1KHZ);

Wheel wheel_left(&count_m3, &total_count_m3);
Wheel wheel_right(&count_m4, &total_count_m4);

double fInput, fOutput, fTarget;
double fCalibAngle, fCalibAngleSum;
int nNumSum;
int32_t nMinAccIntr, nMaxAccIntr;

#define BALANCING_KP	50.0
#define BALANCING_KI	10.0
#define BALANCING_KD	0.0
#define INITIAL_TARGET	0

PID gBalancingPid(&fInput, &fOutput, &fTarget, BALANCING_KP, BALANCING_KI, BALANCING_KD, DIRECT);

void balancing_print(void)
{
	Serial.print(millis()); Serial.print(" ms");
	Serial.print(" AccIntr "); Serial.print(wheel_left.GetAccIntr());
	Serial.print(" "); Serial.print(wheel_right.GetAccIntr());
	Serial.print(" K "); Serial.print(gBalancingPid.GetKp(), 2);
	Serial.print(" "); Serial.print(gBalancingPid.GetKi(), 2);
	Serial.print(" "); Serial.print(gBalancingPid.GetKd(), 2);
	Serial.print(" Cal "); Serial.print(fCalibAngle, 2); Serial.print(" deg");
	Serial.print(" Cur "); Serial.print(fInput, 2); Serial.print(" deg");
	Serial.print(" Out "); Serial.print(fOutput, 2); Serial.print(" PWM ");
	Serial.print(" "); Serial.print(wheel_left.GetCurSpeed(), 2);
	Serial.print(" "); Serial.print(wheel_right.GetCurSpeed(), 2);
	Serial.println(" mm/s");
}

void reset_calibration(void)
{
	nNumSum = 0;
	fCalibAngleSum = 0;
	nMinAccIntr = 0;
	nMaxAccIntr = 0;
	wheel_left.ResetAccIntr();
	wheel_right.ResetAccIntr();
	Serial.println(__FUNCTION__);
}

void auto_calibration(float angle_pitch)
{
	int32_t nAccIntr;

	nAccIntr = wheel_left.GetAccIntr() + wheel_right.GetAccIntr();
	if (nAccIntr < nMinAccIntr)
		nMinAccIntr = nAccIntr;
	if (nAccIntr > nMaxAccIntr)
		nMaxAccIntr = nAccIntr;

	fCalibAngleSum += angle_pitch;
	nNumSum++;

	if (nMinAccIntr < nAccIntr
	 && -1 < nAccIntr
	 && nAccIntr < 1
	 && nAccIntr < nMaxAccIntr) {
		fCalibAngle = fCalibAngleSum / nNumSum;
		reset_calibration();
	}
}

void balancing_setup()
{
	init_encoders();
	mpu6050_setup();

	wheel_left.AttachMotor(&m3);
	wheel_right.AttachMotor(&m4);

	gBalancingPid.SetSampleTime(10);
	gBalancingPid.SetMode(AUTOMATIC);
	gBalancingPid.SetOutputLimits(-MAX_MMPS, MAX_MMPS);
	
	fTarget = INITIAL_TARGET;
}

void balancing_loop()
{
	float angle_pitch;

	mpu6050_loop();

	angle_pitch = get_pitch_angle();

	if (angle_pitch < -20 || 20 < angle_pitch) {
		wheel_left.SetPwm(0);
		wheel_right.SetPwm(0);

		wheel_left.Loop();
		wheel_right.Loop();

		return;
	}

	auto_calibration(angle_pitch);

	fInput = angle_pitch - fCalibAngle;
	gBalancingPid.Compute();

	if (fOutput == 0) {
		wheel_left.SetPwm(fOutput);
		wheel_right.SetPwm(fOutput);
	} else if (fOutput > 0) {
		wheel_left.SetPwm(fOutput + MIN_PWM);
		wheel_right.SetPwm(fOutput + MIN_PWM);
	} else {
		wheel_left.SetPwm(fOutput - MIN_PWM);
		wheel_right.SetPwm(fOutput - MIN_PWM);
	}

	wheel_left.Loop();
	wheel_right.Loop();

	balancing_print();
}

void balancing_inc_kp(void)
{
	double Kp, Ki, Kd;

	Kp = gBalancingPid.GetKp();
	Ki = gBalancingPid.GetKi();
	Kd = gBalancingPid.GetKd();
	Kp += 1;
	gBalancingPid.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_inc_ki(void)
{
	double Kp, Ki, Kd;

	Kp = gBalancingPid.GetKp();
	Ki = gBalancingPid.GetKi();
	Kd = gBalancingPid.GetKd();
	Ki += 1;
	gBalancingPid.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_inc_kd(void)
{
	double Kp, Ki, Kd;

	Kp = gBalancingPid.GetKp();
	Ki = gBalancingPid.GetKi();
	Kd = gBalancingPid.GetKd();
	Kd += 0.1;
	gBalancingPid.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_dec_kp(void)
{
	double Kp, Ki, Kd;

	Kp = gBalancingPid.GetKp();
	Ki = gBalancingPid.GetKi();
	Kd = gBalancingPid.GetKd();
	Kp -= 1;
	gBalancingPid.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_dec_ki(void)
{
	double Kp, Ki, Kd;

	Kp = gBalancingPid.GetKp();
	Ki = gBalancingPid.GetKi();
	Kd = gBalancingPid.GetKd();
	Ki -= 1;
	gBalancingPid.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_dec_kd(void)
{
	double Kp, Ki, Kd;

	Kp = gBalancingPid.GetKp();
	Ki = gBalancingPid.GetKi();
	Kd = gBalancingPid.GetKd();
	Kd -= 0.1;
	gBalancingPid.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_inc_tgt(void)
{
	fCalibAngle += 0.1;
	balancing_print();
}

void balancing_dec_tgt(void)
{
	fCalibAngle -= 0.1;
	balancing_print();
}
