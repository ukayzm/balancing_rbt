#include <Arduino.h>
#include <AFMotor.h>
#include <PID_v1.h>
#include "wheel.h"
#include "mpu6050.h"

AF_DCMotor m3(3, MOTOR34_1KHZ);
AF_DCMotor m4(4, MOTOR34_1KHZ);

Wheel wheel_left(&count_m3, &total_count_m3, calib_data_m3);
Wheel wheel_right(&count_m4, &total_count_m4, calib_data_m4);

double fInput, fOutput, fTarget;

#define BALANCING_KP	100.0
#define BALANCING_KI	30.0
#define BALANCING_KD	0.0

PID gBalancingPid(&fInput, &fOutput, &fTarget, BALANCING_KP, BALANCING_KI, BALANCING_KD, DIRECT);

void balancing_print(void)
{
	Serial.print(millis()); Serial.print(" ms");
	Serial.print(" Intr "); Serial.print(0);
	Serial.print(" K "); Serial.print(gBalancingPid.GetKp(), 2);
	Serial.print(" "); Serial.print(gBalancingPid.GetKi(), 2);
	Serial.print(" "); Serial.print(gBalancingPid.GetKd(), 2);
	Serial.print(" Tgt "); Serial.print(fTarget, 2); Serial.print(" deg");
	Serial.print(" Cur "); Serial.print(fInput, 2); Serial.print(" deg");
	Serial.print(" Out "); Serial.print(0);
	Serial.print(" + "); Serial.print(fOutput, 2);
	Serial.print(" = mm/s "); Serial.print(fOutput, 2);
	Serial.print(" "); Serial.print(0);
	Serial.println("");
}

void balancing_setup()
{
	init_encoders();
	mpu6050_setup();

	wheel_left.AttachMotor(&m3);
	wheel_right.AttachMotor(&m4);

	//wheel_left.bDiag = 1;
	//wheel_right.bDiag = 1;

	gBalancingPid.SetSampleTime(10);
	gBalancingPid.SetMode(AUTOMATIC);
	gBalancingPid.SetOutputLimits(-MAX_MMPS, MAX_MMPS);
	
	fTarget = 0;
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

	fInput = angle_pitch;
	gBalancingPid.Compute();

	wheel_left.SetSpeed(fOutput);
	wheel_right.SetSpeed(fOutput);

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
	Ki += 0.01;
	gBalancingPid.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_inc_kd(void)
{
	double Kp, Ki, Kd;

	Kp = gBalancingPid.GetKp();
	Ki = gBalancingPid.GetKi();
	Kd = gBalancingPid.GetKd();
	Kd += 1;
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
	Kd -= 0.01;
	gBalancingPid.SetTunings(Kp, Ki, Kd);
	balancing_print();
}

void balancing_inc_tgt(void)
{
	fTarget += 0.1;
}

void balancing_dec_tgt(void)
{
	fTarget -= 0.1;
}
