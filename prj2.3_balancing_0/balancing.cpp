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
int32_t nDir;

#define PID_KAS

#if defined(PID_KAS)
#define WHEEL_KP		2.0
#define WHEEL_KI		0.0
#define WHEEL_KD		0.0
float Kp_wheel, Ki_wheel, Kd_wheel;
#endif

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
#if defined(PID_KAS)
	Serial.print(" Kw "); Serial.print(Kp_wheel, 2);
	Serial.print(" "); Serial.print(Ki_wheel, 2);
	Serial.print(" "); Serial.print(Kd_wheel, 2);
#endif
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

#if defined(PID_KAS)
	Kp_wheel = WHEEL_KP;
	Ki_wheel = WHEEL_KI;
	Kd_wheel = WHEEL_KD;
#endif
}

void compute_pid_basic(float angle_pitch)
{
	auto_calibration(angle_pitch);

	fInput = angle_pitch - fCalibAngle;
	gBalancingPid.Compute();
}

#if defined(PID_KAS)
void compute_pid_kas(float angle_pitch)
{
	int32_t count_l, count_r;
	static int32_t last_count_l, last_count_r;

	fInput = angle_pitch - fCalibAngle;
	gBalancingPid.Compute();

	count_l = wheel_left.GetAccIntr();
	wheel_left.ResetAccIntr();
	count_r = wheel_right.GetAccIntr();
	wheel_right.ResetAccIntr();

	fOutput += Kp_wheel * (count_l + count_r) + Kd_wheel * (count_l + count_r - (last_count_r + last_count_l));

	last_count_l = count_l;
	last_count_r = count_r;
}
#endif

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

#if defined(PID_KAS)
	compute_pid_kas(angle_pitch);
#else
	compute_pid_basic(angle_pitch);
#endif

	nPwmL = fOutput + nDir;
	nPwmR = fOutput - nDir;

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

void balancing_inc_cal(void)
{
	fCalibAngle += 0.1;
	balancing_print();
}

void balancing_dec_cal(void)
{
	fCalibAngle -= 0.1;
	balancing_print();
}

void balancing_inc_tgt(void)
{
	fTarget += 1;
	balancing_print();
}

void balancing_dec_tgt(void)
{
	fTarget -= 1;
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
	fTarget = 0;
	balancing_print();
}

void balancing_inc_wheel_kp(void)
{
#if defined(PID_KAS)
	Kp_wheel += 0.1;
	balancing_print();
#endif
}

void balancing_dec_wheel_kp(void)
{
#if defined(PID_KAS)
	Kp_wheel -= 0.1;
	balancing_print();
#endif
}

