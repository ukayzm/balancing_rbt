#include "Arduino.h"
#include "wheel.h"


#define M3_INTR_PIN 2
#define M3_DIR_PIN  A0
#define M4_INTR_PIN 3
#define M4_DIR_PIN  A1


int16_t count_m3, count_m4;
unsigned long total_count_m3, total_count_m4;

void on_intr_m3(void)
{
	if (digitalRead(M3_DIR_PIN))
		count_m3++;
	else
		count_m3--;
	total_count_m3++;
}

void on_intr_m4(void)
{
	if (digitalRead(M4_DIR_PIN))
		count_m4--;
	else
		count_m4++;
	total_count_m4++;
}

void init_encoders(void)
{
	pinMode(M3_INTR_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(M3_INTR_PIN), on_intr_m3, FALLING);
	pinMode(M3_DIR_PIN, INPUT_PULLUP);
	count_m3 = 0;

	pinMode(M4_INTR_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(M4_INTR_PIN), on_intr_m4, FALLING);
	pinMode(M4_DIR_PIN, INPUT_PULLUP);
	count_m4 = 0;
}

Wheel::Wheel(int16_t *counter, unsigned long *total_counter)
{
	pCounter = counter;
	pulTotalCounter = total_counter;
}

void Wheel::AttachMotor(AF_DCMotor *motor)
{
	pMotor = motor;
	setMotorPwm(0);
}

void Wheel::SetPwm(int16_t pwm)
{
	setMotorPwm(pwm);
}

void Wheel::Loop(void)
{
	unsigned long cur_us = micros();
	unsigned long curTotalIntr;
	int i, num_intr;

	for (i = NUM_INTR_SAVE - 1; i > 0; i--) {
		anIntr[i] = anIntr[i-1];
		aulInterval[i] = aulInterval[i-1];
	}
	anIntr[0] = *pCounter;
	*pCounter = 0;
	aulInterval[0] = cur_us - ulLastLoopUs;
	curTotalIntr = *pulTotalCounter;

	/* calculate current speed of wheel */
	num_intr = 0;
	ulIntervalUs = 0;
	for (i = 0; i < NUM_INTR_SAVE; i++) {
		num_intr += anIntr[i];
		ulIntervalUs += aulInterval[i];
	}
	ulIntervalUs /= NUM_INTR_SAVE;
	if (ulIntervalUs && num_intr) {
		fCurSpeed = (float)(num_intr * 1000.0 * MM_PER_INTR) / NUM_INTR_SAVE / (float)ulIntervalUs * 1000.0;
	} else {
		fCurSpeed = 0;
	}

	ulLastLoopUs = cur_us;
	nAccIntr += anIntr[0];

	/* diagnostic print */
	if (bDiag) {
		if (ulTotalIntr != curTotalIntr || nCurPwm) {
			Print();
		}
	}

	ulTotalIntr = curTotalIntr;
}

void Wheel::setMotorPwm(int16_t pwm)
{
	if (pwm > 255)
		pwm = 255;
	if (pwm < -255)
		pwm = -255;
	/* dumb speed up */
	if (pwm > 0) {
		pMotor->run(FORWARD);
	} else if (pwm < 0) {
		pMotor->run(BACKWARD);
	} else {
		pMotor->run(RELEASE);
	}
	nCurPwm = pwm;
	if (pwm < 0)
		pwm = -pwm;
	pMotor->setSpeed(pwm);
}

void Wheel::Print(void)
{
	int i;

	Serial.print(ulLastLoopUs/1000); Serial.print(" ms");
	Serial.print(" Intr "); Serial.print(anIntr[0]);
	Serial.print(" "); Serial.print(nAccIntr);
	Serial.print(" Speed "); Serial.print(fCurSpeed, 2); Serial.print(" mm/s");
	Serial.print(" Out "); Serial.print(nCurPwm);
	Serial.println("");
}

int16_t Wheel::GetCurPwm(void)
{
	return nCurPwm;
}

int32_t Wheel::GetAccIntr(void)
{
	return nAccIntr;
}

void Wheel::ResetAccIntr(void)
{
	nAccIntr = 0;
}

float Wheel::GetCurSpeed(void)
{
	return fCurSpeed;
}
