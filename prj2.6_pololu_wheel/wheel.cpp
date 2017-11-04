#include "Arduino.h"
#include "wheel.h"


Wheel::Wheel(int16_t *counter, unsigned long *total_counter, int pwm_pin, int ctrl0_pin, int ctrl1_pin, uint16_t init_pwm, uint16_t min_pwm)
{
	pCounter = counter;
	pulTotalCounter = total_counter;
	unInitPwm = init_pwm;
	unMinPwm = min_pwm;

	pin_pwm = pwm_pin;
	pin_ctrl0 = ctrl0_pin;
	pin_ctrl1 = ctrl1_pin;

	pinMode(pin_pwm, OUTPUT);
	pinMode(pin_ctrl0, OUTPUT);
	pinMode(pin_ctrl1, OUTPUT);
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
		digitalWrite(pin_ctrl0, LOW);
		digitalWrite(pin_ctrl1, HIGH);
	} else if (pwm < 0) {
		digitalWrite(pin_ctrl1, LOW);
		digitalWrite(pin_ctrl0, HIGH);
	} else {
		digitalWrite(pin_ctrl0, LOW);
		digitalWrite(pin_ctrl1, LOW);
	}
	nCurPwm = pwm;
	if (pwm < 0)
		pwm = -pwm;
	analogWrite(pin_pwm, pwm);
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

uint16_t Wheel::GetInitPwm(void)
{
	return unInitPwm;
}

uint16_t Wheel::GetMinPwm(void)
{
	return unMinPwm;
}
