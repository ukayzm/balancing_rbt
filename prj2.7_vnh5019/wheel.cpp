#include "Arduino.h"
#include "wheel.h"


Wheel::Wheel(int16_t *counter, unsigned long *total_counter, int pwm_pin, int ctrl0_pin, int ctrl1_pin, uint16_t init_pwm, uint16_t min_pwm, float min_speed)
{
	pCounter = counter;
	pulTotalCounter = total_counter;
}

Wheel::SetPIN(int pwm_pin, int ctrl0_pin, int ctrl1_pin, int cs_pin)
{
	pin_pwm = pwm_pin;
	pin_ctrl0 = ctrl0_pin;
	pin_ctrl1 = ctrl1_pin;
	pin_cs = cs_pin;

	pinMode(pin_pwm, OUTPUT);
	pinMode(pin_ctrl0, OUTPUT);
	pinMode(pin_ctrl1, OUTPUT);
	pinMode(pin_cs, INPUT);
}

Wheel::SetCharacteristics(uint16_t init_pwm, uint16_t min_pwm, float min_speed)
{
	unInitPwm = init_pwm;
	unMinPwm = min_pwm;
	fMinSpeed = min_speed;
}

void Wheel::SetPwm(int16_t pwm)
{
	if (pwm > 255)
		pwm = 255;
	if (pwm < -255)
		pwm = -255;
	nCurPwm = pwm;

	setMotorDir(pwm);
	setMotorPwm(pwm);
}

void Wheel::setMotorDir(int16_t pwm)
{
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
}

void Wheel::setMotorPwm(int16_t pwm)
{
	if (pwm < 0)
		pwm = -pwm;
#if 0
	if (abs(fCurSpeed) < fMinSpeed) {
		pwm = unInitPwm;
	}
#endif
	analogWrite(pin_pwm, pwm);
}

void Wheel::GetCurrent(void)
{
	// 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
	return analogRead(pin_cs) * 34;
}

void Wheel::Update(void)
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
	aulInterval[0] = cur_us - ulLastUpdateUs;
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

	setMotorPwm(nCurPwm);

	ulLastUpdateUs = cur_us;
	nAccIntr += anIntr[0];

	/* diagnostic print */
	if (bDiag) {
		if (ulTotalIntr != curTotalIntr || nCurPwm) {
			Print();
		}
	}

	ulTotalIntr = curTotalIntr;
}

void Wheel::Print(void)
{
	int i;

	Serial.print(ulLastUpdateUs/1000); Serial.print(" msec");
	Serial.print("\tIntr "); Serial.print(anIntr[0]);
	Serial.print(" "); Serial.print(nAccIntr);
	Serial.print("\tPWM "); Serial.print(nCurPwm); Serial.print("     ");
	Serial.print("\tSpeed "); Serial.print(fCurSpeed, 2); Serial.print(" mm/s");
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

float Wheel::GetMinSpeed(void)
{
	return fMinSpeed;
}

uint16_t Wheel::GetInitPwm(void)
{
	return unInitPwm;
}

uint16_t Wheel::GetMinPwm(void)
{
	return unMinPwm;
}
