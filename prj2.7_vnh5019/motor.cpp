#include "Arduino.h"
#include "motor.h"


Motor::Motor(int16_t *counter, unsigned long *total_counter)
{
	pCounter = counter;
	pulTotalCounter = total_counter;
}

void Motor::SetPIN(int pwm_pin, int ctrl0_pin, int ctrl1_pin, int cs_pin)
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

void Motor::SetCharacteristics(uint16_t init_pwm, uint16_t min_pwm, uint16_t min_rpm)
{
	unInitPwm = init_pwm;
	unMinPwm = min_pwm;
	unMinRpm = min_rpm;
}

void Motor::SetPwm(int16_t pwm)
{
	if (pwm > 255)
		pwm = 255;
	if (pwm < -255)
		pwm = -255;
	nTgtPwm = pwm;

	setMotorDir(pwm);
	pwm = calibratePwmAtLowSpeed(pwm);
	setMotorPwm(pwm);
}

void Motor::setMotorDir(int16_t pwm)
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

int Motor::calibratePwmAtLowSpeed(int pwm)
{
	if (pwm != 0 && unMinRpm > 0 && abs(pwm) < unInitPwm) {
		if (abs(nCurRpm) < unMinRpm) {
#if defined(MOTOR_PWM_MIDIFICATION_DEBUG)
Serial.print("(1) ");
Serial.print(pwm);
#endif
			if (pwm > 0) {
				pwm = unInitPwm;
			} else {
				pwm = -unInitPwm;
			}
#if defined(MOTOR_PWM_MIDIFICATION_DEBUG)
Serial.print(" -> ");
Serial.println(pwm);
#endif
		} else if (nCurRpm > 0 && 0 > pwm) {
#if defined(MOTOR_PWM_MIDIFICATION_DEBUG)
Serial.print("(2) ");
Serial.print(pwm);
#endif
			pwm = -unInitPwm;
#if defined(MOTOR_PWM_MIDIFICATION_DEBUG)
Serial.print(" -> ");
Serial.println(pwm);
#endif
		} else if (nCurRpm < 0 && 0 < pwm) {
#if defined(MOTOR_PWM_MIDIFICATION_DEBUG)
Serial.print("(3) ");
Serial.print(pwm);
#endif
			pwm = unInitPwm;
#if defined(MOTOR_PWM_MIDIFICATION_DEBUG)
Serial.print(" -> ");
Serial.println(pwm);
#endif
		}
	}
	return pwm;
}

//#define MOTOR_PWM_MIDIFICATION_DEBUG
void Motor::setMotorPwm(int16_t pwm)
{
	nCurPwm = pwm;
	if (pwm >= 0) {
		analogWrite(pin_pwm, pwm);
	} else {
		analogWrite(pin_pwm, -pwm);
	}
}

uint16_t Motor::GetCurrent(void)
{
	// 5V / 1024 ADC counts / 144 mV per A = 34 mA per count
	return analogRead(pin_cs) * 34;
}

void Motor::Update(void)
{
	unsigned long cur_us = micros();
	unsigned long curTotalIntr;
	long num_intr;
	int i;

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
	if (ulIntervalUs && num_intr) {
		nCurRpm = num_intr * 60 * 1000 / INTR_PER_REV * 1000 / (long)ulIntervalUs;
	} else {
		nCurRpm = 0;
	}

#if 0
	setMotorPwm(calibratePwmAtLowSpeed(nTgtPwm));
#else
	setMotorPwm(nTgtPwm);
#endif

	ulLastUpdateUs = cur_us;
	nAccIntr += anIntr[0];

	/* diagnostic print */
	if (bDiag) {
		if (ulTotalIntr != curTotalIntr || nTgtPwm) {
			Print();
		}
	}

	ulTotalIntr = curTotalIntr;
}

void Motor::Print(void)
{
	int i;

	Serial.print("intr "); Serial.print(anIntr[0]);
	Serial.print(" "); Serial.print(nAccIntr);
	Serial.print("\tPWM "); Serial.print(nCurPwm); Serial.print("     ");
	Serial.print("\tRPM "); Serial.print(nCurRpm);
	Serial.println("");
}

int16_t Motor::GetCurPwm(void)
{
	return nCurPwm;
}

int16_t Motor::GetTgtPwm(void)
{
	return nTgtPwm;
}

int32_t Motor::GetAccIntr(void)
{
	return nAccIntr;
}

void Motor::ResetAccIntr(void)
{
	nAccIntr = 0;
}

int16_t Motor::GetCurRpm(void)
{
	return nCurRpm;
}

uint16_t Motor::GetMinRpm(void)
{
	return unMinRpm;
}

uint16_t Motor::GetInitPwm(void)
{
	return unInitPwm;
}

uint16_t Motor::GetMinPwm(void)
{
	return unMinPwm;
}
