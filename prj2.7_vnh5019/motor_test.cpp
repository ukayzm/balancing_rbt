#include <Arduino.h>
#include "board.h"
#include "motor_test.h"

void print_motor_test_header1(void)
{
	Serial.println("pwm_freq,pwm,intr,RPM");
}

void print_motor_test1(Motor *m, int msec)
{
	int32_t rpm;
	int32_t intr;
	
	Serial.print(",\t");
	if (m) {
		intr = m->GetAccIntr();

		Serial.print(intr); Serial.print(",\t");
		rpm = intr * 60 * 1000 / INTR_PER_REV / msec;
		Serial.print(rpm);
	} else {
		Serial.print("0,\t0");
	}
}

void measure_motor_test1(int pwm, Motor *m0, Motor *m1)
{
	int i, msec;
	uint16_t freq = getPwmFrequencyTimer1();

	Serial.print(freq);
	Serial.print(",\t");
	Serial.print(pwm);
	m0->SetPwm(pwm);
	if (m1) m1->SetPwm(pwm);
	delay(200);
	m0->Update();
	if (m1) m1->Update();
	m0->ResetAccIntr();
	if (m1) m1->ResetAccIntr();
	msec = millis();
	for (i = 0; i < 80; i++) {
		delay(10);
		m0->Update();
		if (m1) m1->Update();
	}
	msec = millis() - msec;
	print_motor_test1(m0, msec);
	if (m1) print_motor_test1(m1, msec);
	Serial.println();
}

void do_motor_test1(Motor *m0, Motor *m1)
{
	int16_t pwm;

	/* increasing */
	print_motor_test_header1();
	pwm = 0;
	m0->SetPwm(pwm);
	if (m1) m1->SetPwm(pwm);
	delay(1000);
	for (pwm = 0; pwm <= 260; pwm += 10) {
		measure_motor_test1(pwm, m0, m1);
	}

	/* decreasing */
	print_motor_test_header1();
	pwm = 260;
	m0->SetPwm(pwm);
	if (m1) m1->SetPwm(pwm);
	delay(1000);
	for (pwm = 260; pwm >= 0; pwm -= 10) {
		measure_motor_test1(pwm, m0, m1);
	}

	m0->SetPwm(0);
	if (m1) m1->SetPwm(0);
	m0->Update();
	if (m1) m1->Update();
}

void do_motor_test_pwm(Motor *m0, Motor *m1)
{
	int divisor[] = {1024, 256, 64, 8, 1};
	int i;

	for (i = 0; i < sizeof(divisor) / sizeof(divisor[0]); i++) {
		setDivisorTimer1(divisor[i]);
		do_motor_test1(m0, m1);
	}
	setDivisorTimer1(1);
}

void do_motor_test2(Motor *m)
{
	int i;

	m->bDiag = 1;

	for (i = 0; i < 100; i++) {
		m->SetPwm(m->GetMinPwm());
		delay(10);
		m->Update();
	}

	for (i = 0; i < 100; i++) {
		m->SetPwm(m->GetInitPwm());
		delay(10);
		m->Update();
#if 1
		if (m->GetCurRpm() > m->GetMinRpm()) {
			break;
		}
#endif
	}

	for (i = 0; i < 100; i++) {
		m->SetPwm(m->GetMinPwm());
		delay(10);
		m->Update();
	}

	for (i = 0; i < 100; i++) {
		m->SetPwm(0);
		delay(10);
		m->Update();
	}
}

