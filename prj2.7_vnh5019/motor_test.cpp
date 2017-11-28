#include <Arduino.h>
#include "board.h"
#include "motor_test.h"

void print_motor_test_header1(void)
{
	Serial.println("pwm_freq,pwm,intr,RPM,instant_RPM");
}

void print_motor_test1(Motor *m, unsigned long msec)
{
	int32_t rpm;
	int32_t intr;
	
	Serial.print(",\t");
	if (m) {
		intr = m->GetAccIntr();

		Serial.print(intr); Serial.print(",\t");
		rpm = intr * 60 * 1000 / INTR_PER_REV / (long)msec;
		Serial.print(rpm);
		Serial.print(",\t");
		Serial.print(m->GetCurRpm());
	} else {
		Serial.print("0,\t0,\t0");
	}
}

void measure_motor_test1(int pwm, Motor *m0, Motor *m1)
{
	int i;
	unsigned long msec;
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
	for (i = 0; i < 800 / INTERVAL_MOTOR_UPDATE; i++) {
		delay(INTERVAL_MOTOR_UPDATE);
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


/*
 * motor test2
 * 1. to get initial pwm value to start wheel spinning when increasing pwm
 * 2. to get minimum pwm value to keep wheel spinning when decreasing pwm
 * 3. to get minimum RPM value to keep wheel spinning when decreasing pwm
 */
#define MAX_PWM_IN_TEST2	60

void do_motor_test2(Motor *m0, Motor *m1)
{
	int16_t pwm;

	/* increasing */
	print_motor_test_header1();
	pwm = 0;
	m0->SetPwm(pwm);
	if (m1) m1->SetPwm(pwm);
	delay(1000);
	for (pwm = 0; pwm <= MAX_PWM_IN_TEST2; pwm += 1) {
		measure_motor_test1(pwm, m0, m1);
	}

	/* decreasing */
	print_motor_test_header1();
	pwm = MAX_PWM_IN_TEST2;
	m0->SetPwm(pwm);
	if (m1) m1->SetPwm(pwm);
	delay(1000);
	for (pwm = MAX_PWM_IN_TEST2; pwm >= 0; pwm -= 1) {
		measure_motor_test1(pwm, m0, m1);
	}

	m0->SetPwm(0);
	if (m1) m1->SetPwm(0);
	m0->Update();
	if (m1) m1->Update();
}

#define INTERVAL_MOTOR_UPDATE3	30

void print_motor_test_header3(void)
{
	Serial.println("pwm_freq,pwm,intr,RPM,instant_RPM");
}

void do_motor_test3_unit(int pwm, Motor *m0, Motor *m1)
{
	unsigned long msec;
	uint16_t freq = getPwmFrequencyTimer1();

	Serial.print(freq);
	Serial.print(",\t");
	Serial.print(pwm);

	msec = millis();
	m0->ResetAccIntr();
	m1->ResetAccIntr();
	delay(INTERVAL_MOTOR_UPDATE3);
	m0->Update();
	m1->Update();
	msec = millis() - msec;
	print_motor_test1(m0, msec);
	print_motor_test1(m1, msec);

	Serial.println();
}

void do_motor_test3(Motor *m0, Motor *m1)
{
	int i, pwm;

	print_motor_test_header3();
	for (pwm = 0; pwm < 30; pwm++) {
		m0->SetPwm(pwm);
		m1->SetPwm(pwm);
		for (i = 0; i < 500 / INTERVAL_MOTOR_UPDATE3; i++) {
			do_motor_test3_unit(pwm, m0, m1);
		}
	}

	print_motor_test_header3();
	for (pwm = 30; pwm > 0; pwm--) {
		m0->SetPwm(pwm);
		m1->SetPwm(pwm);
		for (i = 0; i < 500 / INTERVAL_MOTOR_UPDATE3; i++) {
			do_motor_test3_unit(pwm, m0, m1);
		}
	}

	m0->SetPwm(0);
	m0->SetPwm(0);
	m1->Update();
	m1->Update();
}

void print_motor_test4_header(void)
{
	Serial.println("pwm_freq,tgt_pwm,cur_pwm,intr,RPM");
}
void print_motor_test4(Motor *m)
{
	int32_t intr = m->GetAccIntr();
	int16_t cur_pwm = m->GetCurPwm();
	int16_t tgt_pwm = m->GetTgtPwm();
	int16_t rpm = m->GetCurRpm();
	uint16_t freq = getPwmFrequencyTimer1();

	Serial.print(freq);
	Serial.print(",\t");
	Serial.print(tgt_pwm);
	Serial.print(",\t");
	Serial.print(cur_pwm);
	Serial.print(",\t");
	Serial.print(intr);
	Serial.print(",\t");
	Serial.print(rpm);
	Serial.println();
}

void do_motor_test4_unit(Motor *m, int pwm)
{
	int i;

	m->ResetAccIntr();

	m->SetPwm(pwm);
	m->Update();
	for (i = 0; i < 1000 / INTERVAL_MOTOR_UPDATE; i++) {
		delay(INTERVAL_MOTOR_UPDATE);
		m->Update();
		print_motor_test4(m);
		m->ResetAccIntr();
	}
}

/*
 * Test motor speed transition across InitPwm
 *                IP*2   IP/2   0  -IP/2  -IP*2
 *  InitPwm * 2          (1)   (2)   (3)   (4)
 *  InitPwm / 2    (5)         (7)   (8)   (9)
 *      0         (10)  (11)        (13)  (14)
 * -InitPwm / 2   (15)  (16)  (17)        (19)
 * -InitPwm * 2   (20)  (21)  (22)  (23)
 */
void do_motor_test4(Motor *m)
{
	int i;

	print_motor_test4_header();

	/* transition between 0 and the others */
	do_motor_test4_unit(m, 0);						// init
	do_motor_test4_unit(m, -(m->GetInitPwm() * 2));	// (14)
	do_motor_test4_unit(m, 0);						// (22)
	do_motor_test4_unit(m, -(m->GetInitPwm() / 2));	// (13)
	do_motor_test4_unit(m, 0);						// (17)
	do_motor_test4_unit(m,  (m->GetInitPwm() / 2));	// (11)
	do_motor_test4_unit(m, 0);						//  (7)
	do_motor_test4_unit(m,  (m->GetInitPwm() * 2));	// (10)
	do_motor_test4_unit(m, 0);						//  (2)

	/* transition between IP*2 and the others */
	do_motor_test4_unit(m,  (m->GetInitPwm() * 2));	// init (10)
	do_motor_test4_unit(m, -(m->GetInitPwm() * 2));	//  (4)
	do_motor_test4_unit(m,  (m->GetInitPwm() * 2));	// (20)
	do_motor_test4_unit(m, -(m->GetInitPwm() / 2));	//  (3)
	do_motor_test4_unit(m,  (m->GetInitPwm() * 2));	// (15)
	do_motor_test4_unit(m,  (m->GetInitPwm() / 2));	//  (1)
	do_motor_test4_unit(m,  (m->GetInitPwm() * 2));	//  (5)

	/* transition between IP/2 and the others */
	do_motor_test4_unit(m,  (m->GetInitPwm() / 2));	// init (1)
	do_motor_test4_unit(m, -(m->GetInitPwm() / 2));	//  (8)
	do_motor_test4_unit(m,  (m->GetInitPwm() / 2));	// (16)
	do_motor_test4_unit(m, -(m->GetInitPwm() * 2));	//  (9)
	do_motor_test4_unit(m,  (m->GetInitPwm() / 2));	// (21)

	/* transition between -IP/2 and the others */
	do_motor_test4_unit(m, -(m->GetInitPwm() / 2));	// init (8)
	do_motor_test4_unit(m, -(m->GetInitPwm() * 2));	// (19)
	do_motor_test4_unit(m, -(m->GetInitPwm() / 2));	// (23)

	do_motor_test4_unit(m, 0);						// finish
}

