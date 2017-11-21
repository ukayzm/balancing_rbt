#include <Arduino.h>
#include "board.h"
#include "wheel_test.h"

void print_wheel_test_header1(void)
{
	Serial.println("pwm,intr,RPM,speed");
}
void print_wheel_test1(Wheel *w, int msec)
{
	int32_t rpm;
	int32_t intr;
	
	Serial.print(",\t");
	if (w) {
		intr = w->GetAccIntr();

		Serial.print(intr); Serial.print(",\t");
		rpm = intr * 60 * 1000 / ENCODER_CPR / GEAR_RATIO / msec;
		Serial.print(rpm); Serial.print(",\t");
		Serial.print(w->GetCurSpeed(), 2);
	} else {
		Serial.print("0,\t0,\t0.00");
	}
}

void measure_wheel_test1(int pwm, Wheel *w0, Wheel *w1)
{
	int i, msec;

	Serial.print(pwm);
	w0->SetPwm(pwm);
	if (w1) w1->SetPwm(pwm);
	delay(200);
	w0->Update();
	if (w1) w1->Update();
	w0->ResetAccIntr();
	if (w1) w1->ResetAccIntr();
	msec = millis();
	for (i = 0; i < 80; i++) {
		delay(10);
		w0->Update();
		if (w1) w1->Update();
	}
	msec = millis() - msec;
	print_wheel_test1(w0, msec);
	if (w1) print_wheel_test1(w1, msec);
	Serial.println();
}

void do_wheel_test1(Wheel *w0, Wheel *w1)
{
	int16_t pwm;

	/* increasing */
	print_wheel_test_header1();
	pwm = 0;
	w0->SetPwm(pwm);
	if (w1) w1->SetPwm(pwm);
	delay(1000);
	for (pwm = 0; pwm <= 260; pwm += 10) {
		measure_wheel_test1(pwm, w0, w1);
	}

	/* decreasing */
	print_wheel_test_header1();
	pwm = 260;
	w0->SetPwm(pwm);
	if (w1) w1->SetPwm(pwm);
	delay(1000);
	for (pwm = 260; pwm >= 0; pwm -= 10) {
		measure_wheel_test1(pwm, w0, w1);
	}

	w0->SetPwm(0);
	if (w1) w1->SetPwm(0);
	w0->Update();
	if (w1) w1->Update();
}

void do_wheel_test_pwm(Wheel *w0, Wheel *w1)
{
	int divisor[] = {1024, 256, 64, 8, 1};
	int i;

	for (i = 0; i < sizeof(divisor) / sizeof(divisor[0]); i++) {
		setDivisorTimer1(divisor[i]);
		do_wheel_test1(w0, w1);
	}
	setDivisorTimer1(1);
}

void do_wheel_test2(Wheel *w)
{
	int i;

	w->bDiag = 1;

	for (i = 0; i < 100; i++) {
		w->SetPwm(w->GetMinPwm());
		delay(10);
		w->Update();
	}

	for (i = 0; i < 100; i++) {
		w->SetPwm(w->GetInitPwm());
		delay(10);
		w->Update();
#if 1
		if (w->GetCurSpeed() > w->GetMinSpeed()) {
			break;
		}
#endif
	}

	for (i = 0; i < 100; i++) {
		w->SetPwm(w->GetMinPwm());
		delay(10);
		w->Update();
	}

	for (i = 0; i < 100; i++) {
		w->SetPwm(0);
		delay(10);
		w->Update();
	}
}

