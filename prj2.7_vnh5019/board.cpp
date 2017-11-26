#include "Arduino.h"
#include "board.h"

int16_t count_m0, count_m1;
unsigned long total_count_m0, total_count_m1;
Motor motor_left(&count_m0, &total_count_m0);
Motor motor_right(&count_m1, &total_count_m1);


void on_intr_m0(void)
{
	if (digitalRead(M0_DIR_PIN))
		count_m0++;
	else
		count_m0--;
	total_count_m0++;
}

void on_intr_m1(void)
{
	if (digitalRead(M1_DIR_PIN))
		count_m1--;
	else
		count_m1++;
	total_count_m1++;
}

/**
 * Divides a given PWM pin frequency by a divisor.
 *
 * The resulting frequency is equal to the base frequency divided by
 * the given divisor:
 *   - Base frequencies:
 *      o The base frequency for pins 3, 9, 10, and 11 is 31250 Hz.
 *      o The base frequency for pins 5 and 6 is 62500 Hz.
 *   - Divisors:
 *      o The divisors available on pins 5, 6, 9 and 10 are: 1, 8, 64,
 *        256, and 1024.
 *      o The divisors available on pins 3 and 11 are: 1, 8, 32, 64,
 *        128, 256, and 1024.
 *
 * PWM frequencies are tied together in pairs of pins. If one in a
 * pair is changed, the other is also changed to match:
 *   - Pins 5 and 6 are paired on timer0
 *   - Pins 9 and 10 are paired on timer1
 *   - Pins 3 and 11 are paired on timer2
 *
 * Note that this function will have side effects on anything else
 * that uses timers:
 *   - Changes on pins 3, 5, 6, or 11 may cause the delay() and
 *     millis() functions to stop working. Other timing-related
 *     functions may also be affected.
 *   - Changes on pins 9 or 10 will cause the Servo library to function
 *     incorrectly.
 *
 * Thanks to macegr of the Arduino forums for his documentation of the
 * PWM frequency divisors. His post can be viewed at:
 *   http://forum.arduino.cc/index.php?topic=16612#msg121031
 */
void setPwmFrequency(int pin, int divisor)
{
	byte mode;
	if(pin == 5 || pin == 6 || pin == 9 || pin == 10) {
		switch(divisor) {
			case 1: mode = 0x01; break;
			case 8: mode = 0x02; break;
			case 64: mode = 0x03; break;
			case 256: mode = 0x04; break;
			case 1024: mode = 0x05; break;
			default: return;
		}
		if(pin == 5 || pin == 6) {
			TCCR0B = TCCR0B & 0b11111000 | mode;
		} else {
			TCCR1B = TCCR1B & 0b11111000 | mode;
		}
	} else if(pin == 3 || pin == 11) {
		switch(divisor) {
			case 1: mode = 0x01; break;
			case 8: mode = 0x02; break;
			case 32: mode = 0x03; break;
			case 64: mode = 0x04; break;
			case 128: mode = 0x05; break;
			case 256: mode = 0x06; break;
			case 1024: mode = 0x07; break;
			default: return;
		}
		TCCR2B = TCCR2B & 0b11111000 | mode;
	}
}

void setDivisorTimer1(int divisor)
{
	Serial.print("Timer1 for PIN 9, 10: PWM frequency = 31250 / ");
	Serial.print(divisor);
	Serial.print(" = ");
	Serial.print(31250 / divisor);
	Serial.print(" Hz");
	Serial.println();

	setPwmFrequency(9, divisor);
}

	/*
	 * For M0_PWM_PIN 9 and M1_PWM_PIN 10, the frequency for the divisors are:
	 * o 31250 Hz for 1
	 * o  3906 Hz for 8
	 * o   488 Hz for 64
	 * o   122 Hz for 256
	 * o    30 Hz for 1024
	 */
uint16_t getPwmFrequencyTimer1(void)
{
	byte mode = TCCR1B & 0b00000111;
	switch (mode) {
	case 0x1:
		return 31250;
	case 0x2:
		return 3906;
	case 0x3:
		return 488;
	case 0x4:
		return 122;
	case 0x5:
		return 30;
	default:
		Serial.print("unknown (");
		Serial.print(mode);
		Serial.println(")");
		break;
	}
}

void setup_board()
{
	pinMode(M0_INTR_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(M0_INTR_PIN), on_intr_m0, FALLING);
	pinMode(M0_DIR_PIN, INPUT_PULLUP);
	count_m0 = 0;

	pinMode(M1_INTR_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(M1_INTR_PIN), on_intr_m1, FALLING);
	pinMode(M1_DIR_PIN, INPUT_PULLUP);
	count_m1 = 0;

	motor_left.SetPIN(M0_PWM_PIN, M0_CTRL0_PIN, M0_CTRL1_PIN, M0_CURRENT_PIN);
	motor_right.SetPIN(M1_PWM_PIN, M1_CTRL0_PIN, M1_CTRL1_PIN, M1_CURRENT_PIN);

#if 1
	motor_left.SetCharacteristics(INITIAL_PWM_M0, MIN_PWM_M0, MIN_RPM_M0);
	motor_right.SetCharacteristics(INITIAL_PWM_M1, MIN_PWM_M1, MIN_RPM_M1);
#endif

	Serial.println("Power supply: 3S LiPo Battery");
	Serial.println("Motor driver: VNH5019");

	/*
	 * For M0_PWM_PIN 9 and M1_PWM_PIN 10, the frequency for the divisors are:
	 * o 31250 Hz for 1
	 * o  3906 Hz for 8
	 * o   488 Hz for 64
	 * o   122 Hz for 256
	 * o    30 Hz for 1024
	 */
	setDivisorTimer1(1);
}

