#include <Arduino.h>
#include <NewPing.h>

//#define PING_INTERVAL	10	/* 10 ms -> 1.7 m */
//#define PING_INTERVAL	20	/* 20 ms -> 3.4 m */
//#define PING_INTERVAL	30	/* 30 ms -> 5.1 m */
//#define PING_INTERVAL	40	/* 40 ms -> 6.8 m */
#define PING_INTERVAL	50	/* 50 ms -> 8.5 m */
//#define PING_INTERVAL	60	/* 50 ms -> 10.2 m */

#define PIN_TRIG	A4
#define PIN_ECHO	A5

NewPing sonar(PIN_TRIG, PIN_ECHO, 500);

unsigned long pingTimer;
unsigned int distance_cm;

void sr04_setup(void)
{
	pingTimer = millis() + 75;
}

void echo_check(void)
{
	if (sonar.check_timer()) {
		distance_cm = sonar.ping_result / US_ROUNDTRIP_CM;
	}
}

void sr04_loop(void)
{
	static unsigned long prev_ms;
	unsigned long cur_ms = millis();

	if (cur_ms >= pingTimer) {
		pingTimer = cur_ms + PING_INTERVAL;
		sonar.timer_stop();
		sonar.ping_timer(echo_check);
		Serial.print(cur_ms - prev_ms);
		Serial.print(" ms ");
		Serial.print(distance_cm);
		Serial.print(" cm");
		Serial.println();
		prev_ms = cur_ms;
	}
}

