#include "Arduino.h"
#include <Servo.h>
#include <NewPing.h>


/* 10 ms -> 1.7 m */
/* 20 ms -> 3.4 m */
/* 30 ms -> 5.1 m */
/* 40 ms -> 6.8 m */
/* 50 ms -> 8.5 m */
#define PING_INTERVAL	28

#define ADJUST_INTERVAL	18

#define PIN_TRIG	A4
#define PIN_ECHO	A5
NewPing sonar(PIN_TRIG, PIN_ECHO, 500);	/* max distance 500 cm */


#define PIN_SERVO_PAN		10	/* PAN servo */
#define MIN_ANGLE_PAN		0
#define MAX_ANGLE_PAN		160
#define INITIAL_ANGLE_PAN	80
Servo servo_pan;
int pos_pan;
int dir_pan = 2;

//#define HAS_TILT_SERVO
#if defined(HAS_TILT_SERVO)
#define PIN_SERVO_TILT		9	/* TILT servo */
Servo servo_tilt;
int pos_tilt;
int dir_tilt = 1;
#endif

int radar_state;
unsigned long radar_timer;
unsigned int distance_cm;

enum {
	STATE_ADJUST_POSITION,
	STATE_GET_DISTANCE,
	STATE_MAX,
};

void radar_setup() {
	/* servo */
	pinMode(PIN_SERVO_PAN, OUTPUT);
	servo_pan.attach(PIN_SERVO_PAN);
	pos_pan = INITIAL_ANGLE_PAN;
	servo_pan.write(pos_pan);
#if defined(HAS_TILT_SERVO)
	pinMode(PIN_SERVO_TILT, OUTPUT);
	servo_tilt.attach(PIN_SERVO_TILT);
	pos_tilt = 80;
	servo_tilt.write(pos_tilt);
#endif
	delay(500);

	radar_state = STATE_ADJUST_POSITION;

	radar_timer = millis() + ADJUST_INTERVAL;
}

void radar_print(void)
{
	static unsigned long prev_ms;
	unsigned long cur_ms = millis();

	Serial.print(cur_ms - prev_ms);
	Serial.print(" ms ");
	Serial.print(pos_pan);	Serial.print(" deg ");
#if defined(HAS_TILT_SERVO)
	Serial.print(pos_tilt);	Serial.print(" deg ");
#endif
	Serial.print(distance_cm); Serial.print(" cm ");
	Serial.println();
	prev_ms = cur_ms;
}

void echo_check(void)
{
	if (sonar.check_timer()) {
		distance_cm = sonar.ping_result / US_ROUNDTRIP_CM;
	}
}

void radar_loop()
{
	unsigned long cur_ms = millis();

	if (cur_ms < radar_timer) {
		return;
	}

	if (radar_state == STATE_ADJUST_POSITION) {
		radar_state = STATE_GET_DISTANCE;
		radar_timer = cur_ms + PING_INTERVAL;
		sonar.timer_stop();
		sonar.ping_timer(echo_check);
	} else if (radar_state == STATE_GET_DISTANCE) {
		radar_state = STATE_ADJUST_POSITION;
		radar_timer = cur_ms + ADJUST_INTERVAL;

		/* set servo angle */
		if (pos_pan + dir_pan < MIN_ANGLE_PAN || pos_pan + dir_pan > MAX_ANGLE_PAN)
			dir_pan = -dir_pan;
		pos_pan += dir_pan;
		servo_pan.write(pos_pan);
#if defined(HAS_TILT_SERVO)
#endif

		radar_print();
	}
}

