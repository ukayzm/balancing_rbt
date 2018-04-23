#include "Arduino.h"
#include "board.h"


/*
 * Enable definition to share timer 2 with IRremote library.
 * You have to modify the IRremote library
 *  - add a call to timer_isr() at the end of IRTimer() in IRremote.cpp
 */
#define SHARE_TIMER2_WITH_IRREMOTE

#if defined(SHARE_TIMER2_WITH_IRREMOTE)
#define TIMER2_HZ	20000
#else
#define TIMER2_HZ	16000
#endif

#if defined(SHARE_TIMER2_WITH_IRREMOTE)
void timer_isr();
extern void (*external_timer2_isr)();
#endif

void setup_board()
{
	Serial.println("Power supply: 3S LiPo Battery");
	Serial.println("Motor driver: a4988");

#if defined(SHARE_TIMER2_WITH_IRREMOTE)
	external_timer2_isr = timer_isr;
#else
	/*
	 * set timer2 interrupt at 8kHz
	 * ref: http://www.instructables.com/id/Arduino-Timer-Interrupts
	 */
	cli();//stop interrupts

	TCCR2A = 0;// set entire TCCR2A register to 0
	TCCR2B = 0;// same for TCCR2B
	TCNT2  = 0;//initialize counter value to 0
#if (TIMER2_HZ == 8000)
	// set compare match register for 8khz increments
	OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
	// Set CS21 bit for 8 prescaler
	TCCR2B |= (1 << CS21);   
#elif (TIMER2_HZ == 16000)
	// set compare match register for 8khz increments
	OCR2A = 124;// = (16*10^6) / (16000*8) - 1 (must be <256)
	// Set CS21 bit for 8 prescaler
	TCCR2B |= (1 << CS21);   
#else
#error unsupported TIMER2_HZ
#endif
	// turn on CTC mode
	TCCR2A |= (1 << WGM21);
	// enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);

	sei();//allow interrupts
#endif

	Serial.println("timer2 done.");

	// setup pin
	pinMode(STEP0, OUTPUT);
	pinMode(DIR0, OUTPUT);
	pinMode(EN0, OUTPUT);
	pinMode(STEP1, OUTPUT);
	pinMode(DIR1, OUTPUT);
	pinMode(EN1, OUTPUT);

	digitalWrite(EN0, HIGH);
	digitalWrite(EN1, LOW);
}

/*
 * ISR for timer2 runs at 8kHz frequency.
 */
uint16_t cnt, max_cnt;
#if defined(SHARE_TIMER2_WITH_IRREMOTE)
void timer_isr()
#else
ISR(TIMER2_COMPA_vect)
#endif
{
	if (max_cnt > 0) {
		if (cnt == 0) {
			digitalWrite(STEP0, HIGH);  
			digitalWrite(STEP1, HIGH);  
		} else if (cnt == 1) {
			digitalWrite(STEP0, LOW);  
			digitalWrite(STEP1, LOW);  
		}
		cnt++;
		if (cnt > max_cnt) {
			cnt = 0;
		}
	}
}

/*
 * MICROSTEP:
 * 1: full step
 * 2: half step
 * 4: quarter step
 * 8: eights step
 * 16: sixteenth step
 */
#define MICROSTEP	16
uint16_t rpm2maxcnt(uint16_t rpm)
{
	float pps = rpm * MICROSTEP * (200.0 / 60.0);
	uint16_t maxcnt = (uint16_t)(TIMER2_HZ / pps);
#if 1
	Serial.print(rpm); Serial.print(" RPM");
	Serial.print("\t= "); Serial.print(pps); Serial.print(" pulse per sec (Hz)");
	Serial.print("\t= "); Serial.print(max_cnt); Serial.print(" timer tick between pulse");
	Serial.println();
#endif
	return maxcnt;
}

void motor_set_rpm(int16_t rpm)
{
	if (rpm == 0) {
		max_cnt = 0;
		cnt = 0;
		digitalWrite(STEP0, LOW);
		digitalWrite(STEP1, LOW);
		digitalWrite(EN0, HIGH);
		digitalWrite(EN1, HIGH);
		max_cnt = 0;
	} else if (rpm < 0) {
		digitalWrite(DIR0, LOW);
		digitalWrite(DIR1, HIGH);
		digitalWrite(EN0, LOW);
		digitalWrite(EN1, LOW);
		max_cnt = rpm2maxcnt(-rpm);
	} else {
		digitalWrite(DIR0, HIGH);
		digitalWrite(DIR1, LOW);
		digitalWrite(EN0, LOW);
		digitalWrite(EN1, LOW);
		max_cnt = rpm2maxcnt(rpm);
	}
}

