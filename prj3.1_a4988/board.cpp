#include "Arduino.h"
#include "board.h"


/*
 * Enable definition to share timer 2 with IRremote library.
 * You have to modify the IRremote library
 *  - add a call to timer_isr() at the end of IRTimer() in IRremote.cpp
 */

#if defined(SHARE_TIMER2_WITH_IRREMOTE)
#define TIMER_HZ	20000
#else
#define TIMER_HZ	64000
//#define TIMER_HZ	128000
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
	Serial.println("timer_isr attached");
#else

	cli();//stop interrupts

#if defined(USE_MOTOR_TIMER1)
	//set timer1 interrupt at 1Hz
	TCCR1A = 0;// set entire TCCR1A register to 0
	TCCR1B = 0;// same for TCCR1B
	TCNT1  = 0;//initialize counter value to 0
#if 0
	// set compare match register for 1hz increments
	OCR1A = 15624;// = (16*10^6) / (1*1024) - 1 (must be <65536)

	// Set CS10 and CS12 bits for 1024 prescaler
	TCCR1B |= (1 << CS12) | (1 << CS10);  
#else
	//OCR1A = 125;  // 16Khz
	//OCR1A = 100;  // 20Khz
	//OCR1A = 80;   // 25Khz
#if (TIMER_HZ == 64000)
	OCR1A = 30;   // 64Khz
#elif (TIMER_HZ == 128000)
	OCR1A = 15;   // 128Khz
#else
#error unsupported TIMER_HZ
#endif

	// Set the timer pre-scaler
	// Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
	TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);
#endif
	// turn on CTC mode
	TCCR1B |= (1 << WGM12);
	// enable timer compare interrupt
	TIMSK1 |= (1 << OCIE1A);
#else
	/*
	 * set timer2 interrupt at 8kHz
	 * ref: http://www.instructables.com/id/Arduino-Timer-Interrupts
	 */
	TCCR2A = 0;// set entire TCCR2A register to 0
	TCCR2B = 0;// same for TCCR2B
	TCNT2  = 0;//initialize counter value to 0
#if (TIMER_HZ == 8000)
	// set compare match register for 8khz increments
	OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
	// Set CS21 bit for 8 prescaler
	TCCR2B |= (1 << CS21);   
#elif (TIMER_HZ == 16000)
	// set compare match register for 16khz increments
	OCR2A = 124;// = (16*10^6) / (16000*8) - 1 (must be <256)
	// Set CS21 bit for 8 prescaler
	TCCR2B |= (1 << CS21);   
#elif (TIMER_HZ == 64000)
	// set compare match register for 16khz increments
	OCR2A = 30;// = (16*10^6) / (64000*8) - 1 (must be <256)
	// Set CS21 bit for 8 prescaler
	TCCR2B |= (1 << CS21);   
#else
#error unsupported TIMER_HZ
#endif
	// turn on CTC mode
	TCCR2A |= (1 << WGM21);
	// enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);

	Serial.println("timer2 done.");
#endif

	sei();//allow interrupts
#endif

	// setup pin
	pinMode(STEP0, OUTPUT);
	pinMode(DIR0, OUTPUT);
	pinMode(EN0, OUTPUT);
	pinMode(STEP1, OUTPUT);
	pinMode(DIR1, OUTPUT);
	pinMode(EN1, OUTPUT);

	digitalWrite(EN0, LOW);
	digitalWrite(EN1, LOW);

	Serial.println("board initialized.");
}

/*
 * ISR for timer2 runs at 8kHz frequency.
 */
uint16_t cnt, max_cnt;
#if defined(SHARE_TIMER2_WITH_IRREMOTE)
void timer_isr()
#elif defined(USE_MOTOR_TIMER1)
ISR(TIMER1_COMPA_vect)
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
	uint16_t maxcnt = (uint16_t)(TIMER_HZ / pps);
	if (maxcnt < 1)
		maxcnt = 1;
#if 0
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

