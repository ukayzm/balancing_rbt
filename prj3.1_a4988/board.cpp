#include "Arduino.h"
#include "board.h"
#include "mpu6050.h"


#define TIMER2_HZ	16000


void setup_board()
{
	Serial.println("Power supply: 3S LiPo Battery");
	Serial.println("Motor driver: a4988");

	//setup_IR();
	//Serial.println("IR done.");

	mpu6050_setup();
	Serial.println("mpu6050 done.");
	
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

	Serial.println("timer2 done.");

	// setup pin
	pinMode(STEP0, OUTPUT);
	pinMode(DIR0, OUTPUT);
	pinMode(EN0, OUTPUT);
	pinMode(STEP1, OUTPUT);
	pinMode(DIR1, OUTPUT);
	pinMode(EN1, OUTPUT);

	digitalWrite(EN0, LOW);
	digitalWrite(EN1, LOW);
}

/*
 * ISR for timer2 runs at 8kHz frequency.
 */
uint16_t cnt, max_cnt;
ISR(TIMER2_COMPA_vect)
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

uint16_t rpm2maxcnt(uint16_t rpm)
{
	float pps = rpm * 200.0 / 60.0;
	return (uint16_t)(TIMER2_HZ / pps);
}

void motor_set_rpm(int16_t rpm)
{
	if (rpm == 0) {
		max_cnt = 0;
		cnt = 0;
		digitalWrite(STEP0, LOW);
		max_cnt = 0;
	} else if (rpm < 0) {
		digitalWrite(DIR0, LOW);
		digitalWrite(DIR1, HIGH);
		max_cnt = rpm2maxcnt(-rpm);
	} else {
		digitalWrite(DIR0, HIGH);
		digitalWrite(DIR1, LOW);
		max_cnt = rpm2maxcnt(rpm);
	}
#if 0
	Serial.print(rpm);
	Serial.print(", ");
	Serial.println(max_cnt);
#endif
}

