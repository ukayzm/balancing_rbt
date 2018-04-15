/*
 * prj3.1 - a4988
 */

//simple A4988 connection
//jumper reset and sleep together
//connect  VDD to Arduino 3.3v or 5v
//connect  GND to Arduino GND (GND near VDD)
//connect  1A and 1B to stepper coil 1
//connect 2A and 2B to stepper coil 2
//connect VMOT to power source (9v battery + term)
//connect GRD to power source (9v battery - term)

int stp = 5;  //connect pin 13 to step
int dir = 6;  // connect pin 12 to dir
int en = 7;  // connect pin 12 to dir

void setup() 
{                
	Serial.begin(115200);
	pinMode(stp, OUTPUT);
	pinMode(dir, OUTPUT);       
	pinMode(en, OUTPUT);       
	digitalWrite(en, LOW);   

	/*
	 * set timer2 interrupt at 8kHz
	 * ref: http://www.instructables.com/id/Arduino-Timer-Interrupts
	 */
	cli();//stop interrupts

	TCCR2A = 0;// set entire TCCR2A register to 0
	TCCR2B = 0;// same for TCCR2B
	TCNT2  = 0;//initialize counter value to 0
	// set compare match register for 8khz increments
	OCR2A = 249;// = (16*10^6) / (8000*8) - 1 (must be <256)
	// turn on CTC mode
	TCCR2A |= (1 << WGM21);
	// Set CS21 bit for 8 prescaler
	TCCR2B |= (1 << CS21);   
	// enable timer compare interrupt
	TIMSK2 |= (1 << OCIE2A);

	sei();//allow interrupts

	digitalWrite(dir, HIGH);
}

/*
 * ISR for timer2 runs at 8kHz frequency.
 */
uint16_t cnt, max_cnt;
ISR(TIMER2_COMPA_vect)
{
	if (max_cnt > 0) {
		if (cnt == 0) {
			digitalWrite(stp, HIGH);  
		} else if (cnt == 1) {
			digitalWrite(stp, LOW);  
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
	return (uint16_t)(8000.0 / pps);
}

void set_rpm(int16_t rpm)
{
	if (rpm == 0) {
		max_cnt = 0;
		cnt = 0;
		digitalWrite(stp, LOW);
	}
	max_cnt = rpm2maxcnt(rpm);
	Serial.print(rpm);
	Serial.print(", ");
	Serial.println(max_cnt);
}

void loop() 
{
	int rpm;

	for (rpm = 0; rpm <= 30; rpm += 1)
	{
		set_rpm(rpm);
		delay(1000);
	}
	for (rpm = 30; rpm <= 300; rpm += 10)
	{
		set_rpm(rpm);
		delay(1000);
	}
}
