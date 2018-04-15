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
int a = 0;     //  gen counter

void setup() 
{                
	Serial.begin(115200);
	pinMode(stp, OUTPUT);
	pinMode(dir, OUTPUT);       
	pinMode(en, OUTPUT);       
	digitalWrite(en, LOW);   
}

void loop() 
{
	if (a <  200)  //sweep 200 step in dir 1
	{
		a++;
		digitalWrite(stp, HIGH);   
		delay(10);               
		digitalWrite(stp, LOW);  
		delay(10);              
		Serial.println(a);
	}
	else     //sweep 200 in dir 2
	{
		digitalWrite(dir, HIGH);
		a++;
		digitalWrite(stp, HIGH);  
		delay(10);               
		digitalWrite(stp, LOW);  
		delay(10);

		if (a>400)  // Initialize
		{
			a = 0;
			digitalWrite(dir, LOW);
		}
		Serial.println(a);
	}
}
