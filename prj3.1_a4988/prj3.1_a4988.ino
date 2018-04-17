/*
 * prj3.1 - a4988
 */


void setup() 
{                
	Serial.begin(115200);
	pinMode(stp, OUTPUT);
	pinMode(dir, OUTPUT);       
	pinMode(dir2, OUTPUT);       
	pinMode(en, OUTPUT);       
	digitalWrite(en, LOW);   

	set_rpm(300);
}

void loop() 
{
#if 1
	int rpm;

	for (rpm = 0; rpm <= 30; rpm += 2)
	{
		set_rpm(rpm);
		delay(500);
	}
	for (rpm = 30; rpm <= 500; rpm += 10)
	{
		set_rpm(rpm);
		delay(500);
	}
#endif
}

extern double fAngleKp;
extern double fAngleKi;
extern double fAngleKd;
extern double fSpeedKp;
extern double fSpeedKi;
extern double fSpeedKd;

void check_ir()
{
  uint32_t ir_code;
  static uint32_t last_ir_code;
  static uint32_t last_ir_ms;

  ir_code = recv_IR();

  if (ir_code == 0) {
    return;
  }
  if (ir_code != 0x4de93dc4
   && ir_code != 0x26e6c1ca
   && ir_code != 0x6d89e538
   && ir_code != 0xdad4e90b) {
    if ((last_ir_ms + 250 > millis()) && (ir_code == last_ir_code)) {
      return;
    }
  }

  if (ir_code == 0x4de93dc4) {
    Serial.println("^");
	balancing_inc_tgt();
  } else if (ir_code == 0x26e6c1ca) {
    Serial.println("v");
	balancing_dec_tgt();
  } else if (ir_code == 0x6d89e538) {
    Serial.println(">");
	balancing_inc_dir();
  } else if (ir_code == 0xdad4e90b) {
    Serial.println("<");
	balancing_dec_dir();
  } else if (ir_code == 0x7d399127) {
    Serial.println("OK");
	balancing_reset_tgtdir();
  } else if (ir_code == 0xcf98a7b6) {
    Serial.println("CH+");
	fSpeedKp += 0.01;
  } else if (ir_code == 0x107f5e27) {
    Serial.println("CH-");
	fSpeedKp -= 0.01;
  } else if (ir_code == 0x68a199f0) {
    Serial.println("REC");
	fSpeedKi += 0.01;
  } else if (ir_code == 0xf169e8b2) {
    Serial.println("REPLAY");
	fSpeedKi -= 0.01;
  } else if (ir_code == 0xd7d018ec) {
    Serial.println("VOL+");
	fSpeedKd += 0.01;
  } else if (ir_code == 0xf49b208a) {
    Serial.println("VOL-");
	fSpeedKd -= 0.01;
  } else if (ir_code == 0x16d5cb04) {
    Serial.println("FF");
	fAngleKp += 0.01;
  } else if (ir_code == 0x7547960e) {
    Serial.println("NEXT");
	fAngleKp -= 0.01;
  } else if (ir_code == 0x32939470) {
    Serial.println("PLAY/PAUSE");
	fAngleKi += 0.01;
  } else if (ir_code == 0x407e2e01) {
    Serial.println("STOP");
	fAngleKi -= 0.01;
  } else if (ir_code == 0x19fd189b) {
    Serial.println("REW");
	fAngleKd += 0.01;
  } else if (ir_code == 0xd1921028) {
    Serial.println("PREV");
	fAngleKd -= 0.01;
  } else if (ir_code == 0x26ecbcf3) {
    Serial.println("(0)");
  } else if (ir_code == 0x9004b206) {
    Serial.println("(1)");
  } else if (ir_code == 0xc35f14b9) {
    Serial.println("(2)");
  } else if (ir_code == 0x6bef8366) {
    Serial.println("PWR");
  } else if (ir_code == 0xbe663d0a) {
    Serial.println("MUTE");
  } else if (ir_code == 0xae7fbf1d) {
    Serial.println("TEXT");
  } else {
    Serial.println(ir_code, HEX);
  }
  last_ir_ms = millis();
  last_ir_code = ir_code;
}
