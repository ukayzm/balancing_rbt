/*
 * prj3.1 - a4988
 */
#include <IRremote.h>
#include "board.h"
#include "mpu6050.h"
#include "pid.h"

IRrecv irrecv(IR_PIN);

unsigned long loop_start_usec;

void check_ir();
extern void balancing_setup(void);
extern void balancing_loop(void);
extern void motor_set_rpm(int16_t rpm);

extern Pid AnglePid, SpeedPid;

void setup() 
{                
	Serial.begin(115200);
	Serial.println("start setup...");

	mpu6050_setup();
	Serial.println("MPU6050 initialized");

	irrecv.enableIRIn(); // Start the receiver
	Serial.println("IR initialized");

	setup_board();
	balancing_setup();

	loop_start_usec = micros();
}

void loop() 
{
#if 1
	unsigned long used_usec;

	mpu6050_loop();
	balancing_loop();
	check_ir();

	/* delay until LOOP_USEC passes */
	used_usec = micros() - loop_start_usec;
	if (used_usec < LOOP_USEC) {
		delayMicroseconds(LOOP_USEC - used_usec);
	}
	loop_start_usec = micros();
#else
#if 0
	int i;

	for (i = 0; i < 30; i++) {
		motor_set_rpm(i);
		delay(500);
	}
#else
	int i;

	for (i = 0; i < 500; i += 1) {
		motor_set_rpm(i);
		delay(5);
	}
	for (i = 500; i >= -500; i -= 1) {
		motor_set_rpm(i);
		delay(5);
	}
	for (i = -500; i < 0; i += 1) {
		motor_set_rpm(i);
		delay(5);
	}
#endif
#endif
}

#if 1
extern double fAngleKp;
extern double fAngleKi;
extern double fAngleKd;
extern double fSpeedKp;
extern double fSpeedKi;
extern double fSpeedKd;
extern void balancing_inc_tgt(void);
extern void balancing_dec_tgt(void);
extern void balancing_inc_dir(void);
extern void balancing_dec_dir(void);
extern void balancing_reset_tgtdir(void);

void check_ir()
{
  decode_results results;
  uint32_t ir_code;
  static uint32_t last_ir_code;
  static uint32_t last_ir_ms;

  results.value = 0;
  if (irrecv.decode(&results)) {
    irrecv.resume(); // Receive the next value
  }
  ir_code = results.value;

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
	SpeedPid.setKp(SpeedPid.getKp() + SPEED_KP_STEP);
  } else if (ir_code == 0x107f5e27) {
    Serial.println("CH-");
	SpeedPid.setKp(SpeedPid.getKp() - SPEED_KP_STEP);
  } else if (ir_code == 0x68a199f0) {
    Serial.println("REC");
	SpeedPid.setKi(SpeedPid.getKi() + SPEED_KI_STEP);
  } else if (ir_code == 0xf169e8b2) {
    Serial.println("REPLAY");
	SpeedPid.setKi(SpeedPid.getKi() - SPEED_KI_STEP);
  } else if (ir_code == 0xd7d018ec) {
    Serial.println("VOL+");
	SpeedPid.setKd(SpeedPid.getKd() + SPEED_KD_STEP);
  } else if (ir_code == 0xf49b208a) {
    Serial.println("VOL-");
	SpeedPid.setKd(SpeedPid.getKd() - SPEED_KD_STEP);
  } else if (ir_code == 0x16d5cb04) {
    Serial.println("FF");
	AnglePid.setKp(AnglePid.getKp() + ANGLE_KP_STEP);
  } else if (ir_code == 0x7547960e) {
    Serial.println("NEXT");
	AnglePid.setKp(AnglePid.getKp() - ANGLE_KP_STEP);
  } else if (ir_code == 0x32939470) {
    Serial.println("PLAY/PAUSE");
	AnglePid.setKi(AnglePid.getKi() + ANGLE_KI_STEP);
  } else if (ir_code == 0x407e2e01) {
    Serial.println("STOP");
	AnglePid.setKi(AnglePid.getKi() - ANGLE_KI_STEP);
  } else if (ir_code == 0x19fd189b) {
    Serial.println("REW");
	AnglePid.setKd(AnglePid.getKd() + ANGLE_KD_STEP);
  } else if (ir_code == 0xd1921028) {
    Serial.println("PREV");
	AnglePid.setKd(AnglePid.getKd() - ANGLE_KD_STEP);
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
#endif

