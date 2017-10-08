#include <AFMotor.h>
#include "wheel.h"

extern void setup_IR(void);
extern uint32_t recv_IR(void);

AF_DCMotor m3(3, MOTOR34_1KHZ);
AF_DCMotor m4(4, MOTOR34_1KHZ);

Wheel wheel_left(&count_m3, &total_count_m3, calib_data_m3);
Wheel wheel_right(&count_m4, &total_count_m4, calib_data_m4);

#define LOOP_MS     10
unsigned long loop_timer;

void wheel_loop()
{
  wheel_left.Loop();
  wheel_right.Loop();

  /*
   * The angle calculations are tuned for a loop time of LOOP_MS milliseconds.
   * To make sure every loop is exactly LOOP_MS milliseconds a wait loop
   * is created by setting the loop_timer variable to +4000 microseconds
   * every loop.
   */
  while (loop_timer > micros());
  loop_timer += LOOP_MS * 1000;
}

void do_test1(Wheel *p)
{
  int i;

  /*Set the loop_timer variable at the next end loop time */
  loop_timer = micros() + LOOP_MS * 1000;
#define TEST1_DELAY	80
  p->SetSpeed(10);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(20);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(50);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(120);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(MAX_MMPS); for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(0);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(MAX_MMPS); for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(120);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(50);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(20);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(10);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(0);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
#if 1
  p->SetSpeed(-10);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(-20);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(-50);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(-120);	for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(-MAX_MMPS); for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(0);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(-MAX_MMPS); for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(-120);	for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(-50);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(-20);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
  p->SetSpeed(-10);		for (i = 0; i < TEST1_DELAY; i++) wheel_loop();
#endif

  p->SetPwm(0);
  for (i = 0; i < 10; i++) wheel_loop();
}

void do_test0(void)
{
  int i;

  wheel_left.Calibrate();
  wheel_right.Calibrate();

  wheel_left.SetPwm(0);
  wheel_right.SetPwm(0);
  for (i = 0; i < 10; i++) wheel_loop();

}

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  setup_IR();

  init_encoders();

  wheel_left.AttachMotor(&m3);
  wheel_right.AttachMotor(&m4);

  //do_test();
  Serial.println("start...");
}

uint32_t last_ir_code;
uint32_t last_ir_ms;

void loop()
{
  uint32_t ir_code;

  ir_code = recv_IR();

  if (ir_code) {
    if ((last_ir_ms + 250 < millis()) || (ir_code != last_ir_code)) {
      if (ir_code == 0x4de93dc4) {
        Serial.println("^");
      } else if (ir_code == 0x26e6c1ca) {
        Serial.println("v");
      } else if (ir_code == 0xdad4e90b) {
        Serial.println("<");
      } else if (ir_code == 0x6d89e538) {
        Serial.println(">");
      } else if (ir_code == 0x7d399127) {
        Serial.println("OK");
      } else if (ir_code == 0x68a199f0) {
        Serial.println("REC");
      } else if (ir_code == 0xf169e8b2) {
        Serial.println("REPLAY");
      } else if (ir_code == 0xcf98a7b6) {
        Serial.println("CH+");
        wheel_left.bDiag = 1;
		wheel_left.IncKp();
        wheel_right.bDiag = 1;
		wheel_right.IncKp();
      } else if (ir_code == 0x107f5e27) {
        Serial.println("CH-");
        wheel_left.bDiag = 1;
		wheel_left.DecKp();
        wheel_right.bDiag = 1;
		wheel_right.DecKp();
      } else if (ir_code == 0xd7d018ec) {
        Serial.println("VOL+");
        wheel_left.bDiag = 1;
		wheel_left.IncKi();
        wheel_right.bDiag = 1;
		wheel_right.IncKi();
      } else if (ir_code == 0xf49b208a) {
        Serial.println("VOL-");
        wheel_left.bDiag = 1;
		wheel_left.DecKi();
        wheel_right.bDiag = 1;
		wheel_right.DecKi();
      } else if (ir_code == 0x32939470) {
        Serial.println("PLAY/PAUSE");
      } else if (ir_code == 0x16d5cb04) {
        Serial.println("FF");
        wheel_left.bDiag = 1;
		wheel_left.IncKd();
        wheel_right.bDiag = 1;
		wheel_right.IncKd();
      } else if (ir_code == 0x19fd189b) {
        Serial.println("REW");
        wheel_left.bDiag = 1;
		wheel_left.DecKd();
        wheel_right.bDiag = 1;
		wheel_right.DecKd();
      } else if (ir_code == 0x407e2e01) {
        Serial.println("STOP");
      } else if (ir_code == 0x7547960e) {
        Serial.println("NEXT");
      } else if (ir_code == 0xd1921028) {
        Serial.println("PREV");
      } else if (ir_code == 0x26ecbcf3) {
        Serial.println("(0)");
        do_test0();
      } else if (ir_code == 0x9004b206) {
        Serial.println("(1)");
        wheel_left.bDiag = 1;
        wheel_right.bDiag = 0;
        do_test1(&wheel_left);
      } else if (ir_code == 0xc35f14b9) {
        Serial.println("(2)");
        wheel_left.bDiag = 0;
        wheel_right.bDiag = 1;
        do_test1(&wheel_right);
      } else if (ir_code == 0x6bef8366) {
        Serial.println("PWR");
      } else if (ir_code == 0xbe663d0a) {
        Serial.println("MUTE");
      } else if (ir_code == 0xae7fbf1d) {
        Serial.println("TEXT");
      } else {
        Serial.println(ir_code, HEX);
      }
	}
    last_ir_ms = millis();
	last_ir_code = ir_code;
  }

  delay(10);
}

