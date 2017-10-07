#include <AFMotor.h>
#include "wheel.h"

AF_DCMotor m3(3, MOTOR34_1KHZ);
AF_DCMotor m4(4, MOTOR34_1KHZ);

Wheel wheel_left(&count_m3, &total_count_m3, calib_data_m3);
Wheel wheel_right(&count_m4, &total_count_m4, calib_data_m4);

#define LOOP_MS     10
unsigned long loop_timer;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  init_encoders();

  wheel_left.AttachMotor(&m3);
  wheel_right.AttachMotor(&m4);

  wheel_left.bDiag = 1;
  wheel_right.bDiag = 1;

  /*Set the loop_timer variable at the next end loop time */
  loop_timer = micros() + LOOP_MS * 1000;

  int i;
  Wheel *p = &wheel_left;

#if 1
  p->SetSpeed(20);		for (i = 0; i < 100; i++) loop();
  p->SetSpeed(50);		for (i = 0; i < 100; i++) loop();
  p->SetSpeed(100);		for (i = 0; i < 100; i++) loop();
  p->SetSpeed(200);		for (i = 0; i < 100; i++) loop();
  p->SetSpeed(MAX_MMPS); for (i = 0; i < 100; i++) loop();
  p->SetSpeed(200);		for (i = 0; i < 100; i++) loop();
  p->SetSpeed(100);		for (i = 0; i < 100; i++) loop();
  p->SetSpeed(50);		for (i = 0; i < 100; i++) loop();
  p->SetSpeed(20);		for (i = 0; i < 100; i++) loop();
  p->SetSpeed(0);		for (i = 0; i < 100; i++) loop();
  p->SetSpeed(-20);		for (i = 0; i < 100; i++) loop();
  p->SetSpeed(-50);	for (i = 0; i < 100; i++) loop();
  p->SetSpeed(-100);	for (i = 0; i < 100; i++) loop();
  p->SetSpeed(-200);	for (i = 0; i < 100; i++) loop();
  p->SetSpeed(-MAX_MMPS); for (i = 0; i < 100; i++) loop();
  p->SetSpeed(-200);	for (i = 0; i < 100; i++) loop();
  p->SetSpeed(-100);	for (i = 0; i < 100; i++) loop();
  p->SetSpeed(-50);	for (i = 0; i < 100; i++) loop();
  p->SetSpeed(-20);		for (i = 0; i < 100; i++) loop();
#else
#if 1
  wheel_left.Calibrate();
  wheel_right.Calibrate();
#else
  p->SetSpeed(70);		for (i = 0; i < 100; i++) loop();
#endif
#endif

  wheel_left.SetPwm(0);
  wheel_right.SetPwm(0);
  for (i = 0; i < 100; i++) loop();

  while (1);

  /*Set the loop_timer variable at the next end loop time */
  loop_timer = micros() + LOOP_MS * 1000;

}

unsigned long prev_total_count_m3, prev_total_count_m4;

void loop()
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

