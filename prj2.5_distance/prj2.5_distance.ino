#include "Arduino.h"

extern void sr04_setup(void);
extern void sr04_loop(void);
extern void sharpir_setup(void);
extern void sharpir_loop(void);

#define LOOP_MS     10
unsigned long loop_timer;


void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);

  sr04_setup();

  Serial.println("start...");

  /*Set the loop_timer variable at the next end loop time */
  loop_timer = micros() + LOOP_MS * 1000;
  while (loop_timer > micros());
  loop_timer += LOOP_MS * 1000;
}


void loop()
{
  sr04_loop();

  /*
   * The angle calculations are tuned for a loop time of LOOP_MS milliseconds.
   * To make sure every loop is exactly LOOP_MS milliseconds a wait loop
   * is created by setting the loop_timer variable to +4000 microseconds
   * every loop.
   */
  while (loop_timer > micros());
  loop_timer += LOOP_MS * 1000;
}

