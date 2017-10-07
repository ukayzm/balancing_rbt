#include "mpu6050.h"


#define LOOP_MS     10
unsigned long loop_timer;

void setup() {
  // put your setup code here, to run once:

  Serial.begin(115200);
  mpu6050_setup();

  /*Set the loop_timer variable at the next end loop time */
  loop_timer = micros() + LOOP_MS * 1000;

}

void loop()
{
  char buf[10];

  mpu6050_loop();

#if 1
  Serial.print(get_last_time());
  Serial.print(F(",\t"));
  Serial.print(get_last_x_angle(), 2);
  Serial.print(F(",\t"));
  Serial.print(get_last_y_angle(), 2);
  Serial.print(F(",\t"));
  Serial.print(get_last_z_angle(), 2);
  Serial.print(F(",\t"));
#if 1
  Serial.print(get_yaw_angle(), 2);
  Serial.print(F(",\t"));
  Serial.print(get_pitch_angle(), 2);
  Serial.print(F(",\t"));
  Serial.print(get_roll_angle(), 2);
  Serial.print(F(",\t"));
#endif

  Serial.print(get_last_gyro_x_angle(), 2);
  Serial.print(F(",\t"));
  Serial.print(get_last_gyro_y_angle(), 2);
  Serial.print(F(",\t"));
  Serial.print(get_last_gyro_z_angle(), 2);
  Serial.println(F(""));
#endif

  /*
   * The angle calculations are tuned for a loop time of LOOP_MS milliseconds.
   * To make sure every loop is exactly LOOP_MS milliseconds a wait loop
   * is created by setting the loop_timer variable to +4000 microseconds
   * every loop.
   */
  while (loop_timer > micros());
  loop_timer += LOOP_MS * 1000;
}

