#include <Arduino.h>
#include "board.h"
#include "mpu6050.h"
#include "pid.h"


float fTgtSpeed, fTgtAngle;
float fCurAngle, fSetSpeed;
int32_t nDir;

Pid AnglePid(ANGLE_KP, ANGLE_KI, ANGLE_KD, 500, LOOP_USEC);
Pid SpeedPid(SPEED_KP, SPEED_KI, SPEED_KD, 500, LOOP_USEC);

extern void motor_set_rpm(int16_t rpm);

void balancing_print_k(void)
{
	Serial.print("K,"); Serial.print(fTgtSpeed, 2);
	Serial.print(",\t"); Serial.print(fTgtAngle, 2);
	Serial.print(",\t"); Serial.print(SpeedPid.getKp(), 1);
	Serial.print(",\t"); Serial.print(SpeedPid.getKi(), 1);
	Serial.print(",\t"); Serial.print(SpeedPid.getKd(), 1);
	Serial.print(",\t"); Serial.print(AnglePid.getKp(), 1);
	Serial.print(",\t"); Serial.print(AnglePid.getKi(), 1);
	Serial.print(",\t"); Serial.print(AnglePid.getKd(), 1);
	Serial.println("");
}

void balancing_print(void)
{
	static unsigned long last_ms;
	unsigned long cur_ms = millis();

	if (last_ms / 1000 != cur_ms / 1000) {
		balancing_print_k();
	}
	Serial.print("P,"); Serial.print(cur_ms - last_ms);
	//Serial.print(",\tTgtSpeed "); Serial.print(fTgtSpeed, 2);
	//Serial.print(",\t"); Serial.print(SpeedPid.getKp(), 2);
	//Serial.print(",\t"); Serial.print(SpeedPid.getKi(), 2);
	//Serial.print(",\t"); Serial.print(SpeedPid.getKd(), 2);
	//Serial.print(",\t"); Serial.print(SpeedPid.pterm, 2);
	//Serial.print(",\t"); Serial.print(SpeedPid.iterm, 2);
	//Serial.print(",\t"); Serial.print(SpeedPid.dterm, 2);
	//Serial.print(",\tTgtAngle "); Serial.print(fTgtAngle, 2);
	Serial.print(",\t"); Serial.print(fCurAngle, 2);
	//Serial.print(",\t"); Serial.print(AnglePid.getKp(), 1);
	//Serial.print(",\t"); Serial.print(AnglePid.getKi(), 1);
	//Serial.print(",\t"); Serial.print(AnglePid.getKd(), 1);
	//Serial.print(",\t"); Serial.print(AnglePid.pterm, 1);
	//Serial.print(",\t"); Serial.print(AnglePid.iterm, 1);
	//Serial.print(",\t"); Serial.print(AnglePid.dterm, 1);
	Serial.print(",\t"); Serial.print(fSetSpeed, 0);
	Serial.println("");
	last_ms = cur_ms;
}

void balancing_setup()
{
}

void balancing_loop()
{
	fCurAngle = get_pitch_angle();
	if (fCurAngle < -30 || 30 < fCurAngle) {
		motor_set_rpm(0);
		return;
	}

#if 1
	fTgtAngle = SpeedPid.updatePID(fTgtSpeed, fSetSpeed);
#else
	fTgtAngle = 0;
#endif
	fSetSpeed = AnglePid.updatePID(fTgtAngle, fCurAngle);
	motor_set_rpm(fSetSpeed);

	balancing_print();
}

void balancing_inc_tgt(void)
{
	fTgtSpeed += 1;
	balancing_print_k();
}

void balancing_dec_tgt(void)
{
	fTgtSpeed -= 1;
	balancing_print_k();
}

void balancing_inc_dir(void)
{
	nDir += 1;
	balancing_print_k();
}

void balancing_dec_dir(void)
{
	nDir -= 1;
	balancing_print_k();
}

void balancing_reset_tgtdir(void)
{
	nDir = 0;
	fTgtSpeed = 0;
	balancing_print_k();
}

