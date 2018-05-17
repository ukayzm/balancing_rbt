#ifndef __PID_H__
#define __PID_H__

#include "Arduino.h"

class Pid
{
public:
	Pid(float p, float i, float d, float iLimit, unsigned long typical_interval_usec);
	float updatePID(float target, float current);
	void resetPID();

	void setKp(float p);
	void setKi(float i);
	void setKd(float d);
	void setIlimit(float limit);

	float getKp();
	float getKi();
	float getKd();
	float getIlimit();

	float pterm, iterm, dterm;

private:
	float Kp;
	float Ki;
	float Kd;
	float iterm_limit;
	float integrated_error;
	float last_error;
	float last_input;
	float max_interval_sec;
	unsigned long last_usec;
};

#endif
