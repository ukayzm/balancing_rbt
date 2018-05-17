#include "Arduino.h"
#include "pid.h"

Pid::Pid(float p, float i, float d, float i_limit, unsigned long typical_interval_usec)
{
	Kp = p; 
	Ki = i; 
	Kd = d; 
	iterm_limit = i_limit; 
	max_interval_sec = (float)typical_interval_usec / 1000000.0 * 3;
	resetPID();
}

float Pid::updatePID(float target, float current)
{
	unsigned long usec = micros();
	float error;
	float dt = (float)(usec - last_usec) / 1000000.0;

	if (dt > max_interval_sec) {
		last_usec = usec;
		last_error = 0;
		last_input = 0;
		integrated_error = 0; 
		return 0;
	}

#if 1
	error = target - current;

	pterm = Kp * error;
	if (1 < error) {
		//pterm *= abs(error);
	}
	integrated_error += Ki * error * dt;
	iterm = constrain(integrated_error, -iterm_limit, iterm_limit);

	dterm = -Kd * (current - last_input) / dt;
#else
	error = (target - current) * dt; 

	pterm = Kp * error;

	integrated_error += error;    
	integrated_error = constrain(integrated_error, -iterm_limit, iterm_limit);
	iterm = Ki * integrated_error; 

	dterm = Kd * (error - last_error);
#endif

	last_input = current;
	last_error = error;
	last_usec = usec;

	return (pterm + iterm + dterm); 
}

void Pid::resetPID()
{
	integrated_error = 0;
	last_error = 0;
	last_input = 0;
}

void Pid::setKp(float p)
{
	Kp = p; 
}

void Pid::setKi(float i)
{
	Ki = i; 
} 

void Pid::setKd(float d)
{
	Kd = d; 
}

void Pid::setIlimit(float limit)
{
	iterm_limit = limit; 
} 

float Pid::getKp()
{
	return Kp;  
}

float Pid::getKi()
{
	return Ki;  
} 

float Pid::getKd()
{
	return Kd;  
}

float Pid::getIlimit()
{
	return iterm_limit;  
}     

