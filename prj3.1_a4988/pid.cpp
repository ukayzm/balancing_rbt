#include "Arduino.h"
#include "pid.h"

Pid::Pid(float p, float i, float d, float i_limit, unsigned long typical_interval_usec)
{
	Kp = p; 
	Ki = i; 
	Kd = d; 
	iterm_limit = i_limit; 
	integrated_error = 0; 
	last_error = 0; 
	max_interval_sec = (float)typical_interval_usec / 1000000.0 * 3;
}

float Pid::updatePID(float target, float current)
{
	unsigned long usec = micros();
	float pterm, iterm, dterm, error;
	float dt = (float)(usec - last_usec) / 1000000.0;

	if (dt > max_interval_sec) {
		last_usec = usec;
		last_error = 0;
		integrated_error = 0; 
		return 0;
	}
	error = (target - current) * dt; 

	pterm = Kp * error;

	integrated_error += error;    
	integrated_error = constrain(integrated_error, -iterm_limit, iterm_limit);
	iterm = Ki * integrated_error; 

	pterm = Kd * (error - last_error);

	last_error = error;
	last_usec = usec;

	return (pterm + iterm + pterm); 
}

void Pid::resetPID()
{
	integrated_error = 0; 
	last_error = 0; 
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

