#include "Arduino.h"
#include "wheel.h"


#define M3_INTR_PIN 2
#define M3_DIR_PIN  A0
#define M4_INTR_PIN 3
#define M4_DIR_PIN  A1


int16_t count_m3, count_m4;
unsigned long total_count_m3, total_count_m4;

void on_intr_m3(void)
{
	if (digitalRead(M3_DIR_PIN))
		count_m3++;
	else
		count_m3--;
	total_count_m3++;
}

void on_intr_m4(void)
{
	if (digitalRead(M4_DIR_PIN))
		count_m4--;
	else
		count_m4++;
	total_count_m4++;
}

CalibData calib_data_m3[NUM_CDATA] = {
	{ -260, -321.70 },
	{ -250, -315.10 },
	{ -240, -299.69 },
	{ -230, -297.49 },
	{ -220, -290.70 },
	{ -210, -286.90 },
	{ -200, -281.53 },
	{ -190, -275.12 },
	{ -180, -267.71 },
	{ -170, -260.11 },
	{ -160, -251.12 },
	{ -150, -240.72 },
	{ -140, -228.72 },
	{ -130, -215.12 },
	{ -120, -198.93 },
	{ -110, -179.94 },
	{ -100, -157.14 },
	{ -90, -131.76 },
	{ -80, -102.36 },
	{ -70, -69.18 },
	{ -60, -32.59 },
	{ -50, -1.20 },
	{ -40, -0.00 },
	{ -30, -0.00 },
	{ -20, -0.00 },
	{ -10, -0.00 },
	{ 0, 0.00 },
	{ 10, 0.00 },
	{ 20, 0.00 },
	{ 30, 0.00 },
	{ 40, 0.00 },
	{ 50, 1.20 },
	{ 60, 17.19 },
	{ 70, 35.79 },
	{ 80, 89.17 },
	{ 90, 130.36 },
	{ 100, 155.76 },
	{ 110, 176.74 },
	{ 120, 196.14 },
	{ 130, 212.73 },
	{ 140, 226.12 },
	{ 150, 238.12 },
	{ 160, 248.31 },
	{ 170, 257.91 },
	{ 180, 265.31 },
	{ 190, 272.52 },
	{ 200, 279.11 },
	{ 210, 284.51 },
	{ 220, 289.51 },
	{ 230, 296.12 },
	{ 240, 298.51 },
	{ 250, 314.09 },
	{ 260, 325.47 },
};

CalibData calib_data_m4[NUM_CDATA] = {
	{ -260, -313.12 },
	{ -250, -306.29 },
	{ -240, -289.10 },
	{ -230, -287.71 },
	{ -220, -280.91 },
	{ -210, -276.51 },
	{ -200, -270.52 },
	{ -190, -263.91 },
	{ -180, -256.71 },
	{ -170, -248.71 },
	{ -160, -239.52 },
	{ -150, -228.53 },
	{ -140, -216.13 },
	{ -130, -201.54 },
	{ -120, -185.14 },
	{ -110, -166.54 },
	{ -100, -143.95 },
	{ -90, -118.96 },
	{ -80, -84.77 },
	{ -70, -31.99 },
	{ -60, -15.60 },
	{ -50, -0.80 },
	{ -40, -0.00 },
	{ -30, -0.00 },
	{ -20, -0.00 },
	{ -10, -0.00 },
	{ 0, 0.00 },
	{ 10, 0.00 },
	{ 20, 0.00 },
	{ 30, 0.00 },
	{ 40, 0.00 },
	{ 50, 1.40 },
	{ 60, 18.99 },
	{ 70, 36.19 },
	{ 80, 51.78 },
	{ 90, 64.98 },
	{ 100, 124.56 },
	{ 110, 167.74 },
	{ 120, 186.14 },
	{ 130, 202.54 },
	{ 140, 217.13 },
	{ 150, 229.13 },
	{ 160, 239.32 },
	{ 170, 248.51 },
	{ 180, 256.72 },
	{ 190, 264.11 },
	{ 200, 270.31 },
	{ 210, 277.11 },
	{ 220, 281.30 },
	{ 230, 288.11 },
	{ 240, 289.91 },
	{ 250, 305.89 },
	{ 260, 317.52 },
};

void init_encoders(void)
{
	pinMode(M3_INTR_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(M3_INTR_PIN), on_intr_m3, FALLING);
	pinMode(M3_DIR_PIN, INPUT_PULLUP);
	count_m3 = 0;

	pinMode(M4_INTR_PIN, INPUT_PULLUP);
	attachInterrupt(digitalPinToInterrupt(M4_INTR_PIN), on_intr_m4, FALLING);
	pinMode(M4_DIR_PIN, INPUT_PULLUP);
	count_m4 = 0;
}

Wheel::Wheel(int16_t *counter, unsigned long *total_counter, CalibData cdata[NUM_CDATA])
{
	pCounter = counter;
	pulTotalCounter = total_counter;
	pPid = new PID(&fCurSpeed, &fPidOutput, &fTgtSpeed, KP, KI, KD, DIRECT);
	pPid->SetSampleTime(12);
	pPid->SetMode(AUTOMATIC);
	pPid->SetOutputLimits(-255, 255);

	pstCdata = cdata;
}

Wheel::~Wheel(void)
{
	delete pPid;
}

void Wheel::AttachMotor(AF_DCMotor *motor)
{
	pMotor = motor;
	setMotorPwm(0);
	pPid->SetMode(MANUAL);
}

void Wheel::SetSpeed(int16_t speed)
{
	nTgtPwm = 0;
	fTgtSpeed = speed;
	nPwmBias = 0;
	if (speed == 0) {
		nPwmBias = 0;
	} else if (speed <= -MAX_MMPS) {
		nPwmBias = -255;
	} else if (speed >= MAX_MMPS) {
		nPwmBias = 255;
	} else if (pstCdata) {
		/* use calibrate data for bias */
		int i;
		for (i = 0; i < NUM_CDATA - 1; i++) {
			if (pstCdata[i].speed <= speed && speed <= pstCdata[i+1].speed) {
				nPwmBias = (pstCdata[i].pwm + pstCdata[i+1].pwm) / 2;
				break;
			}
		}
	}
	if (speed != 0 && nPwmBias == 0) {
		nPwmBias = fTgtSpeed * 255.0 / MAX_MMPS;
	}
	pPid->SetMode(AUTOMATIC);
}

void Wheel::SetPwm(int16_t pwm)
{
	nTgtPwm = pwm;
	fTgtSpeed = 0;
	pPid->SetMode(MANUAL);
}

void Wheel::Loop(void)
{
	unsigned long cur_us = micros();
	unsigned long cur_TotalCounter;
	int i, num_intr;

	for (i = NUM_INTR_SAVE - 1; i > 0; i--) {
		anIntr[i] = anIntr[i-1];
		aulInterval[i] = aulInterval[i-1];
	}
	anIntr[0] = *pCounter;
	*pCounter = 0;
	aulInterval[0] = cur_us - ulLastLoopUs;
	cur_TotalCounter = *pulTotalCounter;

	/* calculate current speed of wheel */
	num_intr = 0;
	ulIntervalUs = 0;
	for (i = 0; i < NUM_INTR_SAVE; i++) {
		num_intr += anIntr[i];
		ulIntervalUs += aulInterval[i];
	}
	ulIntervalUs /= NUM_INTR_SAVE;
	if (ulIntervalUs && num_intr) {
		fCurSpeed = (float)(num_intr * 1000.0 * MM_PER_INTR) / NUM_INTR_SAVE / (float)ulIntervalUs * 1000.0;
	} else {
		fCurSpeed = 0;
	}

	if (pPid->GetMode() == AUTOMATIC) {
		/* PID control */
		pPid->Compute();
		setMotorPwm(fPidOutput + nPwmBias);
	} else {
		setMotorPwm(nTgtPwm);
	}

	ulLastLoopUs = cur_us;

	/* diagnostic print */
	if (bDiag) {
		if (ulTotalCounter != cur_TotalCounter || fTgtSpeed != 0 || nTgtPwm) {
			Print();
		}
	}

	ulTotalCounter = cur_TotalCounter;
}

void Wheel::setMotorPwm(int16_t pwm)
{
	if (pwm > 255)
		pwm = 255;
	if (pwm < -255)
		pwm = -255;
	/* dumb speed up */
	if (pwm > 0) {
		pMotor->run(FORWARD);
	} else if (pwm < 0) {
		pMotor->run(BACKWARD);
	} else {
		pMotor->run(RELEASE);
	}
	nCurPwm = pwm;
	if (pwm < 0)
		pwm = -pwm;
	pMotor->setSpeed(pwm);
}

void Wheel::Print(void)
{
	int i;

	Serial.print(ulLastLoopUs); Serial.print(" us");
	Serial.print(", intr "); Serial.print(anIntr[0]);
	Serial.print(", Kp "); Serial.print(pPid->GetKp(), 2);
	Serial.print(", Ki "); Serial.print(pPid->GetKi(), 2);
	Serial.print(", Kd "); Serial.print(pPid->GetKd(), 2);
	Serial.print(", tgt "); Serial.print(fTgtSpeed); Serial.print(" mm/s");
	Serial.print(", cur "); Serial.print(fCurSpeed); Serial.print(" mm/s");
	Serial.print(", out "); Serial.print(nPwmBias);
	Serial.print(" + "); Serial.print(fPidOutput);
	Serial.print("= PWM "); Serial.print(nCurPwm);
	Serial.print(", "); Serial.print(nTgtPwm); Serial.print(", ");
	Serial.println("");
}

#define LOOP_MS     10

unsigned long Wheel::nextDelay(unsigned long micro)
{
  /*
   * The angle calculations are tuned for a loop time of LOOP_MS milliseconds.
   * To make sure every loop is exactly LOOP_MS milliseconds a wait loop
   * is created by setting the loop_timer variable to +4000 microseconds
   * every loop.
   */
  while (micro > micros());
  return micros() + LOOP_MS * 1000;
}

double Wheel::measure(void)
{
	int i;
	unsigned long loop_timer, prev_us;
	unsigned long prev_intr;
	double speed;

	prev_intr = ulTotalCounter;
	loop_timer = nextDelay(0);
	prev_us = loop_timer;

	for (i = 0; i < 100; i++) {
		Loop();
		loop_timer = nextDelay(loop_timer);
	}
	speed = (double)(ulTotalCounter - prev_intr) * MM_PER_INTR * 1000 / ((double)(loop_timer - prev_us) / 1000);

	return speed;
}

void Wheel::Calibrate(void)
{
	int16_t pwm;
	double speed;
	int i;
	CalibData cdata[NUM_CDATA];

	bDiag = 0;

	/* forward */
	SetPwm(-260);
	nextDelay(nextDelay(0));

	for (pwm = -260, i = 0; pwm <= 260; pwm += 10, i++) {
		SetPwm(pwm);
		Serial.print("pwm ");	Serial.print(pwm);	Serial.print(": ");
		speed = measure();
		Serial.print(speed, 2); Serial.println(" mm/s");
		cdata[i].pwm = pwm;
		cdata[i].speed = speed;
	}

	/* backward */
	for (pwm = 260, i = NUM_CDATA - 1; pwm >= -260; pwm -= 10, i--) {
		SetPwm(pwm);
		Serial.print("pwm ");	Serial.print(pwm);	Serial.print(": ");
		speed = measure();
		Serial.print(speed, 2); Serial.println(" mm/s");
		cdata[i].speed = (cdata[i].speed + speed) / 2;
	}

	SetPwm(0);
	Loop();

	/* print calibrate data */
	for (i = 0; i < NUM_CDATA; i++) {
		Serial.print("\t{ ");
		Serial.print(cdata[i].pwm);
		Serial.print(", ");
		Serial.print(cdata[i].speed, 2);
		Serial.println(" },");
	}
}

void Wheel::Calibrate2(void)
{
	int16_t pwm;
	double speed;

	SetPwm(0);
	/*Set the loop_timer variable at the next end loop time */

	bDiag = 0;

	/* forward, incremental */
	for (pwm = 50; pwm <= 80; pwm += 1) {
		SetPwm(pwm);
		Serial.print("pwm ");	Serial.print(pwm);	Serial.print(": ");
		speed = measure();
		Serial.print(speed, 2); Serial.println(" mm/s");
	}
	/* forward, decremental */
	for (pwm = 80; pwm >= 50; pwm -= 1) {
		SetPwm(pwm);
		Serial.print("pwm ");	Serial.print(pwm);	Serial.print(": ");
		speed = measure();
		Serial.print(speed, 2); Serial.println(" mm/s");
	}

	/* backward, incremental */
	for (pwm = -50; pwm >= -80; pwm -= 1) {
		SetPwm(pwm);
		Serial.print("pwm ");	Serial.print(pwm);	Serial.print(": ");
		speed = measure();
		Serial.print(speed, 2); Serial.println(" mm/s");
	}

	/* backward, decremental */
	for (pwm = -80; pwm <= -50; pwm += 1) {
		SetPwm(pwm);
		Serial.print("pwm ");	Serial.print(pwm);	Serial.print(": ");
		speed = measure();
		Serial.print(speed, 2); Serial.println(" mm/s");
	}
}

