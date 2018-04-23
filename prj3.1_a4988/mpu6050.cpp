/* Copyright (C) 2012 Kristian Lauszus, TKJ Electronics. All rights reserved.

 This software may be distributed and modified under the terms of the GNU
 General Public License version 2 (GPL2) as published by the Free Software
 Foundation and appearing in the file GPL2.TXT included in the packaging of
 this file. Please note that GPL2 Section 2[b] requires that all works based
 on this software must also be made publicly available under the terms of
 the GPL2 ("Copyleft").

 Contact information
 -------------------

 Kristian Lauszus, TKJ Electronics
 Web      :  http://www.tkjelectronics.com
 e-mail   :  kristianl@tkjelectronics.com
 */

#include "Arduino.h"
#include <Wire.h>
#include "Kalman.h" // Source: https://github.com/TKJElectronics/KalmanFilter
#include "mpu6050.h"

#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf

Kalman kalmanX; // Create the Kalman instances
Kalman kalmanY;

/* IMU Data */
double accX, accY, accZ;
double gyroX, gyroY, gyroZ;
int16_t tempRaw;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter

uint32_t prev_micros;
uint8_t i2cData[14]; // Buffer for I2C data

const int MPU_addr=0x68;
#define MPU6050_CONFIG_1

// TODO: Make calibration routine

void mpu6050_setup() {
	Wire.begin();
#if defined(MPU6050_CONFIG_1)
	TWBR = ((F_CPU / 400000L) - 16) / 2; // Set I2C frequency to 400kHz

	Wire.beginTransmission(MPU_addr);
	Wire.write(0x19);
	Wire.write(7);    // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
	Wire.write(0x00); // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
	Wire.write(0x00); // Set Gyro Full Scale Range to ±250deg/s
	Wire.write(0x00); // Set Accelerometer Full Scale Range to ±2g
	Wire.endTransmission(false);

	/* PLL with X axis gyroscope reference and disable sleep mode */
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x6B);
	Wire.write(0x01);
	Wire.endTransmission(true);
#else
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x6B);
	Wire.write(0);
	Wire.endTransmission(true);
#endif

	delay(100); // Wait for sensor to stabilize

	/* Set kalman and gyro starting angle */
	Wire.beginTransmission(0x68);
	Wire.write(0x3B);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_addr, 6, true);
	accX = Wire.read()<<8 | Wire.read();
	accY = Wire.read()<<8 | Wire.read();
	accZ = Wire.read()<<8 | Wire.read();

	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	double roll  = atan2(accY, accZ) * RAD_TO_DEG;
	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

	kalmanX.setAngle(roll); // Set starting angle
	kalmanY.setAngle(pitch);
	gyroXangle = roll;
	gyroYangle = pitch;
	compAngleX = roll;
	compAngleY = pitch;

	prev_micros = micros();
}

/*
 * output of mpu6050_loop():
 *
 * roll: X angle by accelerometer
 * gyroXangle: X angle by gyroscope
 * compAngleX: X angle by complementary filter
 * kalAngleX: X angle by Kalman filter
 * pitch: Y angle by accelerometer
 * gyroYangle: Y angle by gyroscope
 * compAngleY: Y angle by complementary filter
 * kalAngleY: Y angle by Kalman filter
 */
void mpu6050_loop()
{
	uint32_t now;

	/* Update all the values */
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x3B);
	Wire.endTransmission(false);
	Wire.requestFrom(MPU_addr, 14, true);
	accX = Wire.read()<<8 | Wire.read();
	accY = Wire.read()<<8 | Wire.read();
	accZ = Wire.read()<<8 | Wire.read();
	tempRaw = Wire.read()<<8 | Wire.read();
	gyroX = Wire.read()<<8 | Wire.read();
	gyroY = Wire.read()<<8 | Wire.read();
	gyroZ = Wire.read()<<8 | Wire.read();

	now = micros();
	double dt = (double)(now - prev_micros) / 1000000; // Calculate delta time
	prev_micros = now;

	// Source: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf eq. 25 and eq. 26
	// atan2 outputs the value of -π to π (radians) - see http://en.wikipedia.org/wiki/Atan2
	// It is then converted from radians to degrees
#ifdef RESTRICT_PITCH // Eq. 25 and 26
	double roll  = atan2(accY, accZ) * RAD_TO_DEG;
	double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
	double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
	double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

	double gyroXrate = gyroX / 131.0; // Convert to deg/s
	double gyroYrate = gyroY / 131.0; // Convert to deg/s

#ifdef RESTRICT_PITCH
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((roll < -90 && kalAngleX > 90) || (roll > 90 && kalAngleX < -90)) {
		kalmanX.setAngle(roll);
		compAngleX = roll;
		kalAngleX = roll;
		gyroXangle = roll;
	} else
		kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleX) > 90)
		gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
	kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt);
#else
	// This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
	if ((pitch < -90 && kalAngleY > 90) || (pitch > 90 && kalAngleY < -90)) {
		kalmanY.setAngle(pitch);
		compAngleY = pitch;
		kalAngleY = pitch;
		gyroYangle = pitch;
	} else
		kalAngleY = kalmanY.getAngle(pitch, gyroYrate, dt); // Calculate the angle using a Kalman filter

	if (abs(kalAngleY) > 90)
		gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
	kalAngleX = kalmanX.getAngle(roll, gyroXrate, dt); // Calculate the angle using a Kalman filter
#endif

	gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
	gyroYangle += gyroYrate * dt;
	//gyroXangle += kalmanX.getRate() * dt; // Calculate gyro angle using the unbiased rate
	//gyroYangle += kalmanY.getRate() * dt;

	compAngleX = COMPLEMENTARY_ALPHA * (compAngleX + gyroXrate * dt) + (1 - COMPLEMENTARY_ALPHA) * roll; // Calculate the angle using a Complimentary filter
	compAngleY = COMPLEMENTARY_ALPHA * (compAngleY + gyroYrate * dt) + (1 - COMPLEMENTARY_ALPHA) * pitch;

	// Reset the gyro angle when it has drifted too much
	if (gyroXangle < -180 || gyroXangle > 180)
		gyroXangle = kalAngleX;
	if (gyroYangle < -180 || gyroYangle > 180)
		gyroYangle = kalAngleY;

#if 0
	/* Print Data */
	Serial.print(prev_micros/1000); Serial.print("\t");

#if 0 // Set to 1 to activate
	Serial.print(accX); Serial.print("\t");
	Serial.print(accY); Serial.print("\t");
	Serial.print(accZ); Serial.print("\t");

	Serial.print(gyroX); Serial.print("\t");
	Serial.print(gyroY); Serial.print("\t");
	Serial.print(gyroZ); Serial.print("\t");

	Serial.print("\t");
#endif

	Serial.print(roll); Serial.print("\t");
	Serial.print(gyroXangle); Serial.print("\t");
	Serial.print(compAngleX); Serial.print("\t");
	Serial.print(kalAngleX); Serial.print("\t");

	Serial.print("\t");

	Serial.print(pitch); Serial.print("\t");
	Serial.print(gyroYangle); Serial.print("\t");
	Serial.print(compAngleY); Serial.print("\t");
	Serial.print(kalAngleY); Serial.print("\t");

#if 0 // Set to 1 to print the temperature
	Serial.print("\t");

	double temperature = (double)tempRaw / 340.0 + 36.53;
	Serial.print(temperature); Serial.print("\t");
#endif

	Serial.print("\r\n");
#endif
}

float get_pitch_angle(void)
{
	/* in this robot, Y is pitch. */
	return -kalAngleY - 0.8;
}

