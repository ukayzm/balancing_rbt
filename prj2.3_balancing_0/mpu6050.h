#ifndef __MPU6050_H__
#define __MPU6050_H__

#define COMPLEMENTARY_ALPHA		0.93

extern void mpu6050_setup(void);
extern void mpu6050_loop(void);
extern float get_pitch_angle(void);

extern double gyroXangle, gyroYangle; // Angle calculate using the gyro only
extern double compAngleX, compAngleY; // Calculated angle using a complementary filter
extern double kalAngleX, kalAngleY; // Calculated angle using a Kalman filter


#endif // __MPU6050_H__
