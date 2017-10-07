#ifndef __MPU6050_H__
#define __MPU6050_H__

#define COMPLEMENTARY_ALPHA		0.98

extern void mpu6050_setup(void);
extern void mpu6050_loop(void);
extern unsigned long get_last_time();
extern float get_last_x_angle();
extern float get_last_y_angle();
extern float get_last_z_angle();
extern float get_last_gyro_x_angle();
extern float get_last_gyro_y_angle();
extern float get_last_gyro_z_angle();

extern float get_yaw_angle(void);
extern float get_pitch_angle(void);
extern float get_roll_angle(void);


#endif // __MPU6050_H__
