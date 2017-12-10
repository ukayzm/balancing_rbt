#!/usr/bin/env python3

import serial
import numpy as np
import matplotlib.pyplot as plt
import time

numX = 600
TgtSpeed = 0
TgtAngle = 0
Kp_angle = 0
Ki_angle = 0
Kd_angle = 0
Kp_speed = 0
Ki_speed = 0
Kd_speed = 0

X = np.linspace(-numX, 0, numX)

# angle to PWM
tgtAngle = np.zeros(numX)
curAngle = np.zeros(numX)
curAngle.fill(10)
pwm = np.zeros(numX)
pwm.fill(100)

# PWM to RPM
tgtRpm = np.zeros(numX)
curRpm = np.zeros(numX)

f, ax = plt.subplots(3, sharex=True, figsize=(12,10), dpi=80)
#plt.ion()

curAngleY, = ax[0].plot(X, curAngle, 'b-')
tgtAngleY, = ax[0].plot(X, tgtAngle, 'r')
ax[0].set_ylim(-180, 180)
ax[0].set_ylabel('Pitch angle (degrees)')

ax20 = ax[0].twinx()
ax20.set_ylim(-260, 260)
pwmY1, = ax20.plot(X, pwm, 'b.')
ax20.set_ylabel('PWM')

ax[1].set_ylim(-260, 260)
pwmY2, = ax[1].plot(X, pwm, 'b.')
ax[1].set_ylabel('PWM')

ax21 = ax[1].twinx()
ax21.plot(X, curRpm, 'b-')
ax21.plot(X, tgtRpm, 'r')
ax21.set_ylim(-300, 300)
ax21.set_ylabel('RPM')

plt.tight_layout()

def plot_k(k):
    kk = k[1:]
    ff = [float(i) for i in kk]
    TgtSpeed = ff[0]
    TgtAngle = ff[1]
    Kp_speed = ff[2]
    Ki_speed = ff[3]
    Kd_speed = ff[4]
    Kp_angle = ff[5]
    Ki_angle = ff[6]
    Kd_angle = ff[7]
    str_k_angle = "Angle Kp %0.2f, Ki %0.2f, Kd %0.2f" % (
        Kp_angle, Ki_angle, Kd_angle)
    ax[0].set_title(str_k_angle)
    str_k_speed = "Speed Kp %0.2f, Ki %0.2f, Kd %0.2f" % (
        Kp_speed, Ki_speed, Kd_speed)
    ax[1].set_title(str_k_speed)


ser = serial.Serial('/dev/ttyACM0', 115200)
last_draw_sec = int(time.time())

while True:
    s = ser.readline().decode('ISO-8859-1').rstrip('\n').rstrip('\r')
    g = s.split(",")
    if (g[0] is "P" and len(g) is 12):
        pp = g[1:]
        ff = [float(i) for i in pp]
        print(ff)
        curAngle = np.roll(curAngle, -1)
        curAngle[-1] = ff[1]
        tgtAngle = np.roll(tgtAngle, -1)
        tgtAngle[-1] = TgtAngle
        pwm = np.roll(pwm, -1)
        pwm[-1] = ff[2]
        cur_sec = int(time.time())
        if (last_draw_sec != cur_sec):
            curAngleY.set_ydata(curAngle)
            tgtAngleY.set_ydata(tgtAngle)
            pwmY1.set_ydata(pwm)
            plt.draw()
            plt.pause(0.001)
            last_draw_sec = cur_sec
    elif (g[0] is "K" and len(g) is 9):
        plot_k(g)
        continue
    else:
        print(s)
        continue

ser.close()
