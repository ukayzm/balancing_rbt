#!/usr/bin/env python3

import serial
import numpy as np
import matplotlib.pyplot as plt
import time

numX = 600
TgtRpm = 0
TgtAngle = 0
Kp_angle = 0
Ki_angle = 0
Kd_angle = 0
Kp_speed = 0
Ki_speed = 0
Kd_speed = 0

MaxAngle = 20

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

# PWM and PID terms
pTerm = np.zeros(numX)
iTerm = np.zeros(numX)
dTerm = np.zeros(numX)

# Current
currentL = np.zeros(numX)
currentR = np.zeros(numX)

plt.ion()
fig, ax = plt.subplots(4, sharex=True, figsize=(12,10), dpi=80)

# subplot 0
curAngleY, = ax[0].plot(X, curAngle, 'r-')
tgtAngleY, = ax[0].plot(X, tgtAngle, 'g')
ax[0].set_ylim(-MaxAngle - 1, MaxAngle + 1)
ax[0].set_ylabel('Pitch angle (degrees)')

ax20 = ax[0].twinx()
ax20.set_ylim(-260, 260)
pwmY1, = ax20.plot(X, pwm, 'b.')
ax20.set_ylabel('PWM')

# subplot 1
ax[1].set_ylim(-260, 260)
pwmY2, = ax[1].plot(X, pwm, 'b.')
ax[1].set_ylabel('PWM')

ax21 = ax[1].twinx()
curRpmY, = ax21.plot(X, curRpm, 'b-')
tgtRpmY, = ax21.plot(X, tgtRpm, 'r')
ax21.set_ylim(-300, 300)
ax21.set_ylabel('PID')

# subplot 2
pTermY, = ax[2].plot(X, pTerm, 'r-')
iTermY, = ax[2].plot(X, iTerm, 'g-')
dTermY, = ax[2].plot(X, dTerm, 'b-')
ax[2].set_ylim(-260, 260)
ax[2].set_ylabel('PID terms')

# subplot 3
currentLY, = ax[3].plot(X, currentL, 'r-')
currentRY, = ax[3].plot(X, currentL, 'b-')
ax[3].set_ylim(-0.0001, 5000)
ax[3].set_ylabel('Current (mA)')

plt.tight_layout()

fig.show()

def plot_k(k):
    kk = k[1:]
    ff = [float(i) for i in kk]
    print(ff)
    TgtRpm = ff[0]
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
    if (g[0] is "P" and len(g) is 10):
        pp = g[1:]
        ff = [float(i) for i in pp]
        print(ff)
        curAngle = np.roll(curAngle, -1)
        curAngle[-1] = np.clip(-ff[1], -MaxAngle, MaxAngle)
        tgtAngle = np.roll(tgtAngle, -1)
        tgtAngle[-1] = np.clip(TgtAngle, -MaxAngle, MaxAngle)
        pTerm = np.roll(pTerm, -1)
        pTerm[-1] = ff[2]
        iTerm = np.roll(iTerm, -1)
        iTerm[-1] = ff[4]
        dTerm = np.roll(dTerm, -1)
        dTerm[-1] = ff[5]
        pwm = np.roll(pwm, -1)
        pwm[-1] = ff[6]
        #tgtRpm = np.roll(tgtRpm, -1)
        #tgtRpm[-1] = TgtRpm;
        #curRpm = np.roll(curRpm, -1)
        #curRpm[-1] = (ff[9] + ff[10]) / 2
        currentL = np.roll(currentL, -1)
        currentL[-1] = ff[7];
        currentR = np.roll(currentR, -1)
        currentR[-1] = ff[8];
        cur_sec = int(time.time() * 10)
        if (last_draw_sec != cur_sec):
            curAngleY.set_ydata(curAngle)
            tgtAngleY.set_ydata(tgtAngle)
            pwmY1.set_ydata(pwm)
            pwmY2.set_ydata(pwm)
            tgtRpmY.set_ydata(tgtRpm)
            curRpmY.set_ydata(curRpm)
            pTermY.set_ydata(pTerm)
            iTermY.set_ydata(iTerm)
            dTermY.set_ydata(dTerm)
            currentLY.set_ydata(currentL)
            currentRY.set_ydata(currentR)
            plt.draw()
            #plt.pause(0.001)
            fig.canvas.flush_events()
            last_draw_sec = cur_sec
    elif (g[0] is "K" and len(g) is 9):
        plot_k(g)
        plt.draw()
        #plt.pause(0.001)
        fig.canvas.flush_events()
        cur_sec = int(time.time() * 10)
        last_draw_sec = cur_sec
        continue
    else:
        print(s)
        continue

ser.close()
