#!/usr/bin/env python3

import serial
import numpy as np
import matplotlib.pyplot as plt

numMeasurements = 100

X = np.linspace(-numMeasurements, 0, numMeasurements)
rollMeasurements = np.zeros(numMeasurements)
speedMeasurements = np.zeros(numMeasurements)
setpoints = np.empty(numMeasurements)
setpoints.fill(180)

plt.subplot(211)
rollGraph = plt.plot(X, rollMeasurements)[0]
setpointGraph = plt.plot(X, setpoints, color='r')[0]
plt.ylim(-30, 30)
plt.ylabel('Roll (degrees)')

plt.subplot(212)
speedGraph = plt.plot(X, speedMeasurements, color='g')[0]
plt.ylim(-300, 300)
plt.ylabel('Motor Speed')
plt.xlabel('Iterations')

ser = serial.Serial('/dev/ttyACM0', 115200)

while True:
    s = ser.readline().decode('ISO-8859-1').rstrip('\n').rstrip('\r')
    g = s.split(",")
    if (len(g) is not 17):
        print(s, "\n")
        continue

    try:
        ff = [float(i) for i in g]
    except:
        print(s, "\n")
        continue

    print(ff)
    roll = ff[3]
    setpoint = ff[2]
    motorSpeed = ff[9]

    rollMeasurements = np.roll(rollMeasurements, -1)
    rollMeasurements[-1] = roll

    speedMeasurements = np.roll(speedMeasurements, -1)
    speedMeasurements[-1] = motorSpeed

    rollGraph.set_ydata(rollMeasurements)
    speedGraph.set_ydata(speedMeasurements)
    plt.draw()
    plt.pause(0.01)

ser.close()
