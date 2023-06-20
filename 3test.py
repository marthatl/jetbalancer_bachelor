from __future__ import print_function
import qwiic_icm20948
import time
import sys
import numpy
from functools import reduce

try:
    IMU = qwiic_icm20948.QwiicIcm20948()
    IMU.setSampleMode(1, 0x00)
    IMU.enableDlpfGyro(False)

    IMU.begin()

    timeRead = []
    rawData = []
    for i in range(1000):
        if IMU.dataReady():
            IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
            timeRead.append(round(time.time()*1000))
            rawData.append((IMU.axRaw, IMU.azRaw, IMU.gyRaw))
        else:
            print("Waiting for data")

except (KeyboardInterrupt, SystemExit) as exErr:
    print("\nEnding Example 1")
    sys.exit(0)

print("Antall målinger \ntimeRead: ", len(timeRead), "rawData: ", len(rawData))

#print(timeA)
maks = 0
mins = 1000
avg = 0

diff = map(lambda x,y: y-x,
        timeRead,
        timeRead[1:])

for d in diff:
    avg += d
    maks = max(maks, d)
    mins = min(mins, d)      
avg /= len(timeRead)

print("Analyse av time:")
print("Avg: " , avg, "\nMaks: " , maks, "\nMin: ", mins)
