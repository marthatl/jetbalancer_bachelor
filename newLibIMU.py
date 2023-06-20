import time
import board
import adafruit_icm20x as ada
import numpy as np

i2c = board.I2C()
icm = ada.ICM20948(i2c)
icm.gyro_range = ada.GyroRange.RANGE_250_DPS

maks, min = 0,0
list = []
integral = 0
preMillis = round(time.time()*1000)
curMillis = preMillis
preAcc = 0

while True:
#for i in range(1000):
	curMillis = round(time.time()*1000)
	deltaMillis = curMillis - preMillis
	gy =icm.gyro[1]
	acc = np.arctan2(icm.acceleration[0], icm.acceleration[2])*180/np.pi
#	gy = round(gy, 2)
	#print(gy)
	list.append(gy)
	integral += gy*deltaMillis


	pitch = acc*0.98 + (gy*deltaMillis + preAcc)*0.02


	print("Time: ", deltaMillis, " int: ", pitch)
	#print(gy)
	preMillis = curMillis
	preAcc = acc

print(len(list))
print("Maks; ", maks)
print("Min: ", min)
