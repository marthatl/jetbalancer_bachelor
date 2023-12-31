from __future__ import print_function
import qwiic_icm20948
import time
import sys
import numpy

def runExample():

	print("\nSparkFun 9DoF ICM-20948 Sensor  Example 1\n")
	IMU = qwiic_icm20948.QwiicIcm20948()
	IMU.setSampleMode(1, 0x00)
	IMU.enableDlpfGyro(False)
	
	if IMU.connected == False:
		print("The Qwiic ICM20948 device isn't connected to the system. Please check your connection", \
			file=sys.stderr)
		return

	IMU.begin()

	while True:
		if IMU.dataReady():
			IMU.getAgmt() # read all axis and temp from sensor, note this also updates all instance variables
			
			#aX = IMU.aXRaw
			acc_pitch = numpy.arctan2(IMU.axRaw, IMU.azRaw)
			print("Acc: ", acc_pitch, " Gyro: ", IMU.gyRaw)
			#aZ = IMU.aZRaw
			#mZ = IMU.mZRaw

			#print(gY)
			time.sleep(0.1)
		else:
			print("Waiting for data")
			time.sleep(0.03)

if __name__ == '__main__':
	try:
		runExample()
	except (KeyboardInterrupt, SystemExit) as exErr:
		print("\nEnding Example 1")
		sys.exit(0)


		#print(\
		# '{: 06d}'.format(IMU.axRaw)\
		#, '\t', '{: 06d}'.format(IMU.ayRaw)\
		#, '\t', '{: 06d}'.format(IMU.azRaw)\
		#, '\t', '{: 06d}'.format(IMU.gxRaw)\
		#, '\t', '{: 06d}'.format(IMU.gyRaw)\
		#, '\t', '{: 06d}'.format(IMU.gzRaw)\
		#, '\t', '{: 06d}'.format(IMU.mxRaw)\
		#, '\t', '{: 06d}'.format(IMU.myRaw)\
		#, '\t', '{: 06d}'.format(IMU.mzRaw)\
		#
