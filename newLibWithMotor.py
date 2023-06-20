import time
import board
import adafruit_icm20x as ada
import numpy as np
from jetbot import Robot
from IPython.display import clear_output

i2c = board.I2C()
icm = ada.ICM20948(i2c)
icm.gyro_range = ada.GyroRange.RANGE_250_DPS
icm.accelerometer_data_rate_divisor = 0 # maximum
icm.gyro_data_rate_divisor = 0
robot = Robot()

#integral = 0
#preTime = round(time.time()*1000)
#curTime = preTime
#preAcc = 0
print("done")
# End of first container

def rad2mdegF(radians):
    return radians * 180000 / np.pi

def rad2mdegI(radians):
    return np.intc(radians) * 57295 #57295 = 180 000 / pi

def PID(curPitch, deltaTime):
    c = 1/40000 # Fant ved prøving og feiling
    kP, kI, kD = 1, 1.3, 0.017
    #kP, kI, kD = 0, 0, 1
    
    offsetPitch = 4800 # loddrett pitch ~= 95 grader --> offset ~5 grader
    reffPitch = 4800
    
    error = curPitch - reffPitch 
    if abs(error)>25000:#give up
        PID.integral = 0
        return 0
    #else:
    PID.integral += error * deltaTime
    derivative = (error - PID.preError)/deltaTime
    
    p = error * kP
    i = PID.integral * kI
    d = derivative * kD

    #PID.preTime = curTime
    PID.preError = error
    return (p + i + d)*c

PID.integral = 0
#PID.preTime = 0
PID.preError = 0

def updatePitch(imu, resetIntegral = 0):
    if (resetIntegral):
        integral = 0
    gy = imu.gyro[1]
    acc = np.arctan2(imu.acceleration[0], imu.acceleration[2])*180000/np.pi
    integral += gy*deltaTime
    pitch = acc*0.02 + ((gy*deltaTime + prePitch)*0.98)
    return pitch

def caliGyrY(numLoops):
    """
    axis = (0 = X-axis), (1 = Y-axis), (2 = Z-axis)
    """
    bias = 0
    for i in range(numLoops):
        bias += rad2mdegF(icm.gyro[1])
    return bias/numLoops

# End of second container

integral = 0
maxSpeed = 0.7
acc_data = icm.acceleration
acc = rad2mdegF(- np.arctan2(acc_data[2], acc_data[0]))
preAcc = 0
preGy = 0
# calibrate for bias
bias = caliGyrY(1000)
preTime = time.time()
prePitch = rad2mdegF(- np.arctan2(acc_data[2], acc_data[0]))
time.sleep(1)
#PID.preTime = np.intc(time.time() * 1000)
#while True:
for i in range(500):
    curTime = time.time()
    deltaTime = (curTime - preTime) # sekunder
    try:
        gy = (rad2mdegF(- icm.gyro[1]) - bias)*deltaTime # milligrader
        acc_data = icm.acceleration
        acc = rad2mdegF(- np.arctan2(acc_data[2], acc_data[0])) # milligrader
    except:
        gy = preGy
        acc = preAcc
    # list.append(gy)
    #integral += gy*deltaTime # milligrader/sec * sec = milligrader
    
    pitch = (acc*0.02 + (gy + prePitch)*0.98) # grader + (grader[/sec * sec] + grader?) 
    
    mspeed = PID(pitch, deltaTime)
    
    if (abs(mspeed) > maxSpeed):
        if(mspeed > 0):
            mspeed = maxSpeed
        else:
            mspeed = -maxSpeed
    robot.set_motors(-mspeed*1.02, -mspeed)
    
    clear_output(wait=True)
    print("pitch ", pitch, "\n acc ", acc, "\n gy ", gy, "\n delta", deltaTime)
    print("bias: ", bias, " prePitch ", prePitch)
    print("motor: ", mspeed, "\n PID.preError ", PID.preError, "\n PID.inte ", PID.integral)
    
    preTime = curTime
    prePitch = pitch
    preAcc = acc
    preGy = gy
robot.stop()

# End of third container

#180000 // np.pi 22435
robot.set_motors(0.25*1.02, 0.25)
#time.sleep(2)
#robot.stop()
print("something")

# End of forth container

robot.stop()

# End of fifth container

