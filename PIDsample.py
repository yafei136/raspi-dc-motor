import time
import RPi.GPIO as io
from simple_pid import PID
import matplotlib.pyplot as plt
import numpy as np
from MotorL298N import MotorL298N



pinL298NIn = [31, 33]
pinEncoder = 37
EncoderNum = 0

motor = MotorL298N(pinL298NIn, pinEncoder)

t = []
pos = []

def outToIn(pidOut):
    if pidOut>=0:
        motor.setDirection(1)
        motor.updatePWM(pidOut)
    else:
        motor.setDirection(-1)
        motor.updatePWM(-pidOut)

def motorPositionCtrl1(dest):
    # Position Control
    currentPosition = motor.getPosition()
    if abs(dest-currentPosition)<2:
        return True
    else:
        pLast = motor.getPosition()*6.28/11
        isStoppedCounter = 0
        timeStart = time.perf_counter()
        while True:
            timeNow = time.perf_counter()
            motor.setDirection(1)
            pidPosition = PID(0.5, 0, -0.1)
            pidPosition.output_limits = (-100, 100)
            pidPosition.setpoint = dest
            pNow = motor.getPosition()*6.28/11
            pidOut = pidPosition(pNow)
            outToIn(pidOut)

            if pLast==pNow:
                isStoppedCounter += 1
            else:
                isStoppedCounter = 0

            if isStoppedCounter > 5:
                motor.setDirection(0)
                motor.updatePWM(0)
                break
            t.append(timeNow - timeStart)
            pos.append(pNow/6.28/20.4)
            time.sleep(0.01)
            pLast = pNow
            #print(motor.getPosition()/11/20.4)

def motorSpeedCtrl(dest):
    # Speed control
    motor.setDirection(1)

    pidSpeed = PID(1, 0.02, 0)
    pidSpeed.output_limits = (0, 100)
    pidSpeed.setpoint = dest

    pLast = motor.getPosition()
    timeLast = time.perf_counter()
    timeStart = timeLast
    countLast = 0
    while True:
        timeNow = time.perf_counter()
        pNow = motor.getPosition()
        dtime = timeNow - timeLast
        speed = (pNow - pLast)/dtime
        if (dtime>5e-3):
            pidOut = pidSpeed(speed)
            motor.updatePWM(pidOut)
            time.sleep(0.01)
        
        countNow = int(timeNow - timeStart)
        if (countNow - countLast) == 1:
            print(speed)
        countLast = countNow
        # print(speed)


# motorPositionCtrl(1100)
# motorPositionCtrl1(1*6.28*20.4)
motorSpeedCtrl(200)
plt.plot(t, pos)
plt.plot(t, np.ones(len(t)))
plt.show()
io.cleanup()