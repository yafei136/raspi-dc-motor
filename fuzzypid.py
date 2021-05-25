import time
import matplotlib.pyplot as plt
import numpy as np
import RPi.GPIO as io
from myPID import PID
from motorL298N1 import MotorL298N
from fuzzyCompute import *

pinL298NIn = [31, 33]
pinEncoder = [35, 37]
EncoderNum = 0


motor = MotorL298N(pinL298NIn, pinEncoder)

t = []
pos = []
pidOutList = []


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
        pidPosition = PID(1, 0, 0)
        pidPosition.output_limits = (-100, 100)
        pidPosition.setpoint = dest

        pLast = motor.getPosition()*6.28/11
        isStoppedCounter = 0
        timeStart = time.perf_counter()
        while True:
            timeNow = time.perf_counter()
            motor.setDirection(1)

            pNow = motor.getPosition()*6.28/11
            pidOut = pidPosition(pNow)
            # print(pidPosition.Kp, pidPosition.Ki, pidPosition.Kd)

            
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
            pidOutList.append(pidOut)
            time.sleep(0.03)
            pLast = pNow
            # print(motor.getPosition()/11/20.4)




# motorPositionCtrl(1100)
motorPositionCtrl1(1*6.28*20.4)
# motorSpeedCtrl(200)
plt.plot(t, pos)
plt.plot(t, np.ones(len(t)))
# plt.plot(t, pidOutList)
plt.show()
io.cleanup()