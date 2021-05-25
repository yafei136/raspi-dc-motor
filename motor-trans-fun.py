
import time
import matplotlib.pyplot as plt
import RPi.GPIO as io
from simple_pid import PID
from MotorL298N import MotorL298N

pinL298NIn = [31, 33]
pinEncoder = 35
EncoderNum = 0

motor = MotorL298N(pinL298NIn, pinEncoder)

motor.setDirection(1)

startTime = time.perf_counter()
t = [0]
pos = [0]
speed = 0
motor.updatePWM(100)
for i in range(100):
    time.sleep(0.01)
    t.append(time.perf_counter()-startTime)
    pos.append(motor.getPosition())
plt.plot(t, pos)
motor.setDirection(0)
motor.updatePWM(0)
plt.show()
for tt in t:
    with open("time.txt", "a+") as file1:
        file1.write(str(tt)+"\n")
for p in pos:
    with open("pos.txt", "a+") as file2:
        file2.write(str(p)+"\n")

#time.sleep(2)
io.cleanup()