import time
import RPi.GPIO as io

class MotorL298N:

    def __init__(self, pinIn, pinEncoder, encoderNum=0):

        io.setmode(io.BOARD)
        io.setup(pinIn, io.OUT)
        io.setup(pinEncoder, io.IN, pull_up_down=io.PUD_UP)
        io.add_event_detect(pinEncoder, io.RISING, callback=self.__encoderCallback)
        self.pwm1 = io.PWM(pinIn[0], 50)
        self.pwm2 = io.PWM(pinIn[1], 50)
        self.currentDirection = 0
        self.encoderNum = encoderNum
        self.addValue = 0

        self.pwm1.start(0)
        self.pwm2.start(0)

    def __encoderCallback(self, channel):
        self.encoderNum += self.currentDirection

    def setDirection(self, dirc):
        self.currentDirection = dirc
        if dirc != 0:
            self.addValue = dirc

    def updatePWM(self, pwmValue):
        if self.currentDirection == 1:
            self.pwm1.ChangeDutyCycle(pwmValue)
            self.pwm2.ChangeDutyCycle(0)
        elif self.currentDirection == -1:
            self.pwm1.ChangeDutyCycle(0)
            self.pwm2.ChangeDutyCycle(pwmValue)
        else:
            self.pwm1.ChangeDutyCycle(0)
            self.pwm2.ChangeDutyCycle(0)

    def getPosition(self):
        return self.encoderNum
