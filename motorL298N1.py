import time
import RPi.GPIO as io

class MotorL298N:

    def __init__(self, pinIn, pinEncoder):

        io.setmode(io.BOARD)
        io.setup(pinIn, io.OUT)
        io.setup(pinEncoder, io.IN, pull_up_down=io.PUD_UP)
        io.add_event_detect(pinEncoder[0], io.RISING, callback=self.__encoderACallback)
        # io.add_event_detect(pinEncoder[1], io.RISING, callback=self.__encoderACallback)
        self.pwm1 = io.PWM(pinIn[0], 50)
        self.pwm2 = io.PWM(pinIn[1], 50)
        self.channelA = pinEncoder[0]
        self.channelB = pinEncoder[1]
        self.currentDirection = 0
        self.directionSet = 0
        self.currentPosition = 0

        self.pwm1.start(0)
        self.pwm2.start(0)

    def __encoderACallback(self, channelA):
        self.currentDirection = io.input(self.channelB) * 2 -1
        self.currentPosition += self.currentDirection

    # def __encoderBCallback(self, channelB):
    #     self.currentPosition += self.currentDirection
    
    def setDirection(self, dirc):
        self.directionSet = dirc

    def updatePWM(self, pwmValue):
        if self.directionSet == 1:
            self.pwm1.ChangeDutyCycle(pwmValue)
            self.pwm2.ChangeDutyCycle(0)
        elif self.directionSet == -1:
            self.pwm1.ChangeDutyCycle(0)
            self.pwm2.ChangeDutyCycle(pwmValue)
        else:
            self.pwm1.ChangeDutyCycle(0)
            self.pwm2.ChangeDutyCycle(0)

    def getPosition(self):
        return self.currentPosition
