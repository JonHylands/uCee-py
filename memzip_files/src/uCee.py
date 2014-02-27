
LED_RED = 1
LED_GREEN = 2
LED_YELLOW = 3
LED_BLUE = 4

RIGHT_RANGE_PIN = 'C1'
LEFT_RANGE_PIN = 'A1'
FRONT_RANGE_PIN = 'C3'

RIGHT_PROX_PIN = 'C0'
LEFT_PROX_PIN = 'A0'
FRONT_PROX_PIN = 'C2'

LEFT_MOTOR_DIRECTION_PIN = 'B15'
LEFT_MOTOR_PWM_CHANNEL = 2
RIGHT_MOTOR_DIRECTION_PIN = 'A4'
RIGHT_MOTOR_PWM_CHANNEL = 1

LEFT_ENCODER_CHANNEL = 1
RIGHT_ENCODER_CHANNEL = 2

#VOLTAGE_CHECK_PIN = 16
#VOLTAGE_FACTOR = 103.57

FRONT_RANGE_OBSTACLE = 120
SIDE_RANGE_OBSTACLE = 90
RANGE_MAX = 1000
OBSTACLE_TURN_TIME = 50 # extra time to continue turning past not seeing the obstacle anymore

MAX_SPEED = 430
TURN_SPEED = 300

# ENCODER_TO_PID_RATIO is the ratio of encoder ticks to mm per 100 ms
# 2000 encoder increments per revolution = 98.96mm
# ENCODER_TO_PID_RATIO = (98.96 * 10) / 2000 = 0.4948
# so if we count 200 ticks per 1/10 second, we're doing 98.96 mm/s
ENCODER_TO_PID_RATIO = 0.4948


def Delay(msec):
    pyb.delay(msec)

def Log(aString):
    print(aString)

def random(limit):
    return pyb.rand() % limit

#================================================
#
#        Class RangeFinder
#

class RangeFinder:

    def __init__(self, rangePinName, proxDotPinName):
        self.rangePort = pyb.ADC(rangePinName)
        self.proxDotPin = proxDotPinName

    def getDistance(self):
        value = self.rangePort.read()
        voltage = value * 0.000805664 # 0-4095 -> 0-3.3 volts
        exactDistance = 100 * ((1.25 / voltage) - 0.15)
        distance = exactDistance
        if self.proxDotPin == -1:
            if distance > 200:
                return RANGE_MAX
            else:
                return distance
        proxDot = pyb.gpio(self.proxDotPin)
        if (distance > 200) and proxDot:
            return RANGE_MAX
        if not proxDot:
            distance = max(0, 55 - distance)
        return distance

#================================================
#
#        Class Metro
#

class Metro:

    def __init__(self, interval_millis):
        self.interval = interval_millis
        self.previous = pyb.millis()

    def setInterval(self, interval_millis):
        self.interval = interval_millis

    def check(self):
        now = pyb.millis()
        if self.interval == 0:
            self.previous = now
            return True
        if (now - self.previous) >= self.interval:
            self.previous = now
            return True
        return False

    def reset(self):
        self.previous = pyb.millis()

#================================================
#
#        Class HeartbeatLED
#

class HeartbeatLED:

    def __init__(self, ledNumber):
        self.led = pyb.Led(ledNumber)
        self.timer = Metro(0)
        self.set(100, 900)

    def set(self, newOnInterval, newOffInterval):
        self.onInterval = newOnInterval
        self.offInterval = newOffInterval
        self.timer.setInterval(self.offInterval)
        self.ledState = 0
        self.led.off()

    def off(self):
        self.led.off()

    def update(self):
        if self.timer.check():
            if self.ledState:
                self.ledState = 0
                if self.onInterval != self.offInterval:
                    self.timer.setInterval(self.offInterval)
                    self.led.off()
            else:
                self.ledState = 1
                if self.onInterval != self.offInterval:
                    self.timer.setInterval(self.onInterval)
                    self.led.on()

#================================================
#
#        Class PIDController
#

class PIDController:
    def __init__(self, kpValue, kiValue, kdValue):
        self.kp = kpValue
        self.ki = kiValue
        self.kd = kdValue
        self.enabled = False
        self.direction = 1
        self.inputValue = 0
        self.outputValue = 0
        self.setpointValue = 0
        self.sampleTime = 100 #ms
        self.lastTime = pyb.millis() - self.sampleTime
        self.iTerm = 0
        self.lastInput = 0
        self.setOutputLimits(0, 1023)

    def setInput(self, newInputValue):
        self.inputValue = newInputValue

    def setSetpointValue(self, newSetpointValue):
        self.setpointValue = newSetpointValue
        if newSetpointValue >= 0:
            self.setControllerDirection(1)
        else:
            self.setControllerDirection(-1)

    def getOutputValue(self):
        return self.outputValue

    def setControllerDirection(self, newDirection):
        if self.enabled and self.direction != newDirection:
#            print("Current direction: ",self.direction, " kp: ", self.kp, " outMin: ", self.outMin, " outMax: ", self.outMax)
            self.setOutputLimits(-self.outMax, -self.outMin)
#            print("Flipped direction: ", newDirection, " kp: ", self.kp, " outMin: ", self.outMin, " outMax: ", self.outMax)
        self.direction = newDirection

    def setOutputLimits(self, newMin, newMax):
        if newMin >= newMax:
            return
        self.outMin = newMin
        self.outMax = newMax
        if self.enabled:
            if self.outputValue > self.outMax:
                self.outputValue = self.outMax
            elif self.outputValue < self.outMin:
                self.outputValue = self.outMin
            if self.iTerm > self.outMax:
                self.iTerm = self.outMax
            elif self.iTerm < self.outMin:
                self.iTerm = self.outMin

    # you need to call enable() before starting to use this
    def enable(self):
        self.enabled = True

    def compute(self):
        if not self.enabled:
            return false

        now = pyb.millis()
        timeChange = now - self.lastTime
        if timeChange >= self.sampleTime:
            error = self.setpointValue - self.inputValue
            self.iTerm += (self.ki * error)
            if self.iTerm > self.outMax:
                self.iTerm = self.outMax
            elif self.iTerm < self.outMin:
                self.iTerm = self.outMin
            deltaInput = (self.inputValue - self.lastInput)

            # Compute PID Output
            output = self.kp * error + self.iTerm - self.kd * deltaInput
            originalOutput = output
            if output > self.outMax:
                output = self.outMax
            elif output < self.outMin:
                output = self.outMin

            self.outputValue = output
#            print("PID:compute error: ", error, " original: ", originalOutput, " min: ", self.outMin, " max: ", self.outMax)

            #Remember some variables for next time
            self.lastInput = self.inputValue
            self.lastTime = now
            return True
        else:
            return False

#================================================
#
#        Class MotorDriver
#

class MotorDriver:

    def __init__(self, directionPinName, pwmNumber, encoderNumber):
        self.directionPin = directionPinName
        self.pwm = pyb.Pwm(pwmNumber)
        self.directionFactor = 1
        self.directSetSpeed(0)
        self.encoder = pyb.Encoder(encoderNumber)
        self.pid = PIDController(0.25, 0.75, 0.01)
        self.pid.enable()
        self.encoderLastCount = 0
        self.desiredSpeed = 0

    def runReversed(self):
        self.directionFactor = -1

    def setSpeed(self, newDesiredSpeed):
        self.pid.setSetpointValue(newDesiredSpeed)
        self.desiredSpeed = newDesiredSpeed

    def directSetSpeed(self, speed):
        actualSpeed = speed * self.directionFactor
        if actualSpeed < 0:
            pyb.gpio(self.directionPin, 1)
            self.pwm.duty_cycle(-actualSpeed)
        else:
            pyb.gpio(self.directionPin, 0)
            self.pwm.duty_cycle(actualSpeed)

    def getCount(self):
        return self.encoder.count()

    def update(self):
        currentCount = self.encoder.count()
        deltaCount = currentCount - self.encoderLastCount
        pidInput = deltaCount * ENCODER_TO_PID_RATIO
        self.pid.setInput(pidInput)
        if self.pid.compute():
#            print("PID Output: ", self.pid.getOutputValue(), " Input: ", pidInput, " Setpoint: ", self.desiredSpeed)
            self.directSetSpeed(self.pid.getOutputValue())
            self.encoderLastCount = currentCount

#================================================
#
#        Class State
#

class State:

    def __init__(self, stateName, enterFunction, updateFunction, exitFunction):
        self.userEnter = enterFunction
        self.userUpdate = updateFunction
        self.userExit = exitFunction
        self.name = stateName

    def enter(self):
        self.startTimeMillis = pyb.millis()
        if self.userEnter is not None:
            self.userEnter()

    def update(self):
        if self.userUpdate is not None:
            self.userUpdate()

    def exit(self):
        if self.userExit is not None:
            self.userExit()

    def elapsedTimeMillis(self):
        return pyb.millis() - self.startTimeMillis

    def getName(self):
        return self.name

#================================================
#
#        Class FiniteStateMachine
#

class FiniteStateMachine:

    def __init__(self, startState):
        self.currentState = startState
        self.nextState = startState
        self.needToTriggerEnter = True
        self.cycleCount = 0

    def transitionTo(self, newState):
        self.nextState = newState

    def getCycleCount(self):
        return self.cycleCount

    def getCurrentStateMillis(self):
        return self.currentState.elapsedTimeMillis()

    def update(self):
        if self.needToTriggerEnter:
            self.currentState.enter()
            self.needToTriggerEnter = False
        if self.currentState.getName() != self.nextState.getName():
            self.currentState.exit()
            self.currentState = self.nextState
            self.currentState.enter()
            self.cycleCount = 0
        self.cycleCount += 1
        self.currentState.update()

#================================================
#
#        Class MicroCrawler
#

class MicroCrawler:

    def __init__(self):
        self.currentSpeed = 0
        self.frontRangeFinder = RangeFinder(FRONT_RANGE_PIN, -1)
        self.leftRangeFinder = RangeFinder(LEFT_RANGE_PIN, -1)
        self.rightRangeFinder = RangeFinder(RIGHT_RANGE_PIN, -1)
        self.heartbeat = HeartbeatLED(LED_GREEN)
        self.leftMotor = MotorDriver(LEFT_MOTOR_DIRECTION_PIN, LEFT_MOTOR_PWM_CHANNEL, LEFT_ENCODER_CHANNEL)
        self.rightMotor = MotorDriver(RIGHT_MOTOR_DIRECTION_PIN, RIGHT_MOTOR_PWM_CHANNEL, RIGHT_ENCODER_CHANNEL)
        self.rightMotor.runReversed()

        self.movingState = State("moving", self.enterMovingState, self.handleMovingState, None)
        self.obstacleAvoidanceState = State("obstacle", self.enterObstacleAvoidanceState, self.handleObstacleAvoidanceState, None)
        self.shutdownState = State("shutdown", self.enterShutdownState, None, None)
        self.stateMachine = FiniteStateMachine(self.movingState)
        #self.testSpeed = 300
        #self.testTimer = Metro(2000)

        self.timer = Metro(100)

    #def checkVoltage(self):
        #value = 0
        #for index in range(1, 5):
            #value += pyb.analogRead(VOLTAGE_CHECK_PIN)
            #Delay(1)
        #voltage = (value / 5) / VOLTAGE_FACTOR
        #if voltage < 6.1:
            #print("Low voltage warning!")
            #print (voltage)
            #self.stateMachine.transitionTo(self.shutdownState)

    def readSensors(self):
        self.frontRangeDistance = self.frontRangeFinder.getDistance()
        self.leftRangeDistance = self.leftRangeFinder.getDistance()
        self.rightRangeDistance = self.rightRangeFinder.getDistance()
        #self.checkVoltage()

    def setSpeed(self, desiredSpeed, desiredTurnRate):
        if desiredTurnRate == 0:
            self.leftMotor.setSpeed(desiredSpeed)
            self.rightMotor.setSpeed(desiredSpeed)
        elif desiredTurnRate > 0:
            self.leftMotor.setSpeed(desiredSpeed)
            self.rightMotor.setSpeed(desiredSpeed - (desiredSpeed * desiredTurnRate))
        else:
            self.leftMotor.setSpeed(desiredSpeed + (desiredSpeed * desiredTurnRate))
            self.rightMotor.setSpeed(desiredSpeed)

    def enterMovingState(self):
        Log("Entering MovingState")
        self.currentSpeed = MAX_SPEED
        self.currentTurnRate = 0.0

    def handleMovingState(self):
        self.setSpeed(self.currentSpeed, self.currentTurnRate)
        if self.frontRangeDistance < FRONT_RANGE_OBSTACLE:
            self.stateMachine.transitionTo(self.obstacleAvoidanceState)
        elif self.leftRangeDistance < SIDE_RANGE_OBSTACLE:
            self.currentTurnRate = 0.25
        elif self.rightRangeDistance < SIDE_RANGE_OBSTACLE:
            self.currentTurnRate = -0.25
        else:
            self.currentTurnRate = 0.0

    def enterObstacleAvoidanceState(self):
        Log('Entering ObstacleAvoidanceState')
        self.setSpeed(0, 0)
        self.currentSpeed = TURN_SPEED
        if (self.leftRangeDistance < SIDE_RANGE_OBSTACLE) and (self.rightRangeDistance < SIDE_RANGE_OBSTACLE):
            Log('Obstacles on both sides')
            if random(10) > 5:
                self.currentTurnRate = 2
            else:
                self.currentTurnRate = -2
        elif self.leftRangeDistance < SIDE_RANGE_OBSTACLE:
            Log('Obstacle on left side')
            self.currentTurnRate = 2
        elif self.rightRangeDistance < SIDE_RANGE_OBSTACLE:
            Log('Obstacle on right side')
            self.currentTurnRate = -2
        else: #nothing on either side, so pick a side at random
            Log('Only front obstacle')
            if random(10) > 5:
                self.currentTurnRate = 2
            else:
                self.currentTurnRate = -2
        self.exitObstacleStateTime = 0

    def handleObstacleAvoidanceState(self):
        self.setSpeed(self.currentSpeed, self.currentTurnRate)
        if self.frontRangeDistance >= FRONT_RANGE_OBSTACLE and self.exitObstacleStateTime == 0:
            self.exitObstacleStateTime = pyb.millis() + OBSTACLE_TURN_TIME
        if self.frontRangeDistance < FRONT_RANGE_OBSTACLE:
            self.exitObstacleStateTime = 0
        if self.exitObstacleStateTime > 0 and pyb.millis() >= self.exitObstacleStateTime:
            self.stateMachine.transitionTo(self.movingState)

    def enterShutdownState(self):
        self.setSpeed(0, 0)

    def update(self):
        self.heartbeat.update()
        self.leftMotor.update()
        self.rightMotor.update()
        if self.timer.check():
            self.readSensors()
            self.stateMachine.update()

#================================================
#
#        Main Program
#

usbPluggedIn = pyb.gpio('A9') # USB VBUS is connected to A9
#usbPluggedIn = False
if usbPluggedIn:
	Log("")
	Log("uCee Terminal Mode")
	Log("")
	for index in range(1, 5):
		led = pyb.Led(index)
		led.on()
		pyb.delay(250)
		led.off()

else:
	Log("Startup delay")
	heartbeat = HeartbeatLED(LED_BLUE)
	heartbeat.set(100, 200)
	delayTimer = Metro(3000)
	while not delayTimer.check():
		heartbeat.update()

	heartbeat.off()
	heartbeat = None
	delayTimer = None
	pyb.gc()

	Log("uCee.py starting")

	uCee = MicroCrawler()
	pyb.gc()
	pyb.info()

	while True:
		uCee.update()
