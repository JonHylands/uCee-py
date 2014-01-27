
LED_PIN = 13

RIGHT_RANGE_PIN = 23
LEFT_RANGE_PIN = 22
FRONT_RANGE_PIN = 21

RIGHT_PROX_PIN = 20
LEFT_PROX_PIN = 6
FRONT_PROX_PIN = 17

RIGHT_ENCODER_A_PIN = 5
RIGHT_ENCODER_B_PIN = 4
LEFT_ENCODER_A_PIN = 3
LEFT_ENCODER_B_PIN = 2

LEFT_MOTOR_ENABLE_PIN = 12
LEFT_MOTOR_DIRECTION_PIN = 11
LEFT_MOTOR_PWM_PIN = 10
RIGHT_MOTOR_PWM_PIN = 9
RIGHT_MOTOR_DIRECTION_PIN = 8
RIGHT_MOTOR_ENABLE_PIN = 7

RIGHT_MOTOR_CURRENT_PIN = 14
LEFT_MOTOR_CURRENT_PIN = 15

VOLTAGE_CHECK_PIN = 16
VOLTAGE_FACTOR = 103.57

FRONT_RANGE_OBSTACLE = 120
SIDE_RANGE_OBSTACLE = 90
RANGE_MAX = 1000
OBSTACLE_TURN_TIME = 50 # extra time to continue turning past not seeing the obstacle anymore

START_SPEED = 500
MAX_SPEED = 1023
TURN_SPEED = 800
SPEED_INCREMENT = 25


def Delay(msec):
	pyb.delay(msec)

def Log(aString):
	print(aString)

#================================================
#
#		Class RangeFinder
#

class RangeFinder:

	def __init__(self, rangePinNumber, proxDotPinNumber):
		self.rangePin = rangePinNumber
		self.proxDotPin = proxDotPinNumber

	def getDistance(self):
		value = pyb.analogRead(self.rangePin)
		voltage = value * 0.0032258 # 0-1023 -> 0-3.3 volts
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
#		Class Metro
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
#		Class HeartbeatLED
#

class HeartbeatLED:

	def __init__(self, ledPinNumber):
		self.ledPin = ledPinNumber
		self.timer = Metro(0)
		self.set(100, 900)

	def set(self, newOnInterval, newOffInterval):
		self.onInterval = newOnInterval
		self.offInterval = newOffInterval
		self.timer.setInterval(self.offInterval)
		self.ledState = 0
		pyb.gpio(self.ledPin, self.ledState)

	def update(self):
		if self.timer.check():
			if self.ledState:
				self.ledState = 0
				if self.onInterval != self.offInterval:
					self.timer.setInterval(self.offInterval)
			else:
				self.ledState = 1
				if self.onInterval != self.offInterval:
					self.timer.setInterval(self.onInterval)
			pyb.gpio(self.ledPin, self.ledState)

#================================================
#
#		Class MotorDriver
#

class MotorDriver:

	def __init__(self, enablePinNumber, directionPinNumber, pwmPinNumber, currentPinNumber):
		self.enablePin = enablePinNumber
		self.directionPin = directionPinNumber
		self.pwmPin = pwmPinNumber
		self.currentPin = currentPinNumber
		self.directionFactor = 1
		pyb.analogWriteResolution(10)
		pyb.analogWriteFrequency(self.pwmPin, 10000)
		pyb.gpio(self.enablePin, 0)
		self.setSpeed(0)
		pyb.gpio(self.enablePin, 1)

	def runReversed(self):
		self.directionFactor = -1

	def setSpeed(self, speed):
		actualSpeed = speed * self.directionFactor
		if actualSpeed < 0:
			pyb.gpio(self.directionPin, 0)
			pyb.analogWrite(self.pwmPin, -actualSpeed)
		else:
			pyb.gpio(self.directionPin, 1)
			pyb.analogWrite(self.pwmPin, 1023 - actualSpeed)

	def getCurrent(self):
		return pyb.analogRead(self.currentPin)

#================================================
#
#		Class State
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
#		Class FiniteStateMachine
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
#		Class MicroCrawler
#

class MicroCrawler:

	def __init__(self):
		self.currentSpeed = 0
		self.frontRangeFinder = RangeFinder(FRONT_RANGE_PIN, -1)
		self.leftRangeFinder = RangeFinder(LEFT_RANGE_PIN, -1)
		self.rightRangeFinder = RangeFinder(RIGHT_RANGE_PIN, -1)
		self.heartbeat = HeartbeatLED(LED_PIN)
		self.leftMotor = MotorDriver(LEFT_MOTOR_ENABLE_PIN, LEFT_MOTOR_DIRECTION_PIN, LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_CURRENT_PIN)
		self.rightMotor = MotorDriver(RIGHT_MOTOR_ENABLE_PIN, RIGHT_MOTOR_DIRECTION_PIN, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_CURRENT_PIN)

		self.movingState = State("moving", self.enterMovingState, self.handleMovingState, None)
		self.obstacleAvoidanceState = State("obstacle", self.enterObstacleAvoidanceState, self.handleObstacleAvoidanceState, None)
		self.shutdownState = State("shutdown", self.enterShutdownState, None, None)
		self.stateMachine = FiniteStateMachine(self.movingState)

		self.timer = Metro(100)

	def checkVoltage(self):
		value = 0
		for index in range(1, 5):
			value += pyb.analogRead(VOLTAGE_CHECK_PIN)
			Delay(1)
		voltage = (value / 5) / VOLTAGE_FACTOR
		if voltage < 6.1:
			print("Low voltage warning!")
			print (voltage)
			self.stateMachine.transitionTo(self.shutdownState)

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
		self.currentSpeed = START_SPEED
		self.currentTurnRate = 0.0

	def handleMovingState(self):
		if self.currentSpeed < MAX_SPEED:
			self.currentSpeed = max(MAX_SPEED, self.currentSpeed + SPEED_INCREMENT)
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
			if pyb.random(10) > 5:
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
			if pyb.random(10) > 5:
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
		if self.timer.check():
			self.readSensors()
			self.stateMachine.update()

#================================================
#
#		Main Program
#

# delay long enough to open a serial terminal

Log("Startup delay")
heartbeat = HeartbeatLED(LED_PIN)
heartbeat.set(100, 100)
delayTimer = Metro(3000)
while not delayTimer.check():
	heartbeat.update()

heartbeat = None
delayTimer = None
pyb.gc()

pyb.gpio(LED_PIN, 0)
Log("uCee.py starting")

uCee = MicroCrawler()
pyb.gc()
pyb.info()

while True:
	uCee.update()
