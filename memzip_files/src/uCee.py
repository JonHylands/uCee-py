
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

RANGE_THRESHOLD = 60
RANGE_MAX = 1000

FORWARD_SPEED = 1023
TURN_AWAY_MODIFIER = 200


def Delay(msec):
	pyb.delay(msec)

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
			distance = 55 - distance
			if distance < 0:
				distance = 0
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
#		Class MicroCrawler
#

class MicroCrawler:

	def __init__(self):
		self.currentLeftSpeed = FORWARD_SPEED
		self.currentRightSpeed = FORWARD_SPEED
		self.frontRangeFinder = RangeFinder(FRONT_RANGE_PIN, -1)
		self.leftRangeFinder = RangeFinder(LEFT_RANGE_PIN, -1)
		self.rightRangeFinder = RangeFinder(RIGHT_RANGE_PIN, -1)
		self.heartbeat = HeartbeatLED(LED_PIN)
		self.leftMotor = MotorDriver(LEFT_MOTOR_ENABLE_PIN, LEFT_MOTOR_DIRECTION_PIN, LEFT_MOTOR_PWM_PIN, LEFT_MOTOR_CURRENT_PIN)
		self.rightMotor = MotorDriver(RIGHT_MOTOR_ENABLE_PIN, RIGHT_MOTOR_DIRECTION_PIN, RIGHT_MOTOR_PWM_PIN, RIGHT_MOTOR_CURRENT_PIN)

	def roam(self):
		self.heartbeat.update()
		self.leftMotor.setSpeed(self.currentLeftSpeed)
		self.rightMotor.setSpeed(self.currentRightSpeed)
		frontRange = self.frontRangeFinder.getDistance()
		if frontRange < RANGE_THRESHOLD:
			print("Obstacle in front")
			if self.leftRangeFinder.getDistance() < RANGE_THRESHOLD:
				self.rightMotor.setSpeed(-self.currentRightSpeed)
			elif self.rightRangeFinder.getDistance() < RANGE_THRESHOLD:
				self.leftMotor.setSpeed(-self.currentLeftSpeed)
			else:
				if pyb.random(10) > 5:
					self.leftMotor.setSpeed(-self.currentLeftSpeed)
				else:
					self.rightMotor.setSpeed(-self.currentRightSpeed)
			while frontRange < RANGE_THRESHOLD:
				Delay(100)
				frontRange =  self.frontRangeFinder.getDistance()
			Delay(250) # give it a little more time to turn
			print("Finished Obstacle")
		if self.leftRangeFinder.getDistance() < RANGE_THRESHOLD:
			self.currentRightSpeed = FORWARD_SPEED - TURN_AWAY_MODIFIER
		elif self.rightRangeFinder.getDistance() < RANGE_THRESHOLD:
			self.currentLeftSpeed = FORWARD_SPEED - TURN_AWAY_MODIFIER
		else:
			self.currentLeftSpeed = FORWARD_SPEED
			self.currentRightSpeed = FORWARD_SPEED

	def checkVoltage(self):
		value = 0
		for index in range(1, 5):
			value += pyb.analogRead(VOLTAGE_CHECK_PIN)
		voltage = (value / 5) / VOLTAGE_FACTOR
		if voltage < 6.1:
			print("Low voltage warning!")
			print (voltage)
			while True:
				pyb.gpio(LED_PIN, 1)
				Delay(100)
				pyb.gpio(LED_PIN, 0)
				Delay(100)

#================================================
#
#		Main Program
#

uCee = MicroCrawler()

Delay(5000)
print("uCee.py")
pyb.gc()
pyb.info()
#uCee.checkVoltage()
while True:
	uCee.roam()
	Delay(10)
