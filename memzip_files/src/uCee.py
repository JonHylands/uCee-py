
print("uCee.py")

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

RANGE_THRESHOLD = 500

FORWARD_SPEED = 1023
TURN_AWAY_MODIFIER = 200


def Delay(msec):
	pyb.delay(msec)

class RangeFinder:
	def __init__(self, pinNumber):
		self.pin = pinNumber
	def getRange():
		return pyb.analogRead(self.pin)

class Led:
	def __init__(self, pinNumber):
		self.pin = pinNumber
	def turnOn(self):
		pyb.gpio(self.pin, 1)
	def turnOff(self):
		pyb.gpio(self.pin, 0)


def setSpeed(leftSpeed, rightSpeed):
	if leftSpeed < 0:
		pyb.gpio(LEFT_MOTOR_DIRECTION_PIN, 0)
		pyb.analogWrite(LEFT_MOTOR_PWM_PIN, -leftSpeed)
	else:
		pyb.gpio(LEFT_MOTOR_DIRECTION_PIN, 1)
		pyb.analogWrite(LEFT_MOTOR_PWM_PIN, 1023 - leftSpeed)
	if rightSpeed < 0:
		pyb.gpio(RIGHT_MOTOR_DIRECTION_PIN, 0)
		pyb.analogWrite(RIGHT_MOTOR_PWM_PIN, -rightSpeed)
	else:
		pyb.gpio(RIGHT_MOTOR_DIRECTION_PIN, 1)
		pyb.analogWrite(RIGHT_MOTOR_PWM_PIN, 1023 - rightSpeed)

def MotorsFwd():
	setSpeed(1023, 1023)

def MotorsBwd():
	setSpeed(-600, -600)

def MotorsL():
	setSpeed(-800, 800)

def MotorsR():
	setSpeed(800, -800)

def MotorsStop():
	setSpeed(0, 0)

def InitMotors():
	pyb.gpio(LEFT_MOTOR_ENABLE_PIN, 0)
	pyb.gpio(RIGHT_MOTOR_ENABLE_PIN, 0)
	pyb.analogWriteResolution(10)
	pyb.analogWriteFrequency(LEFT_MOTOR_PWM_PIN, 10000)
	pyb.analogWriteFrequency(RIGHT_MOTOR_PWM_PIN, 10000)
	MotorsStop();
	pyb.gpio(LEFT_MOTOR_ENABLE_PIN, 1)
	pyb.gpio(RIGHT_MOTOR_ENABLE_PIN, 1)

def CheckVoltage():
	value = 0
	for index in range(1, 5):
		value += pyb.analogRead(VOLTAGE_CHECK_PIN)
	voltage = (value / 5) / VOLTAGE_FACTOR
	if voltage < 6.1:
		MotorsStop()
		pyb.gpio(LED_PIN, 1)
		Delay(10000)
		print("Low voltage warning!")
		print (voltage)
		while True:
			pyb.gpio(LED_PIN, 1)
			Delay(100)
			pyb.gpio(LED_PIN, 0)
			Delay(100)

def Roaming():
	currentLeftSpeed = FORWARD_SPEED
	currentRightSpeed = FORWARD_SPEED
	while True:
		setSpeed(currentLeftSpeed, currentRightSpeed)
		frontRange = frontRangeFinder.getRange()
		if frontRange > RANGE_THRESHOLD:
			if leftRangeFinder.getRange() > RANGE_THRESHOLD:
				MotorsR()
			elif rightRangeFinder.getRange() > RANGE_THRESHOLD:
				MotorsL()
			else:
				if pyb.random(10) > 5:
					MotorsL()
				else:
					MotorsR()
			while frontRange > RANGE_THRESHOLD:
				Delay(100)
				frontRange =  frontRangeFinder.getRange()
			Delay(250) # give it a little more time to turn
		if leftRangeFinder.getRange() > RANGE_THRESHOLD:
			currentRightSpeed = FORWARD_SPEED - TURN_AWAY_MODIFIER
		elif rightRangeFinder.getRange() > RANGE_THRESHOLD:
			currentLeftSpeed = FORWARD_SPEED - TURN_AWAY_MODIFIER
		else:
			currentLeftSpeed = FORWARD_SPEED
			currentRightSpeed = FORWARD_SPEED
		Delay(10)

InitMotors()
#CheckVoltage()
#Roaming()

frontRangeFinder = RangeFinder(FRONT_RANGE_PIN)
leftRangeFinder = RangeFinder(LEFT_RANGE_PIN)
rightRangeFinder = RangeFinder(RIGHT_RANGE_PIN)
led = Led(LED_PIN)

led.turnOn()
Delay(5000)
led.turnOff()

