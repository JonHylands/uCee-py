print("uCee.py")

LED_PIN = 13;

RIGHT_RANGE_PIN = 23;
LEFT_RANGE_PIN = 22;
FRONT_RANGE_PIN = 21;

RIGHT_PROX_PIN = 20;
LEFT_PROX_PIN = 6;
FRONT_PROX_PIN = 17;

RIGHT_ENCODER_A_PIN = 5;
RIGHT_ENCODER_B_PIN = 4;
LEFT_ENCODER_A_PIN = 3;
LEFT_ENCODER_B_PIN = 2;

LEFT_MOTOR_ENABLE_PIN = 12;
LEFT_MOTOR_DIRECTION_PIN = 11;
LEFT_MOTOR_PWM_PIN = 10;
RIGHT_MOTOR_PWM_PIN = 9;
RIGHT_MOTOR_DIRECTION_PIN = 8;
RIGHT_MOTOR_ENABLE_PIN = 7;

RIGHT_MOTOR_CURRENT_PIN = 14;
LEFT_MOTOR_CURRENT_PIN = 15;

VOLTAGE_CHECK_PIN = 16;

RANGE_THRESHOLD = 500


def Delay(msec):
    pyb.delay(msec)

def ReadFrontRange():
	return pyb.analogRead(FRONT_RANGE_PIN)

def ReadLeftRange():
	return pyb.analogRead(LEFT_RANGE_PIN)

def ReadRightRange():
	return pyb.analogRead(RIGHT_RANGE_PIN)

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
	setSpeed(750, 750)

def MotorsBwd():
	setSpeed(-600, -600)

def MotorsL():
	setSpeed(-500, 500)

def MotorsR():
	setSpeed(500, -500)

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

def Roaming():
	while True:
		MotorsFwd()
		frontRange = ReadFrontRange()
		if frontRange > RANGE_THRESHOLD:
			if ReadLeftRange() > RANGE_THRESHOLD:
				MotorsR()
			elif ReadRightRange() > RANGE_THRESHOLD:
				MotorsL()
			else:
				if pyb.random(10) > 5:
					MotorsL()
				else:
					MotorsR()
			while frontRange > RANGE_THRESHOLD:
				Delay(100)
				frontRange = ReadFrontRange()
			Delay(250) # give it a little more time to turn
			MotorsFwd()
		Delay(10)

InitMotors()
Roaming()
