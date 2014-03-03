
from L3G import *
from LSM303 import *

print("Starting")
gyro = L3G()
compass = LSM303()
print("About to enableDefault")
gyro.enableDefault()
compass.enableDefault()
print("About to read")
gyro.read()
compass.read()
print("Gyro value: ", gyro.g)
print("Accel value: ", compass.a)
print("Mag value: ", compass.m)
