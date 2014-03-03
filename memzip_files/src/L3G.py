
import pyb

L3G_ADDRESS = 0b1101011 # on the Pololu MiniIMU-9 v2, SA0 is pulled high

# register addresses

L3G_CTRL_REG1 = 0x20
L3G_OUT_X_L = 0x28

class L3G:
    def __init__(self):
        self.i2c = pyb.I2C(1, L3G_ADDRESS)
        self.g = [0, 0, 0]

    # enableDefault - Turns on the L3G's gyro and places it in normal mode.
    def enableDefault(self):
        self.writeRegister(L3G_CTRL_REG1, 0x0F); # Normal power mode, all axes enabled

    # writeReg - Writes a gyro register
    def writeRegister(self, register, value):
        self.i2c.start()
        self.i2c.write(register)
        self.i2c.write(value)
        self.i2c.stop()

    def readRegister(self, register):
        self.i2c.start()
        self.i2c.write(register)
        value = self.i2c.readAndStop()
        return value

    # This implements a simple 16-bit two's complement
    def twosComplement(self, value):
        if value & 0x8000:
            result = value - 0x10000
        else:
            result = value
        return result

    def read(self):
        self.i2c.start()
        self.i2c.write(L3G_OUT_X_L | (1 << 7)) # turning on the high bit enables auto-increment for reading
        xlg = self.i2c.read()
        xhg = self.i2c.read()
        ylg = self.i2c.read()
        yhg = self.i2c.read()
        zlg = self.i2c.read()
        zhg = self.i2c.readAndStop()
        self.g[0] = self.twosComplement((xhg << 8) | xlg)
        self.g[1] = self.twosComplement((yhg << 8) | ylg)
        self.g[2] = self.twosComplement((zhg << 8) | zlg)

