
import pyb

LSM303_ACCEL_ADDRESS = 0b0011001
LSM303_MAG_ADDRESS = 0b0011110

CTRL_REG1_A = 0x20
CTRL_REG4_A = 0x23
OUT_X_L_A = 0x28

CRA_REG_M = 0x00
CRB_REG_M = 0x01
MR_REG_M = 0x02
DLHC_OUT_X_H_M = 0x03

class LSM303:
    def __init__(self):
        self.accI2C = pyb.I2C(1, LSM303_ACCEL_ADDRESS)
        self.magI2C = pyb.I2C(1, LSM303_MAG_ADDRESS)
        self.a = [0, 0, 0]
        self.m = [0, 0, 0]
        self.m_min = [0, 0, 0]
        self.m_max = [0, 0, 0]

    # writeAccRegister - Writes an accelerometer register
    def writeAccRegister(self, register, value):
        self.accI2C.start()
        self.accI2C.write(register)
        self.accI2C.write(value)
        self.accI2C.stop()

    # writeMagRegister - Writes a magnetometer register
    def writeMagRegister(self, register, value):
        self.magI2C.start()
        self.magI2C.write(register)
        self.magI2C.write(value)
        self.magI2C.stop()

    def enableDefault(self):
        # Accelerometer
        # 0x08 = 0b00001000
        # FS = 00 (+/- 2 g full scale); HR = 1 (high resolution enable)
        self.writeAccRegister(CTRL_REG4_A, 0x08)
        # 0x47 = 0b01000111
        # ODR = 0100 (50 Hz ODR); LPen = 0 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
        self.writeAccRegister(CTRL_REG1_A, 0x47)

        # Magnetometer
        # 0x0C = 0b00001100
        # DO = 011 (7.5 Hz ODR)
        self.writeMagRegister(CRA_REG_M, 0x0C)
        # 0x20 = 0b00100000
        # GN = 001 (+/- 1.3 gauss full scale)
        self.writeMagRegister(CRB_REG_M, 0x20)
        # 0x00 = 0b00000000
        # MD = 00 (continuous-conversion mode)
        self.writeMagRegister(MR_REG_M, 0x00)

    # This implements a simple 16-bit two's complement
    def twosComplement(self, value):
        if value & 0x8000:
            result = value - 0x10000
        else:
            result = value
        return result

    def readAcc(self):
        self.accI2C.start()
        self.accI2C.write(OUT_X_L_A | (1 << 7))
        xla = self.accI2C.read()
        xha = self.accI2C.read()
        yla = self.accI2C.read()
        yha = self.accI2C.read()
        zla = self.accI2C.read()
        zha = self.accI2C.readAndStop()
        self.a[0] = self.twosComplement((xha << 8) | xla)
        self.a[1] = self.twosComplement((yha << 8) | yla)
        self.a[2] = self.twosComplement((zha << 8) | zla)

    def readMag(self):
        self.magI2C.start()
        self.magI2C.write(DLHC_OUT_X_H_M | (1 << 7))
        xhm = self.magI2C.read() # note that the LSM303DLHC reads X, Z, Y, in big-endian order
        xlm = self.magI2C.read()
        zhm = self.magI2C.read()
        zlm = self.magI2C.read()
        yhm = self.magI2C.read()
        ylm = self.magI2C.readAndStop()
        self.m[0] = self.twosComplement((xhm << 8) | xlm)
        self.m[1] = self.twosComplement((yhm << 8) | ylm)
        self.m[2] = self.twosComplement((zhm << 8) | zlm)

    def read(self):
        self.readAcc()
        self.readMag()

