#!/usr/bin/python

import time
from Adafruit_I2C import Adafruit_I2C

class _MPL115A2_Register:

    # Each register is two bytes wide, with the values left-aligned in the
    # register.
    # TOTAL_BITS gives the total number of data bits (and is used to calculate
    # how much right shifting is needed to eliminate the padding).
    # SIGN_BITS is the number of bits reserved for sign information and 
    # determines whether the value is read as a signed or unsigned number.
    # FRACTIONAL_BITS gives the number of bits that should be considered to
    # follow the decimal point.
    # DECIMAL_ZERO_PADDING gives the number of additional zero bits that should
    # be considered to exist between the decimal point and the first data bit.

    def __init__(self, address, total_bits, sign_bits, fractional_bits, decimal_zero_padding):
        self.address = address
        self.total_bits = total_bits
        self.sign_bits = sign_bits
        self.fractional_bits = fractional_bits
        self.decimal_zero_padding = decimal_zero_padding

    def read(self, i2c):
        # Get the data from the register.
        raw_value = i2c.readU16(self.address)

        # Negative values are stored in two's-complement
        if self.sign_bits == 1 and raw_value & 0x8000:
            raw_value = raw_value ^ 0xFFFF * -1

        # Apply the bit parameters.
        return (raw_value >> (16 - self.total_bits)) / float(2**(self.fractional_bits+self.decimal_zero_padding))


class MPL115A2:

    # From the section 3.1 table:
    #                                 address               bits
    #                                          total  sign  frac  dec.zero.pad 
    __REGISTER_PADC = _MPL115A2_Register(0x00,    10,    0,    0,            0)
    __REGISTER_TADC = _MPL115A2_Register(0x02,    10,    0,    0,            0)
    __REGISTER_A0   = _MPL115A2_Register(0x04,    16,    1,    3,            0)
    __REGISTER_B1   = _MPL115A2_Register(0x06,    16,    1,   13,            0)
    __REGISTER_B2   = _MPL115A2_Register(0x08,    16,    1,   14,            0)
    __REGISTER_C12  = _MPL115A2_Register(0x0A,    14,    1,   13,            9)


    def __init__(self, address=0x60, debug=False):
        self.i2c = Adafruit_I2C(address, debug=debug)

        # Read coefficients
        self.a0  = self.__REGISTER_A0.read(self.i2c)
        self.b1  = self.__REGISTER_B1.read(self.i2c)
        self.b2  = self.__REGISTER_B2.read(self.i2c)
        self.c12 = self.__REGISTER_C12.read(self.i2c)


    def getPT(self):
        """Returns a tuple of (pressure, temperature) as measured by the sensor."""

        # Instruct the sensor to begin data conversion.
        self.i2c.write8(0x12, 0x00)

        # Wait until conversion has finished.  The datasheet says "3ms".  We'll
        # wait 5ms just to be sure.  Please note the documentation about
        # time.sleep(), though:
        # http://docs.python.org/3.0/library/time.html#time.sleep
        time.sleep(0.005)

        # Read the raw values.
        padc = self.__REGISTER_PADC.read(self.i2c)
        tadc = self.__REGISTER_TADC.read(self.i2c)

        # Calculate compensated pressure value, section 3.2 of the datasheet.
        pcomp = self.a0 + (self.b1 + self.c12 * tadc) * padc + self.b2 * tadc

        # Calculate final values.  The formula for pressure is from section 3.2
        # of the datasheet.  The formula for temperature is basically magic:
        # http://www.adafruit.com/forums/viewtopic.php?f=19&t=41347
        pressure = pcomp * ((115.0 - 50.0) / 1023.0) + 50.0
        temperature = (tadc - 498.0) / -5.35 + 25.0

        return (pressure, temperature)

    
    def getPressure(self):
        return self.getPT()[0]


    def getTemperature(self):
        return self.getPT()[1]
