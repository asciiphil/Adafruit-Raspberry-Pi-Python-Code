#!/usr/bin/python

import time
import warnings
from Adafruit_I2C import Adafruit_I2C

def _sleepms(ms):
    """A wrapper around time.sleep that tries to be as accurate as possible.
    May still overshoot the requested sleep time, depending on the system's
    clock accuracy."""
    # Experience shows that adding an extra millisecond is sometimes necessary,
    # even with the sleep loop below.
    target = time.time() + (ms + 1) / 1000.0
    while time.time() < target:
        time.sleep(target - time.time())

class SensorSaturatedException(Exception):
    """Used when the sensor is saturated and cannot provide accurate readings."""
    def __init__(self, channel):
        if type(channel) == int:
            self.msg('Channel {0} is saturated.'.format(channel))
        else:
            self.msg('Both channels are saturated.')


class _integrationTime:
    _DATA = {
        #     time   bits     scale       max
        13:  [ 13.7, 0x00,  11.0/322.0,  5047],
        101: [101,   0x01,  81.0/322.0, 37177],
        402: [402,   0x02, 322.0/322.0, 65535],
    }
    def __init__(self, time):
        if not time in _integrationTime._DATA.keys():
            raise IndexError, 'Incorrect integration time: {0}; must be one of {1}'.format(time, sorted(_integrationTime._DATA.keys()))
        self.key = time
        self.time  = _integrationTime._DATA[time][0]
        self.bits  = _integrationTime._DATA[time][1]
        self.scale = _integrationTime._DATA[time][2]
        self.max   = _integrationTime._DATA[time][3]

class TSL2561(object):
    """Main class for interacting with a TSL2561 light sensor.

    The constructor takes several named parameters, all optional:
    * address - The I2C address of the TSL2561 chipset.  The class defines three
      constants for the possible chipset addresses:
      * ADDR_LOW - The address used when the ADDR SEL terminal is connected to
        ground.
      * ADDR_FLOAT - The address used when the ADDR SEL terminal is not
        connected to anything.  This is the default address.
      * ADDR_HIGH - The address used when the ADDR SEL terminal is connected to
        V_DD (the bus voltage).
    * mode - The mode of operation for the lux readings.  The class defines two
      constants for the two possible operating modes:
      * MODE_CONTINUOUS - The sensor is activated when the TSL2561 object is
        initialized.  The `lux` property will return immediately with whatever
        the sensor's current reading is, but that reading is only updated at
        intervals specified by the `integrationTime` parameter.  This mode is ideal
        for relatively continuous readings.  This is the default mode.
      * MODE_SINGLE - The sensor is normally off.  Every time the `lux` property is
        read, the sensor is turned on, the program sleeps for `integrationTime`
        milliseconds, the sensor data is read, the sensor is turned off, and the
        lux value is returned.  This mode is ideal if sensor readings are only
        needed occasionally and you don't want the sensor drawing current all the
        time.
    * gain - The gain applied to sensor readings.  See the `gain` class property
      for more information and acceptable values.  Defaults to 1.
    * integrationTime - The integration time in milliseconds for a sensor
      reading.  See the `integrationTime` class property for more information
      and acceptable values.  Defaults to 402.
    * package - A string indicating which TSL2561 package is in use.  Takes one
      of two values:
      * 'T' - TMB package.  This is the default.
      * 'CS' - Chipscale package.
    * debug - Controls whether additional debugging information is printed as
      the class performs its functions.  Defaults to False.

    To use the class, simply instantiate it and then check the `lux` parameter
    for sensor readings.  The `gain` and `integrationTime` properties may be
    used to alter the sensor's parameters between readings.

    Manual integration intervals are not currently supported.
    Interrupts are not currently supported.
    """

    # Address
    ADDR_LOW   = 0x29
    ADDR_FLOAT = 0x39  # Default address (pin left floating)
    ADDR_HIGH  = 0x49

    # Operating modes
    MODE_CONTINUOUS = 1
    MODE_SINGLE     = 2
    _MODES = set([MODE_CONTINUOUS, MODE_SINGLE])
    
    _REG_MOD = {
        'COMMAND':  0x80,  # Must be 1
        'CLEAR':    0x40,  # Clears any pending interrupt (write 1 to clear)
        'WORD':     0x20,  # 1 = read/write word (rather than byte)
        'BLOCK':    0x10,  # 1 = using block read/write
    }

    _REGISTER = {
        'CONTROL':     0x00,
        'TIMING':      0x01,
        'THRESHHOLDL': 0x02,  # 16-bit
        'THRESHHOLDH': 0x04,  # 16-bit
        'INTERRUPT':   0x06,
        'CRC':         0x08,  # Not for end use.
        'ID':          0x0A,
        'CHAN0':       0x0C,  # 16-bit
        'CHAN1':       0x0E,  # 16-bit
    }

    _CONTROL = {
        'ON':  0x03,
        'OFF': 0x00,
    }

    _GAIN = {
        1:  0x00,
        16: 0x10,
    }

    # For calculating lux.
    # General formula is: B * CH0 - M1 * CH1 * (CH1/CH0)^M2
    # "K" is the ratio of CH1/CH0 and determines which set of coefficients are
    # used.
    # Coefficients are from the "Calculating Lux" section of the datasheet.
    # The TMB and Chipscale packages have different coeffecients.  At
    # instantiation time, the dicts here are replaced by the appropriate
    # component lists.
    _K  = {
        'T':  [0.500,  0.610,  0.800,  1.300],
        'CS': [0.520,  0.650,  0.800,  1.300],
        }
    _B  = {
        'T':  [0.0304, 0.0224, 0.0128, 0.00146, 0.0],
        'CS': [0.0315, 0.0229, 0.0157, 0.00338, 0.0],
        }
    _M1 = { 
        'T':  [0.0620, 0.0310, 0.0153, 0.00112, 0.0], 
        'CS': [0.0593, 0.0291, 0.0180, 0.00260, 0.0],
        }
    _M2 = {
        'T':  [1.4,    0.0,    0.0,    0.0,     0.0],
        'CS': [1.4,    0.0,    0.0,    0.0,     0.0],
        }

    _gain = 1
    _integrationTime = _integrationTime(402)

    def __init__(self, address=ADDR_FLOAT, mode=MODE_CONTINUOUS, gain=_gain, integrationTime=_integrationTime.key, package='T', debug=False):
        if mode not in self._MODES:
            raise ValueError('Incorrect value passed as operating mode.')
        if not address in [TSL2561.ADDR_LOW, TSL2561.ADDR_FLOAT, TSL2561.ADDR_HIGH]:
            addr_msg = '0x{0:x} is not a standard TSL2561 address; continuing anyway'.format(address)
            warnings.warn(addr_msg)
        self.debug = debug
        self._i2c = Adafruit_I2C(address, debug=debug)
        id = self._i2c.readU8(TSL2561._REGISTER['ID'])
        if id < 0 or not id & 0x0A:
            raise EnvironmentError, 'Device at 0x{0:x} does not appear to be a TSL2561.'.format(address)

        self.mode = mode
        if self.mode == TSL2561.MODE_CONTINUOUS:
            self._poweron()

        self.gain = gain
        self.integrationTime = integrationTime
        self._K  = self._K[package]
        self._B  = self._B[package]
        self._M1 = self._M1[package]
        self._M2 = self._M2[package]
        
    @property
    def gain(self):
        """The gain applied to sensor readings.

        Valid values are 1 and 16.  In low-light conditions, a 16x gain will
        provide more accurate results.  In bright-light conditions, 1x gain
        should be used, as 16x might oversaturate the sensor.
        """
        return self._gain

    @gain.setter
    def gain(self, value):
        if not value in TSL2561._GAIN.keys():
            raise IndexError, 'Incorrect gain: {0}; must be one of {1}'.format(value, sorted(TSL2561._GAIN.keys()))
        self._gain = value
        self._setTiming()

    @property
    def integrationTime(self):
        """The amount of time, in milliseconds, to spend collecting data for each reading.

        Valid values are 13, 101, and 402.  Lower values return more quickly,
        but are less accurate, especially in low light conditions.  402ms is the
        standard setting.
        """
        return self._integrationTime.key

    @integrationTime.setter
    def integrationTime(self, value):
        self._integrationTime = _integrationTime(value)
        self._setTiming()

    @property
    def lux(self):
        """The current sensor reading in Lux.

        If the operating mode is MODE_SINGLE, this property will take
        `integrationTime` milliseconds before it returns.  If the operating mode
        is MODE_CONTINUOUS, the property will return immediately, regardless of
        whether the sensor has completed a new cycle in the time since the last
        reading.
        """
        (chan0, chan1) = self._getData()

        chscale = self._integrationTime.scale * self.gain
        schan0 = chan0 / chscale
        schan1 = chan1 / chscale

        if schan0 != 0:
            ratio = schan1 / schan0
        else:
            ratio = 0
        if self.debug:
            print 'ratio: {0}'.format(ratio)
        idx = self._getRatioIdx(ratio)
        if self.debug:
            print 'ratio index: {0}'.format(idx)

        # Calculate
        if self._M2[idx] > 0:
            lux = self._B[idx] * schan0 - self._M1[idx] * schan0 * ratio**self._M2[idx]
        else:
            lux = self._B[idx] * schan0 - self._M1[idx] * schan1

        # Minimum value of zero
        if lux < 0:
            lux = 0
        
        return lux

    def _poweron(self):
        self._i2c.write8(
            TSL2561._REGISTER['CONTROL'] | TSL2561._REG_MOD['COMMAND'],
            TSL2561._CONTROL['ON'])
        if not self._i2c.readU8(TSL2561._REGISTER['CONTROL'] | TSL2561._REG_MOD['COMMAND']) & 0x03 == 0x03:
            raise EnvironmentError, 'TSL2561 did not power on correctly.'.format(address)

    def _poweroff(self):
        self._i2c.write8(
            TSL2561._REGISTER['CONTROL'] | TSL2561._REG_MOD['COMMAND'],
            TSL2561._CONTROL['OFF'])

    def _setTiming(self):
        if self.mode == TSL2561.MODE_SINGLE:
            self._poweron()
        self._i2c.write8(
            TSL2561._REGISTER['TIMING'] | TSL2561._REG_MOD['COMMAND'], 
            TSL2561._GAIN[self.gain] | self._integrationTime.bits)
        if self.mode == TSL2561.MODE_SINGLE:
            self._poweroff()

    def _getData(self):
        if self.mode == TSL2561.MODE_SINGLE:
            self._poweron()
            _sleepms(self._integrationTime.time)
        chan0 = self._i2c.reverseByteOrder(self._i2c.readU16(TSL2561._REGISTER['CHAN0'] | TSL2561._REG_MOD['COMMAND'] | TSL2561._REG_MOD['WORD']))
        chan1 = self._i2c.reverseByteOrder(self._i2c.readU16(TSL2561._REGISTER['CHAN1'] | TSL2561._REG_MOD['COMMAND'] | TSL2561._REG_MOD['WORD']))
        if self.mode == TSL2561.MODE_SINGLE:
            self._poweroff()

        if self.debug:
            print 'chan0: 0x{0:x}, chan1: 0x{1:x}'.format(chan0, chan1)

        if chan0 >= self._integrationTime.max and chan1 >= self._integrationTime.max:
            raise SensorSaturatedException('both')
        elif chan0 >= self._integrationTime.max:
            raise SensorSaturatedException(0)
        elif chan1 >= self._integrationTime.max:
            raise SensorSaturatedException(1)

        return (chan0, chan1)

    def _getRatioIdx(self, ratio):
        for i in xrange(len(self._K)):
            if ratio < self._K[i]:
                return i
        return len(self._K)
                
