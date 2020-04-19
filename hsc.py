try:
    import smbus
except:
    print('Try sudo apt-get install python-smbus')

class HHSC(object):

    def __init__(self, bus = 1):

        try:
            self._bus = smbus.SMBus(bus)
        except:
            print("Bus %d is not available.") % bus
            print("Available busses are listed as /dev/i2c*")
            self._bus = None

    def init(self):
        return False

    def read_temperature(self):
        return 0

    def read_pressure(self):
        return 0
