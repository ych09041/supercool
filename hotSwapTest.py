import smbus
import time
bus = smbus.SMBus(1)

slaveAddress = 0x05


class armObj:
    def __init__(self, busLine):
        self.bus = busLine
        self.positions = []
        self.errorState = False
        resetArm()

    def i2cDetect():
        """Pings all potential i2c addresses of linkages


        """
        addressBook = []
        for address in range(3,11):
            if ping(address):
                addressBook.append(address)
        return addressBook
    def ping(address):
        bus.write_quick(address)
        try:
            number = bus.read_byte(address)
            return True
        except ERRORTYPEHERE:
            return False


    def resetArm():
        address = i2cDetect()
        if len(address) > 1:
            print "ERROR, ARM IS HAS AT LEAST ONE MODULE ON IT"
            self.errorState = True
        else




    def hotSwap():
        newAddresses = self.i2cDetect()
        delta = 0
        operation = "Detected no change"
        if self.positions[-1] not in newAddresses:
            delta += 1
            operation = "Detected that top module was removed"
        for address in newAddresses:
            if address not in self.positions:
                delta += 1
                positions.append(address)
                operation = "Detected the addition of 1 module"
        if delta > 1:
    
    



def writeNumber(address,value):
    bus.write_byte(address, value)
    return -1

def readNumber(address):
    number = bus.read_byte(address)
    return number


ping(1)

while True:
    var = raw_input("Enter slave address, number: ")
    varSplit = var.split(",")
    if not var:
        continue
    writeNumber(int(varSplit[0]), int(varSplit[1]))
    print "RPI: Hi Arduino, I sent you ", var
    time.sleep(1)

    number = readNumber(int(varSplit[0]))
    print "Arduino: Hey RPI, I received a digit ", number
