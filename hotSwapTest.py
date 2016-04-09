import smbus
import time
bus = smbus.SMBus(1)

##slaveAddress = 0x05

class ArmObj:
    def __init__(self, busLine):
        self.bus = busLine
        self.positions = []
        self.errorState = False
        posDetect()

    def posDetect():
        """Pings all potential i2c addresses of linkages,
        sets self.position to the current order and addresses of the linkages
        """

        addressBook = []
        for address in range(3,11):
            onTime = ping(address)
            if onTime> -1:
                addressBook.append((address,onTime))
        addressBook.sort(key=lambda tup: tup[1])
        self.positions = []
        print "Detected ",len(addressBook)," number of linkages. In order:"
        for i in range(0,len(addressBook)):
            print addressBook[i][0]
            self.positions.append(addressBook[i][0])
        self.errorState = False
        return
        
    def ping(address):
        """Pings the given i2c address. Returns the millis() of the Arduino if it is there,
        returns -1 otherwise"""
                                   
        bus.write_byte(address,-1)
        try:
            number = bus.read_byte(address)
            return number
        except ERRORTYPEHERE:
            return -1

    
##    def resetArm():
##        address = i2cDetect()
##        if len(address) > 1:
##            print "ERROR, ARM IS HAS AT LEAST ONE MODULE ON IT"
##            self.errorState = True
##        else




##    def hotSwap():
##        newAddresses = self.i2cDetect()
##        delta = 0
##        operation = "Detected no change"
##        if self.positions[-1] not in newAddresses:
##            delta += 1
##            operation = "Detected that top module was removed"
##        for address in newAddresses:
##            if address not in self.positions:
##                delta += 1
##                positions.append(address)
##                operation = "Detected the addition of 1 module"
##        if delta > 1:
    
        

    def writeOneLink(address,value):
        if value > self.linkMax:
            print "Trying to move too far!"
            self.bus.write_byte(address, self.linkMax)
        if value < self.linkMin:
            print "Trying to move too far!"
            self.bus.write_byte(address, self.linkMin)
        else:
            self.bus.write_byte(address, value)
        return -1

    def readOneLink(address):
        number = self.bus.read_byte(address)
        return number

    def writeArm(lis):
        if len(lis) > len(self.positions):
            print "WARNING: COMMAND IS FOR A LONGER ARM CONFIGURATION"
        if len(lis) < len(self.positions):
            print "WARNING: COMMAND IS FOR A SHORTER ARM CONFIGURATION"
        else:
            for i in range(0,len(lis)):
                self.writeOneLink(self.positions[i], lis[i])
        return


arm = ArmObj(bus)

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
