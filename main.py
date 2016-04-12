import smbus
import time
bus = smbus.SMBus(1)

##slaveAddress = 0x05

class ArmObj:
    """Object for representing 1 mechanical arm. Using OOP just for data management.

    """
    def __init__(self, busLine, i2cMinRange=3, i2cMaxRange=10):
        """Constructor for ArmObj. Needs a busline that it operates on and the interval of i2cAddresses that its linkages
        can live on, inclusive. Constructor initializes the object variables:

        positions = list of i2c addresses in the order they are connected (first element is the lowest linkage, last element is the top linkage)
        errorState = True if something has gone wrong- need to run posDetect() or something else
        linkMin = minimum value that the encoder can sweep to
        linkMax = maximum value that the encoder can sweep to
        posDetect = fills the positions list.

        """
        
        
        self.bus = busLine
        self.positions = []
        self.errorState = False
        self.linkMin = 0
        self.linkMax = 500
        self.i2cMin = i2cMinRange
        self.i2cMax = i2cMaxRange
        self.posDetect()

    def posDetect(self):
        """Pings all potential i2c addresses of linkages, and stores the addresses of those that respond back.
        Orders the addresses in order of the Arduino's on-time and sets self.position addresses of the linkages in decreasing on-times.
        """

        addressBook = []
        for address in range(self.i2cMin,self.i2cMax+1):
            onTime = self.ping(address)
            if onTime> -1:
                addressBook.append((address,onTime))
        addressBook.sort(key=lambda tup: tup[1],reverse=True)
        self.positions = []
        print "Detected ",len(addressBook)," number of linkages. In order:"
        for i in range(0,len(addressBook)):
            print addressBook[i][0]
            self.positions.append(addressBook[i][0])
        self.errorState = False
        return
        
    def ping(self,address):
        """Pings the given i2c address. Returns the millis() of the Arduino if it is there,
        returns -1 otherwise"""
                                   
        
        try:
            bus.write_byte(address,-1)
            number = bus.read_byte(address)
            return number
        except IOError:
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
    
        

    def writeOneLink(self,address,value):
        """Writes 'value' to the device at 'address' over i2c

        """
        if value > self.linkMax:
            print "Trying to move too far!"
            self.bus.write_byte(address, self.linkMax)
        if value < self.linkMin:
            print "Trying to move too far!"
            self.bus.write_byte(address, self.linkMin)
        else:
            self.bus.write_byte(address, value)
        return -1

    def readOneLink(self,address):
        number = self.bus.read_byte(address)
        return number

    def writeArm(self,lis):
        """Takes a list of integers, lis.
        Writes each element of lis to the corresponding linkage by position ordering.
        Failsafe checks to see if lis is the same length as the number of linkages on the arm. If there is a mismatch, nothing is done.

        """
        
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
    if len(varSplit) != 2:
        if varSplit[0] == "p":
            arm.posDetect()
        if varSplit[0] == "c":
            arm.writeArm(int(varSplit[1:len(varSplit)]))
    else:
        arm.writeOneLink(int(varSplit[0]), int(varSplit[1]))
        print "RPI: Hi Arduino, I sent you ", var
        time.sleep(1)

        number = arm.readOneLink(int(varSplit[0]))
        print "Arduino: Hey RPI, I received a digit ", number
