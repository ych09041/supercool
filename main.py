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
        i2cMin = smallest i2c address
        i2cMax = largest i2c address
        posDetect = fills the positions list.
        file = the csv file of positions that we are reading

        """
        
        
        self.bus = busLine
        self.positions = []
        self.errorState = False
        self.linkMin = 0
        self.linkMax = 500
        self.i2cMin = i2cMinRange
        self.i2cMax = i2cMaxRange
        self.posDetect()
        self.mode = "Idle"

        self.file = None

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


        

    def writeOneLink(self,address,value):
        """Writes 'value' to the device at 'address' over i2c

        No Fail-Safes in place. Maybe add one to see if the i2c address is present

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

    def interpretCommand(self,string):
        """Interprets and executes user commands, and calls upon the Arduino commands"

        Codes:
        "Idle": puts the arm in idle. Ignores commands until put into a 'working' mode
        "Direct Drive": lets the user input the desired encoder position for each
        "Record": lets the user record arm motion. Actuation can be done either through
            buttons on the linkage or keyboard command. Direct Drive functionality is
            built into Record.
        """

        if string == "Idle":
            self.mode = "Idle"
            print "Arm is now in Idle mode"
            return
        if string == "Direct Drive":
            self.mode = "Direct Drive"
            print "Arm is now in Direct Drive mode"
            return
        if string == "Record":
            self.mode = "Record"
            print "Arm is now in Record mode. Linkage calibration", \
                "is recommended before recording any motions."
            return




        if self.mode == "Direct Drive":
            self.DirectDrive(string)
            return
        if self.mode == "Record":
            self.Record(string)
            return


        return


    def DirectDrive(self,string):
        """Processes commands for the direct drive state.

        Valid input format:
        Link [Link#] [Position]
        Arm [Position Link #1] [Position Link $2] ...
        """

        ##Optional: run posDetect before executing commands to automatically have the
        ##most up-to-date arm i2c addresses. Might be good for robustness/protection
        ##self.posDetect()

        parsedString = string.split()
        if len(parsedString) == 3 and parsedString[0] == "Link":
            try:
                linkNumber = int(parsedString[1])
                targetPosition = int(parsedString[2])
            except ValueError:
                print "ERROR: Gibberish input for link number and/or desired position"
                return
            if len(self.positions) >= linkNumber and linkNumber >= 0:
                self.writeOneLink(self.positions[linkNumber - 1], targetPosition)
            else:
                print "ERROR: Target link is out of range"

        else if parsedString[0] == "Arm":
            targetArmPosition = []
            try:
                for val in parsedString[1:]):
                    targetArmPosition.append(int(val))
            except ValueError:
                print "ERROR: Gibberish input for 1 or more desired positions"
                return
            self.writeArm(targetArmPosition)

        else:
            print "ERROR: Gibberish input"
            return

    def Record(self,string):

        parsedString = string.split()
        if parsedString[0] == "Open" and len(parsedString) == 2:
            


        


arm = ArmObj(bus)

print "Welcome to the Modular Arm Command Interface."\
      "Type \"help\" for a list of commands."

while True:
    var = raw_input("Enter slave address, number: ")
    arm.interpretCommand(var)
