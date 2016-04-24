import smbus
import time
import os
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

        self.file = None

    def posDetect(self):
        ##NEEDS TO BE UPDATED
        """Author:

        Pings all potential i2c addresses of linkages, and stores the addresses of those that respond back.
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
        ##NEEDS TO BE UPDATED
        """Author:

        Input:
            address: the i2c address to ping
    
        Pings the given i2c address. Returns the millis() of the Arduino if it is there,
        returns -1 otherwise"""
                                   
        
        try:
            bus.write_byte(address,-1)
            number = bus.read_byte(address)
            return number
        except IOError:
            return -1


        

    def writeOneLink(self,address,value):
        ##NEEDS TO BE UPDATED
        """Author:

        Inputs:
            address: i2c address to be written to
            value: the value to write to that address. Not necessarily numeric.

        Writes 'value' to the device at 'address' over i2c

        returns nothing

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
        ##NEEDS TO BE UPDATED
        """Author:

        Input:
            address: the i2c address of the link to read from.

        Gets data from the Arduino at address and returns it.

        return:
            the data it receives

        """
        number = self.bus.read_byte(address)
        return number

    def writeArm(self,lis):
        ##NEEDS TO BE UPDATED
        """Author:

        Input:
            lis: a list of integers, 
        Writes each element of lis to the corresponding linkage by position ordering.
        Failsafe checks to see if lis is the same length as the number of linkages on the arm. If there is a mismatch, nothing is done.

        returns nothing
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
        ##NEEDS TO BE UPDATED
        """Author:

        Input:
            string: the user inputted string at the console

        Interprets user commands and calls the correct method. Pass the methods "string"
        Refer to Codebook and Conventions.txt for dictionary of operations. 

        returns nothing.
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


    def DirectDrive(self,):
        ##NEEDS TO BE UPDATED. SHOULD BE MERGED INTO interpretCommand
        """Author:

        Processes commands for the direct drive state.

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

        elif parsedString[0] == "Arm":
            targetArmPosition = []
            try:
                for val in parsedString[1:]:
                    targetArmPosition.append(int(val))
            except ValueError:
                print "ERROR: Gibberish input for 1 or more desired positions"
                return
            self.writeArm(targetArmPosition)

        else:
            print "ERROR: Gibberish input"
            return

   
    def helpHelp(self, string):
        """Author:

        Inputs:
            string: the user input string.

        Prints out the help documentation- what the various console commands are and how to
        use them.

        returns nothing
        """


        return

    def link(self, string):
        """Author:

        Inputs:
            string: the user input string

        Executes the Link command functionality. Should parse inputs correctly and fail
        gracefully. This method is called when first word in "string" is "Link".

        returns nothing
        """

    def arm(self, string):
        """Author:

        Inputs:
            string: the user input string

        Executes the Arm command functionality. This method is called when the first word
        in "string" is "Arm".

        returns nothing
        """

    def record(self,string):
        """Author:

        Does the back-end execution of the `Record' console command. Called when the first
        word in string is "Record".
        
        Inputs:
            string: total user input at the console in string format

        Function:
            Upon receiving just the word `Record', gets
                all the linkage positions and saves them in the
                .csv file specified by self.file
                Please note to follow the data recording convention
        Errors:
            Bad input- extra words/numbers in string
            
            self.file is closed or does not exist. Should print error statement
                instead of failing.
            
        Returns:
            Nothing"""

        
        return

    def openOpen(self,string):
        """Author:

        Inputs:
            string: total user input at the console

        Opens csv files (follow CodeBook and Conventions.txt). Should fail if one is already
        open. Sets self.file to the target file with correct mode of operation ('w' or 'a')
        (refer to: https://docs.python.org/2/tutorial/inputoutput.html)

        returns nothing
        sets self.file"""

        parsedString = string.split()

        if len(parsedString) > 2:
            self.badInput()
            return  
        if self.file is not None:
            print "There's already a file open; please close it before opening another"
            return
        if not parsedString[1].endswith('.csv'):
            print "File path needs to end in \'.csv\'"
            return
        else:
            if os.path.isfile(parsedString[0]):
                userInput = raw_input("Specified file already exists. (O)verwrite or (A)ppend? ")
                if userInput == "O" or userInput == "o":
                    try:
                        self.file = open(parsedString[0], 'w+')
                    except IOError:
                        print "Permission denied to open the file. Operation aborted."
                    return
                elif userInput == "A" or userInput == "a":
                    try:
                        self.file = open(parsedString[0], 'w+')
                    except IOError:
                        print "Permission denied to open the file. Operation aborted."
                    return
                else:
                    self.badInput()
                    return
            else:
                return
                
                


    def close(self,string):
        """Author:

        Inputs:
            string: total user input at the console

        Closes the open csv. Print warning if no csv is open.

        returns nothing
        sets self.file to None"""

        return

    def run(self,string):
        """Author:

        Inputs:
            string: total user input at the console

        Read CodeBook and Conventions.txt for detailed operation

        returns nothing"""

        return

    def calibrate(self,string):
        """Author

        Inputs:
            string: total user input at the console

        Read CodeBook and Conventions.txt for detailed operation

        returns nothing
        set points on the Arduino end should change"""
    
        return

    def detect(self,string):
         """Author

        Inputs:
            string: total user input at the console

        Read CodeBook and Conventions.txt for detailed operation

        returns nothing
        sets self.positions with udpated locations"""

         return

    def badInput(self):
        """Author: Chenliu Stephen Lu

        Inputs: None

        Helper function for printing out a generic error message. Tells people to run 'help'

        returns nothing"""

        print "Bad input detected. Run 'help' for a list of commands."
        


arm = ArmObj(bus)

print "Welcome to the Modular Arm Command Interface."\
      "Type \"help\" for a list of commands."

while True:
    var = raw_input("Command: ")
    arm.interpretCommand(var)
