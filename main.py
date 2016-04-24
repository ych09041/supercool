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




    def writeOneLink(self,address,command):
        ##NEEDS TO BE UPDATED
        """Author: Cheng Hao Yuan, Stephen Lu

        Inputs:
            address: i2c address to be written to
            command: the command to write to the addressed slave. Not necessarily numeric.
            command will a letter followed by a number (no space in between). The letter cmdChar can be:
                l = Link relative movement, return True when finished, False otherwise
                L = Link absolute movement, return True when finished, False otherwise
                r = Running Record, return current position
                d = Running Detect, return time since turned on (return millis())
                c = calibrate, return True when finished, False otherwise
            the number is the position setpoint.


        Writes 'command' to the device at 'address' over i2c

        returns nothing

        """
        if not command.isalnum():
            ## Call error printout function
            return

        cmdChar = command[0]
        if cmdChar not in "lLrdc":
            ## Call error printout function
            return

        if len(command) > 1:
            value = int(command[1:])
            if value > self.linkMax:
                print "Trying to move too far!"
                value = self.linkMax
            elif value < self.linkMin:
                print "Trying to move too far!"
                value = self.linkMin
            newCommand = cmdChar + value
        else:
            newCommand = cmdChar
            
        self.bus.write_byte(address, newCommand)

        return

    def readOneLink(self,address):
        ##NEEDS TO BE UPDATED
        """Author: Stephen Lu

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
        """Author: Cheng Hao Yuan

        Input:
            string: the user inputted string at the console

        Interprets user commands and calls the correct method. Pass the methods "string"
        Refer to Codebook and Conventions.txt for dictionary of operations.

        returns nothing.
        """
        ## bring all input to lower case and strip leading/trailing whitespaces
        string = string.lower().strip()

        ## help
        if string.startswith("help"):
            self.helpHelp(string)

        ## link (single link drive)
        elif string.startswith("link"):
            self.link(string)

        ## arm (arm drive)
        elif string.startswith("arm"):
            self.arm(string)

        ## record
        elif string.startswith("record"):
            self.mode = "Record"
            print "Arm is now in Record mode. Linkage calibration", \
                "is recommended before recording any motions."
            self.record(string)

        ## open (open csv file)
        elif string.startswith("open"):
            self.openOpen(string)

        ## close (close csv file)
        elif string.startswith("close"):
            self.close(string)

        ## run
        elif string.startswith("run"):
            self.run(string)

        ## calibrate
        elif string.startswith("calibrate"):
            self.calibrate(string)

        ## detect
        elif string.startswith("detect"):
            self.calibrate(string)

        ## no match found (invalid command you need help)
        else:
            print "Invalid command.", \
            "Please type \"help\" to see the operation manual."


        return
    '''
    ## DEPRICATED
    def DirectDrive(self,string):
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
    '''

    def helpHelp(self, string):
        """Author:Yiran 

        Inputs:
            string: the user input string.

        Prints out the help documentation- what the various console commands are and how to
        use them.

        returns nothing
        """

        if len(string) > 5 :
            print "ERROR: Type 'Help' for help"

        else:
            print "Help Manual"
            print "Console Commands: \n 1.Link \n 2. Arm \n 3. Record \n 4. Open \n 5. Close \n 6. Run \n 7. Calibrate \n 8. Detect"
        Num=input("Enter a number for comand help: ")

        if Num == 1:
            print "\n Link \n\n Executes a movement for 1 link specified by the input. Input formats detailed below. Inputs " \
                  "not matching the formats should fail gracefully (print error, go back to console). " \
                  "See belowconventions for interpreting relative and absolute position:" \
                  "\n\n Link Link# Value" \
                  "\n\t Link# indicates the target link that we want to actuate. e.g. Link#=1 indicates we are to move the bottom linkage. " \
                  "Value indicates the # of degrees that we want to move the arm left or right. Value is allowed to be negative! " \
                  "\n\n Link Link# Value -abs" \
                  "\n\t Link# is the same as above" \
                  "\n\t With the optional flag -abs, Value is now interpreted as the absolute position to go to."
        elif Num == 2:
            print "\n Arm \n\n Executes a movement for the entire arm. Input format detailed below. " \
                  "Inputs not matching the formats shoudld fail gracefully (print error, go back to console). " \
                  "See below conventions for interpreting relative and absolute position." \
                  "\n\n Arm List-Of-Numbers" \
                  "\n\t List-Of-Nubmers is a list of numbers seperated by spaces. Length of list is equal to size of arm. Reject the input if the length is different." \
                  "Moves each linkage by the amount specified in the list (relative position)" \
                  "\n\n Arm List-Of-Numbers -abs" \
                  "\n\t With optional flag -abs, the values are now used as absolute position."
        elif Num == 3:
            print "\n Record \n\n Records the current status of the arm, or records waits (meaning pause in motion). " \
                  "Saves data to the open file, throws error if file does not exist. Must follow data recording convention noted below. " \
                  "\n\n Record" \
                  "\n\t Records all the positions of the arm." \
                  "\n\n Record Wait Time" \
                  "\n\t Records a wait in the control sequence of length Time in milliseconds."
        elif Num == 4:
            print "\n Open \n\n Opens a file. Must be able to recognize proper input refuse to open non-csv files), " \
                  "and fail gracefully when unable to locate the specified file. Must also check to see if there is already " \
                  "a file open- rejects input if this is the case." \
                  "\n\n Open FilePath" \
                  "\n\t FilePath is the path to the desired file. If the file does not exist, create it. " \
                  "If it does exist, must prompt user to choose to overwrite, or append to it."
        elif Num == 5:
            print "\n Close \n\n Closes the currently open file. Fails gracefully if no file is currently open. " \
                  "Input should be just the word Close, and nothing else. Reject input if it does not match." \
                  "\n\n Close"
        elif Num == 6:
            print "\n Run \n\n Runs the currently open file or runs the saved file at the specified path " \
                  "(okay to run even if there is an open file). By Run, we mean that it goes through the recorded path specified " \
                  "by the file. Blocks all standard user console commands while executing file, but has emergency abort command/function." \
                  "\n\n Run" \
                  "\n\tRuns the currently open file. Fail gracefully if nothing open. Should prompt user to run `help' when failing." \
                  "\n\n Run path" \
                  "\n\t Run the file at path. Fail gracefully if file is wrong format, invalid input for current arm configuration " \
                  "(e.g. file is for a 4-link arm, but currently only have 2 links), and if file does not exist."
        elif Num == 7:
            print "\n Calibrate \n\n Calibrates the linkages' various encoder positions. Can either calibrate all the links, " \
                  "or just specific links." \
                  "\n\n Calibrate linkNumber" \
                  "\n\t Forces the linkNumber-th link to calibrate. Disables operation until calibration is complete. " \
                  "Must fail gracefully if linkNumber is out of range." \
                  "\n\n Calibrate -all" \
                  "\n\t Keyword -all changes the behavior to calibrate all the linkages along the arm (one at a	time!). " \
                  "Disables all other operations until calibration is complete. Should prompt user with a warning that " \
                  "this operation may take a while before continuing with execution " \
                  "(should have the ability to back out of the operation at this prompt)."
        elif Num == 8:
            print "\n Detect \n\n Detects which linkages are on the arm. Prints out the list of linkage's i2c addressess in " \
                  "order of linkage position. Input must match exactly. Saves the detected positions to the internal position tracker." \
                  "\n\n Detect"
        else :
            print "Type 'Help' for the help manual and enter a number between 1 to 8 for help on console commands." \
                  "\n Thank you!"            
        return

    def link(self, string):
        """Author: Cheng Hao Yuan, Stephen Lu

        Inputs:
            string: the user input string

        Executes the Link command functionality. Should parse inputs correctly and fail
        gracefully. This method is called when first word in "string" is "Link".

        returns nothing
        """
        parsedString = string.split()
        if len(parsedString) == 3 and parsedString[0] == "link":
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
        return


    def arm(self, string):
        """Author: Cheng Hao Yuan, Stephen Lu

        Inputs:
            string: the user input string

        Executes the Arm command functionality. This method is called when the first word
        in "string" is "Arm".

        returns nothing
        """

    def record(self,string):
        """Author: Chris Berthelet

        Does the back-end execution of the `Record' console command. Called when the first
        word in string is "Record".
        
        Records the current status of the arm, or records waits (meaning pause in motion). Saves data
        to the open file, throws error if file does not exist. Must follow data recording convention 
        noted below.

        Record
        Records all the positions of the arm.

        Record Wait Time
        Records a wait in the control sequence of length Time in milliseconds.
        
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
        
        ## Read in string and check if "Record" or if "Record Wait time"
        
        ## case insensitive
        string = string.lower()
        ## create array of all words in string that are separated by spaces
        stringArray = string.split()
        ## count how many words are in the array 
        numberOfWords = len(stringArray)
        
        if numberOfWords == 1:
            ## this means that the only word is "Record" and thus a whole line is recorded    
            theFile = open(self.file,'a')
            
            addressLinks = self.positions
            
            absPositions = []
            
            for add1 in addressLinks: 
                
                absPositions.append(self.readOneLink(add1)) 
            
    
            
            theFile.write(b[0] + '\n')
        
        elif numberOfWords <= 3:
            ## this means that the input may be "Record Wait" or some invalid input
            if stringArray[1] == "wait":
                
                if stringArray[2].isnumeric() == True:
                    ## this means the third input is a valid wait time (ms)
                    pass    
                else:
                    print "INVALID INPUT FOR WAIT TIME"
            
            else:
                print "INVALID INPUT"
            
        else:
            ## this means there are too many inputs
            print "TOO MANY INPUTS"
            
        return

    def openOpen(self,string):
        """Author: Chenliu Stephen Lu

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
        if parsedString[0] != "Open" or parsedString[0] != "open":
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
        """Author: Chenliu Stephen Lu

        Inputs:
            string: total user input at the console

        Closes the open csv. Print warning if no csv is open.

        returns nothing
        sets self.file to None"""

        if string != "Close" or string != "close":
            self.badInput()
            return
        elif self.file is None:
            print "No file is open. Operation aborted."
            return
        else:
            self.file.close()
            self.file = None
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
        """Author: Cheng Hao Yuan

        Inputs:
            string: total user input at the console

        Read CodeBook and Conventions.txt for detailed operation

        returns nothing
        sets self.positions with udpated locations"""
        if string == "detect":
            self.posDetect()
        else:
            print "Invalid command for detect."
        return


    def badInput(self):
        """Author: Chenliu Stephen Lu

        Inputs: None

        Helper function for printing out a generic error message. Tells people to run 'help'

        returns nothing"""

        print "Bad input detected. Run 'help' for a list of commands."

        return
        



arm = ArmObj(bus)

print "Welcome to the Modular Arm Command Interface."\
      "Type \"help\" for a list of commands."

while True:
    var = raw_input("Command: ")
    arm.interpretCommand(var)
