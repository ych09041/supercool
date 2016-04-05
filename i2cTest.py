import smbus
import time
bus = smbus.SMBus(1)

slaveAddress = 0x05

def writeNumber(address,value):
    bus.write_byte(address, value)
    return -1

def readNumber(address):
    number = bus.read_byte(address)
    return number

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
