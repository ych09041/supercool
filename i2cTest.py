import smbus
import time
import numpy as np
bus = smbus.SMBus(1)


slaveAddress = 0x05

def writeNumber(address,value):
    bus.write_i2c_block_data(address, 0, [1])
    return -1

def readNumber(address):
    ##number = bus.read_byte(address)
    testingArray = bus.read_i2c_block_data(address, 0)
    return testingArray

def read_block_dumb(address, numbytes):
    data = ""
    for i in range(0, numbytes):
        received = chr(bus.read_byte(address))
        data += received
        print ord(received)
        print "\n"
    return data

while True:
    var = raw_input("Enter slave address, number: ")
    varSplit = var.split(",")
    if not var:
        continue
    writeNumber(int(varSplit[0]), [int(varSplit[1])])
    print "RPI: Hi Arduino, I sent you ", var
    time.sleep(1)

    ##number = readNumber(int(varSplit[0]))
    number = read_block_dumb(int(varSplit[0]), 4) 
    print "Arduino: Hey RPI, I received a digit ", number
