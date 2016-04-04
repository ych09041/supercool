Code structure:

Arduino: (Arduino IDE)
- I2C slave implemented with Wire.h library.
- At initialization, the Arduino will determine its I2C address by reading the DIP switch states.
- Arduino will listen for address call and a code string, which can include characters for actions and numbers for position setpoints.
- Arduino will parse the incoming message and respond with an acknowledgement message in the callback; master will receive this message.
- Appropriate protection for invalid commands.
- Arduino will use loop() to implement the PID loop for motor position control using the PID library.
- Appropriate protection for out-of-range setpoints, etc.
- Two click buttons will be used to drive the motor manually in either direction.
- Upon receiving a special designated command, the Arduino will respond with its current encoder position.


RPi: (Python 2)
- I2C master implemented with python smbus library.
- Main software will have different operation modes.
- [Manual control mode]: user will use keyboard to navigate and select which link to control, and use cursor keys to move the unit. 
- Use appropriate library to detect input so that user won't have to repeatedly press Enter.
- [Path recording mode]: after sending a special designated command, the RPi will record all slave Arduino's responses (positions) and record them.
- During playback, RPi will sequentially send these recorded setpoints to the Arduino's.
- At initialization, the RPi will scan and recognize the available I2C addresses.
- Appropriate protection for invalid commands.
- Simple GUI in tkinter?
