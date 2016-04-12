#include <Wire.h>

// DIP switch pins
int SW1 = 4; // switch 1, outside
int SW2 = 7; // switch 2, middle
int SW3 = 8; // switch 3, inside

// LED pins
int LED_R = 11;
int LED_G = 12;
int LED_B = 13;

// I2C address and var
int SLAVE_ADDRESS = 0x04; // some initial value, changed later
int state = 0;
char number[10] = {};

void setup() {
  Serial.begin(9600);

  // determine I2C address from switches
  pinMode(13, OUTPUT);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  SLAVE_ADDRESS = !digitalRead(SW1)*1 + !digitalRead(SW2)*2 + !digitalRead(SW3)*4 + 3;
  Serial.print("I2C slave address set to ");
  Serial.println(SLAVE_ADDRESS);
  
  // pinMode LED pins
  pinMode(LED_R,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_B,OUTPUT);

  
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  
  Serial.println("Ready!");
}

void loop() {
  delay(100);
}

// callback for received data
void receiveData(int byteCount) {
  
  int i = 0;
  while(Wire.available()) {
    number[i] = Wire.read();
    
    Serial.print("data received: ");
    long test = number[i];
    Serial.println(test);
    i = i+1;

    if (test == 1) {
      if (state == 0) {
        digitalWrite(13, HIGH); // set the LED on
        state = 1;
      } else{
        digitalWrite(13, LOW); // set the LED off
        state = 0;
      }
    }
  }
}

// callback for sending data
int index = 0;
void sendData() {
  char buf[8];
  n = 1234567;
            
  buf[0] = (char) n;
  buf[1] = (char) n >> 8;
  buf[2] = (char) n >> 16;
  buf[3] = (char) n >> 24;
//  int part1 = (thisIsALong >> (8*0)) & 0xff;
//  int part2 = (thisIsALong >> (8*1)) & 0xff;
//  int part3 = (thisIsALong >> (8*2)) & 0xff;
//  int part4 = (thisIsALong >> (8*3)) & 0xff;
  Wire.write(buf[index]);
  ++index;
  if(index >=4) {
    index = 0;
  } 
}
