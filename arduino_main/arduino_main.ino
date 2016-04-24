#include <Wire.h>
#include <PID_v1.h>

// encoder pins
#define encoder0PinA 2
#define encoder0PinB 3

// motor drive pins
// 2 and 4 should be HIGH at the same time, 1 and 3 same time. NEVER OTHERWISE.
// 3 and 4 HIGH together for braking. 1 and 2 LOW at this time.
#define INPUT1 5 //motor High-side Right-side
#define INPUT2 6 //motor High-side Left-side
#define INPUT3 9 //motor Low-side Left-side
#define INPUT4 10 //motor Low-side Right-side

// LED pins
#define LED_R 11
#define LED_G 12
#define LED_B 13

// DIP switch pins
#define SW1 4 // switch 1, outside
#define SW2 7 // switch 2, middle
#define SW3 8 // switch 3, inside

// I2C address
int SLAVE_ADDRESS = 0x04; // some initial value, changed later
int number = 0;
int state = 0;
char mode = 'c';

int incomingByte = 0;  // for incoming serial data

// Controller variables
double Setpoint, Input, Output;
double Kp = 10, Ki = 1, Kd = 3;
double minPoint = -20;
double maxPoint = 20;
double val = 0;
char heading = 'a';
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Velocity Computation Variables
volatile signed long encoder0Pos = 0;
unsigned long lastTime, currTime;
double currPos, lastPos, velocity, ticktodeg;
int SampleTime = 500; //500  msec

void setup() {
  
  Serial.begin(115200);
  pinMode(INPUT1, OUTPUT);
  digitalWrite(INPUT1, LOW);
  pinMode(INPUT2, OUTPUT);
  digitalWrite(INPUT2, LOW);
  pinMode(INPUT3, OUTPUT);
  digitalWrite(INPUT3, LOW);
  pinMode(INPUT4, OUTPUT);
  digitalWrite(INPUT4, LOW);
  
  pinMode(LED_R,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_B,OUTPUT);

  // determine I2C address from switches
  pinMode(13, OUTPUT);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  SLAVE_ADDRESS = !digitalRead(SW1)*1 + !digitalRead(SW2)*2 + !digitalRead(SW3)*4 + 3;
  Serial.print("I2C slave address set to ");
  Serial.println(SLAVE_ADDRESS);
  
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);
  
  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.println("Calibrating...");
  calibrate();
  
  Input = 0;
  Setpoint = 0; // degrees

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(5);
  myPID.SetControllerDirection(DIRECT);
  myPID.SetOutputLimits((double) minPoint, (double) maxPoint);

  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);

  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);

  currPos, lastPos, velocity  = 0;
  ticktodeg = 1.0 /2.0;
  
  Serial.println("Ready!");

  
}

void loop() {
  // put your main code here, to run repeatedly:

  currPos = encoder0Pos * ticktodeg;
  currTime = millis();

  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();

    if (incomingByte == 97) {
      Setpoint = currPos + 2.0;
      incomingByte = 0;
    }
    if (incomingByte == 100) {
      Setpoint = currPos - 2.0;
      incomingByte = 0;
    }
  }

  Input = currPos;
  myPID.Compute();

  if isAtPosition(){
    if (Output > 0) {
      val = myMap(Output, 0, maxPoint, 0, 1.0);
      motor_forward_raw(val);
      heading = 'a';
    }
    else {
      val = myMap(-Output, 0, maxPoint, 0, 1.0);
      motor_reverse_raw(val);
      heading = 'd';
    }
  }
  else{
    motor_brake_raw()
  }

  
//  digitalWrite(LED_G,HIGH);
//  digitalWrite(LED_R,LOW);
//  digitalWrite(LED_B,LOW);
//  motor_forward_raw(0.2);
//  delay(2000);
//  digitalWrite(LED_G,LOW);
//  digitalWrite(LED_R,HIGH);
//  digitalWrite(LED_B,HIGH);
//  motor_reverse_raw(0.2);
//  delay(2000);
  
}


/*
 * If within 0.5 degrees to final destination we are at final position
 */
void isAtPosition(){
  return abs(currPos - Setpoint) <0.5
  
}


//------------------------------------------------------------

char i2cRead[4];
int i;
// callback for received data
void receiveData(int byteCount) {
  i = 0;
  while(Wire.available()) {
    i2cRead[i] = Wire.read();
    Serial.print("data received: ");
    Serial.println(i2cRead[i]);
    i++;
  }

  mode = i2cRead[0]
}


// callback for sending data
int writeIndex = 0;
void sendData() {
  if (mode == 'c' || mode == 'l' || mode == 'L') {
    sendReady();
  } else if (mode == 'd') {
    sendTime();
  } else if (mode == 'r') {
    sendPos();
  } else {
    Serial.println("Something has gone horribly wrong"); 
  }
}




void motor_forward_raw(float pwm) { // pwm var range 0.0-1.0
  analogWrite(INPUT1, 0);
  analogWrite(INPUT2, 0);
  analogWrite(INPUT3, 0);
  analogWrite(INPUT4, 0);
  analogWrite(INPUT4, 255);
  analogWrite(INPUT2, pwm_float2int(pwm));  
}

void motor_reverse_raw(float pwm) {
  analogWrite(INPUT1, 0);
  analogWrite(INPUT2, 0);
  analogWrite(INPUT3, 0);
  analogWrite(INPUT4, 0);
  analogWrite(INPUT3, 255);
  analogWrite(INPUT1, pwm_float2int(pwm)); 
}

void motor_brake_raw() {
  analogWrite(INPUT1, 0);
  analogWrite(INPUT2, 0);
  analogWrite(INPUT3, 0);
  analogWrite(INPUT4, 0);
  analogWrite(INPUT3, 255);
  analogWrite(INPUT4, 255); 
}

int pwm_float2int(float in) {
  int out = (int)(in*255.0);
  Serial.println(out);
  return out;
}

double myMap(double x, double in_min, double in_max, double out_min, double out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}



void doEncoderB() {
  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {
    // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  // Look for a high-to-low on channel B
  else {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinA) == LOW) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
}

void doEncoderA() {
  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {
      encoder0Pos = encoder0Pos + 1;         // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
    }
  }
  else   // must be a high-to-low edge on channel A
  {
    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == HIGH) {
      encoder0Pos = encoder0Pos + 1;          // CW
    }
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
    }
  }
  //  Serial.println (encoder0Pos, DEC);
  // use for debugging - remember to comment out
}

