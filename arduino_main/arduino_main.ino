#include <Wire.h>
#include <PID_v1.h>

#define TICKS_PER_DEGREE 4.789233*1.125 //1.125 turns ~90deg to 80deg
#define DEGREES_PER_TICK 1.0/TICKS_PER_DEGREE
#define CONTROL_PERIOD 20

// button switch pins

#define BUTTON_R 0
#define BUTTON_L 1
//#define BUTTON_L A0
//#define BUTTON_R A1 // only when debugging. Once done, switch to D0/D1 and disable all Serial actions.


// encoder pins
#define encoder0PinA 3
#define encoder0PinB 2

#define maxSetpoint 33
#define minSetpoint -33
int DEAD_BAND;

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
int SLAVE_ADDRESS = 0x04; // some initial motorpwmue, changed later
int number = 0;
int state = 0;
char mode = 'c';
volatile int calibrated = 1;

int incomingByte = 0;  // for incoming serial data

// Controller variables
double Setpoint, Input, Output;
double Kp = .0012, Ki = 0.0008, Kd = 0.00024;

int loopCount = 0;

double minPoint = -1.0;
double maxPoint = 1.0;

double motorpwm = 0;
char heading = 'a';
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Velocity Computation Variables
long minEncoderPos = 0;
long maxEncoderPos = 80 * TICKS_PER_DEGREE;// Needs to be calibrated
long midEncoderPos = (maxEncoderPos - minEncoderPos) / 2.0;
volatile signed long encoder0Pos = 0;
unsigned long velocityLastTime, lastTime, currTime;
double currPos, lastPos, velocity, ticktodeg;
int SampleTime = 700; //700  msec
int velocitySampleTime = CONTROL_PERIOD;


char i2cmotorpwm[4];
int i;

void setup() {

  //Serial.begin(115200);
  pinMode(INPUT1, OUTPUT);
  digitalWrite(INPUT1, LOW);
  pinMode(INPUT2, OUTPUT);
  digitalWrite(INPUT2, LOW);
  pinMode(INPUT3, OUTPUT);
  digitalWrite(INPUT3, LOW);
  pinMode(INPUT4, OUTPUT);
  digitalWrite(INPUT4, LOW);

  pinMode(LED_R, OUTPUT);
  pinMode(LED_G, OUTPUT);
  pinMode(LED_B, OUTPUT);

  pinMode(BUTTON_L, INPUT_PULLUP); // the buttons read LOW when pressed
  pinMode(BUTTON_R, INPUT_PULLUP);

  // determine I2C address from switches
  pinMode(13, OUTPUT);
  pinMode(SW1, INPUT_PULLUP);
  pinMode(SW2, INPUT_PULLUP);
  pinMode(SW3, INPUT_PULLUP);
  SLAVE_ADDRESS = !digitalRead(SW1) * 1 + !digitalRead(SW2) * 2 + !digitalRead(SW3) * 4 + 3;
  Serial.print("I2C slave address set to ");
  Serial.println(SLAVE_ADDRESS);


  if (SLAVE_ADDRESS == 7) {
    DEAD_BAND = 40;
  } else if (SLAVE_ADDRESS == 3) {
    DEAD_BAND = 34;
  } else if (SLAVE_ADDRESS == 6) {
    DEAD_BAND = 38;
  } else if (SLAVE_ADDRESS == 4) {
    DEAD_BAND = 38;
  } else {
    DEAD_BAND = 38;
  }

  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  
  Serial.println("Calibrating...");
  
  //calibrate();

  Input = 0;
  Setpoint = 0; // degrees

  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  myPID.SetSampleTime(CONTROL_PERIOD);
  myPID.SetControllerDirection(DIRECT);
  myPID.SetOutputLimits((double) minPoint, (double) maxPoint);

  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);

  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(0, doEncoderB, CHANGE);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderA, CHANGE);

  currPos, lastPos, velocity  = 0;
  
  //Serial.println("Initial calibration...");
  //calibrate();
  
  
  Serial.println("Ready!");
}

bool isAtPosition() {
  bool velo_check = abs(velocity) < 0.02;
  bool pos_check = abs(currPos - Setpoint) < 1.0;
  digitalWrite(LED_G, velo_check);
  digitalWrite(LED_B, pos_check);
  return velo_check && pos_check;
}


float prevVel=0;
float prevPrevVel=0;
//double tempKp;

int MAX_LOOPS = 500;
double tempKi = Ki;
double vel_target = 0.1;

double rampFunc(int toRamp) {
  return pow(toRamp/50, 10);
}

void loop() {
  if (Setpoint > maxSetpoint) Setpoint = maxSetpoint;
  if (Setpoint < minSetpoint) Setpoint = minSetpoint;
  
  if (!calibrated) calibrate();
  
  currPos = encoder0Pos * DEGREES_PER_TICK;
  //currPos = (encoder0Pos - midEncoderPos) * DEGREES_PER_TICK;
  Input = currPos;
  currTime = millis();


  // check buttons for manual move
  if (!digitalRead(BUTTON_L)) { // if button is held pressed
    Setpoint = max(currPos - 5.0, minSetpoint);
    Serial.println(Setpoint);
  } else if (!digitalRead(BUTTON_R)) {
    Setpoint = min(currPos + 5.0, maxSetpoint);
    Serial.println(Setpoint);
  }
  //tempKp = gainScheduling(Kp, currPos, Setpoint);
  //myPID.SetTunings(tempKp, Ki, Kd);
  tempKi = min(2*rampFunc(loopCount), 2*rampFunc(MAX_LOOPS)) * Ki * max(10*(velocity - vel_target), 1);
  myPID.SetTunings(Kp,tempKi,Kd);
  myPID.Compute();

  if (!isAtPosition()) {
    loopCount++;
    if (Output > 0) {
      motor_forward_raw(Output);
      heading = 'a';
    }
    else {
      motor_reverse_raw(Output);
      heading = 'd';
    }
  }
  else {
    loopCount = 0;
    myPID.ZeroITerm();
    motor_brake_raw();
  }

  long velocityTimeChange = (currTime - velocityLastTime);
  
  if (velocityTimeChange >= velocitySampleTime)
  {
    velocity = (prevPrevVel + prevVel + (currPos - lastPos) / velocityTimeChange)/3;
    //Serial.println(velocity);
    prevPrevVel = prevVel;
    prevVel = velocity;
    velocityLastTime = currTime;
    lastPos = currPos;
  }

  delay(CONTROL_PERIOD);
}


//------------------------------------------------------------
//#define minKp Kp*0.5
//#define maxKp Kp*1.5
//
//double gainScheduling(double _Kp, double _currPos, double _Setpoint) {
//  if (_currPos >= 0.0 && _Setpoint < _currPos) { // in positive, trying to come up
//    return minKp + (maxSetpoint-_currPos) / maxSetpoint * (maxKp - minKp);
//  } else if (_currPos <= 0.0 && _Setpoint > _currPos) { // in negative, trying to come up
//    return minKp + (_currPos-minSetpoint) / maxSetpoint * (maxKp - minKp);
//  } else {
//    return _Kp;
//  }
//}

//------------------------------------------------------------


// callback for received data
void receiveData(int byteCount) {
  // clears buffer
  for (int k = 0; k < 4; k++) {
    i2cmotorpwm[k] = '\0';
  }
  i = 0;
  while (Wire.available()) {
    if (i == 0) {
      mode = Wire.read();
      //Serial.print("Mode received: ");
      Serial.println(mode);
    } else {
      i2cmotorpwm[i-1] = (char)Wire.read();
      Serial.print("Number received: ");
      Serial.println(i2cmotorpwm[i - 1]);
    }
    i++;
  }
  interp();
}


// callback for sending data
int sendIndex = 0;
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

//---------------------------------------------------------------
//Abstracted functions for doing command interperation

void interp() {
  Serial.print("setpoint before change: ");////////////
  Serial.println(Setpoint);//////////////
  if (mode == 'c') {
    calibrated = 0;
  } else if (mode == 'l') {
    Setpoint += atof(i2cmotorpwm);
    if (Setpoint > maxSetpoint) Setpoint = maxSetpoint;
    if (Setpoint < minSetpoint) Setpoint = minSetpoint;
    Serial.println(i2cmotorpwm);////////////////
    Serial.println(atof(i2cmotorpwm));/////////////
    Serial.print("Motor setpoint: ");
    Serial.println(Setpoint);
  } else if (mode == 'L') {
    Setpoint = atof(i2cmotorpwm);
    if (Setpoint > maxSetpoint) Setpoint = maxSetpoint;
    if (Setpoint < minSetpoint) Setpoint = minSetpoint;
    Serial.println(i2cmotorpwm);
    Serial.print("Motor setpoint: ");
    Serial.println(Setpoint);
  }
}

void calibrate() {
  Serial.println("Start calibration...");
  myPID.SetMode(MANUAL);
  //int count_loop = 0;
  digitalWrite(LED_R, LOW);
  motor_forward_raw(0.0);
  while ((digitalRead(BUTTON_R))) {
    if (!digitalRead(BUTTON_L)) {
      motor_brake_raw();
      calibrated = 1;
      //encoder0Pos = 0.0;
      currPos = encoder0Pos * DEGREES_PER_TICK;
      //currPos = 0.0;
      Input = currPos;
      Setpoint = currPos;
      return;
    }
    //motor_brake_raw();
    //delay(10);
  }
  motor_brake_raw();
  digitalWrite(LED_R, HIGH);
  encoder0Pos = (maxSetpoint+1) * TICKS_PER_DEGREE;
  currPos = (maxSetpoint+1);
  Input = currPos;
  myPID.SetMode(AUTOMATIC);
  Setpoint = 0.0;
  motor_reverse_raw(0.25);
  delay(300);
  motor_brake_raw();
  delay(1000);
  calibrated = 1;
  Serial.println("Calibrated.");
}


void clamp(double *setpoint) {
  if (*setpoint > maxSetpoint) {
    *setpoint = maxSetpoint;
  } else if (*setpoint < minSetpoint) {
    *setpoint = minSetpoint;
  }
}



//---------------------------------------------------------------
//Abstracted functions for sending data

void sendReady() {
  if (isAtPosition()) {
    Wire.write('1');
  } else {
    Wire.write('0');
  }
}


unsigned long sendTimeTemp;
void sendTime() {
  sendTimeTemp = millis();
//  Serial.println(sendTimeTemp);
  sendTimeTemp = (sendTimeTemp >> (8 * sendIndex)) & 0xff;
//  Serial.println(sendTimeTemp);
  Wire.write(sendTimeTemp);
  sendIndex++;
  if (sendIndex > 3) {
    sendIndex = 0;
  }
}

long sendPosTemp = 0L;
void sendPos() {
  sendPosTemp = (long) currPos + 50L;
//  Serial.println("sendPosTemp original");
  //Serial.println(sendPosTemp);
  sendPosTemp = (sendPosTemp >> (8 * sendIndex)) & 0xff;
 // Serial.println("===");
  Serial.println(sendPosTemp);
  Wire.write(sendPosTemp);
  sendIndex++;
  if (sendIndex > 3) {
    sendIndex = 0;
  }
  Serial.println(sendIndex);
}

//---------------------------------------------------------------

#define MAX_PWM 230
#define RAMP_CONST 200

void motor_forward_raw(float pwm) { // pwm var range 0.0-1.0
  analogWrite(INPUT1, 0);
  analogWrite(INPUT2, 0);
  analogWrite(INPUT3, 0);
  analogWrite(INPUT4, 0);
  analogWrite(INPUT4, 255);
  analogWrite(INPUT2, constrain(constrain(pwm_float2int(pwm), DEAD_BAND, DEAD_BAND + MAX_PWM - velocity*RAMP_CONST), 0, 255));
}

void motor_reverse_raw(float pwm) {
  analogWrite(INPUT1, 0);
  analogWrite(INPUT2, 0);
  analogWrite(INPUT3, 0);
  analogWrite(INPUT4, 0);
  analogWrite(INPUT3, 255);
  analogWrite(INPUT1, constrain(constrain(pwm_float2int(pwm), DEAD_BAND, DEAD_BAND + MAX_PWM - velocity*RAMP_CONST), 0, 255));
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
  int out = (int)(in * 255.0);
  //Serial.println(out);
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

