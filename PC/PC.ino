#include <PID_v1.h>
#define encoder0PinA 3
#define encoder0PinB 2

int incomingByte = 0;  // for incoming serial data


// Controller variables
double Setpoint, Input, Output;
double Kp = .9, Ki = 0.006, Kd = 0.0;
double minPoint = -24.0;
double maxPoint = 24.0;
double motorpwm = 0;
char heading = 'a';
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Velocity Computation Variables
long minEncoderPos = 0;
long maxEncoderPos = 916;
long midEncoderPos = (maxEncoderPos - minEncoderPos)/2.0;
volatile signed long encoder0Pos = midEncoderPos;
unsigned long velocityLastTime, lastTime, currTime;
double currPos, lastPos, velocity, ticktodeg;
int SampleTime = 700; //500  msec
int velocitySampleTime = 5;

// Motor PWM
int INPUT1 = 5; //motor High-side Right-side
int INPUT2 = 6; //motor High-side Left-side
int INPUT3 = 9; //motor Low-side Left-side
int INPUT4 = 10; //motor Low-side Right-side
// 2 and 4 should be HIGH at the same time, 1 and 3 same time. NEVER OTHERWISE.
// 3 and 4 HIGH together for braking. 1 and 2 LOW at this time.



void setup() {

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
  attachInterrupt(0, doEncoderB, CHANGE);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(1, doEncoderA, CHANGE);

  Serial.begin (9600);
  currPos, lastPos, velocity  = 0;
  ticktodeg = 1.0 /4.789233;

  // Initialize Motor pwm
  pinMode(INPUT1, OUTPUT);
  digitalWrite(INPUT1, LOW);
  pinMode(INPUT2, OUTPUT);
  digitalWrite(INPUT2, LOW);
  pinMode(INPUT3, OUTPUT);
  digitalWrite(INPUT3, LOW);
  pinMode(INPUT4, OUTPUT);
  digitalWrite(INPUT4, LOW);
}

bool isAtPosition(){
  return abs(currPos - Setpoint) <0.5 && abs(velocity) < 0.02;
}

void loop() {

  currPos = (encoder0Pos - midEncoderPos) * ticktodeg;
  currTime = millis();

  if (Serial.available() > 0 && abs(Setpoint - currPos) < 1.0 && abs(velocity) < 0.02) {
    // read the incoming byte:
    incomingByte = Serial.read();
    // a
    if (incomingByte == 97) {
      Setpoint = currPos +1.0;
//      Setpoint = 10.0;
      incomingByte = 0;
      resetLimits();
    }
    if (incomingByte == 100) {
      Setpoint = currPos-1.0;
//      Setpoint = -10.0;
      incomingByte = 0;
      resetLimits();
    }
    if (incomingByte == 115) {
      Setpoint = 0.0;
      incomingByte = 0;
    }
  }

  Input = currPos;
  myPID.Compute();

  if (!isAtPosition()) {
    if (Output > 0) {
      motorpwm = myMap(Output, 0, maxPoint, 0, 1.0);
      motor_forward_raw(motorpwm);
      heading = 'a';
    }
    else {
      motorpwm = myMap(-Output, 0, maxPoint, 0, 1.0);
      motor_reverse_raw(motorpwm);
      heading = 'd';
    }
  }
  else{
    motor_brake_raw();
  }

  long velocityTimeChange = (currTime-velocityLastTime);

  if(velocityTimeChange >= velocitySampleTime)
  {
    velocity = (currPos - lastPos)/velocityTimeChange;
//    Serial.println(velocity);
    velocityLastTime = currTime;
    lastPos = currPos;
  }
  int timeChange = (currTime-lastTime);

  if(timeChange >=  SampleTime)
  {
    Serial.println("New");
    Serial.println((int)motorpwm, DEC);
    Serial.println(Output,DEC);
    Serial.println(currPos);
    Serial.println(encoder0Pos);
    Serial.println(abs(velocity),DEC);
    lastTime = currTime;
  }
}


void resetLimits(){
  if (abs(Setpoint) >= 4.0 ){
        minPoint = (double) -1.0*abs(Setpoint)*6.0;
        maxPoint = (double) abs(Setpoint)*6.0;
        myPID.SetOutputLimits(minPoint, maxPoint);
      }
      else{
        minPoint = (double) -1.0*abs(4.0)*6.0;
        maxPoint = (double) abs(4.0)*6.0;
        myPID.SetOutputLimits(minPoint, maxPoint);
      }
}


void motor_forward_raw(float pwm) { // pwm var range 0.0-1.0
  analogWrite(INPUT1, 0);
  analogWrite(INPUT2, 0);
  analogWrite(INPUT3, 0);
  analogWrite(INPUT4, 0);
  analogWrite(INPUT4, 255);
  analogWrite(INPUT2, constrain(pwm_float2int(pwm),0,5));

}

void motor_reverse_raw(float pwm) {
  analogWrite(INPUT1, 0);
  analogWrite(INPUT2, 0);
  analogWrite(INPUT3, 0);
  analogWrite(INPUT4, 0);
  analogWrite(INPUT3, 255);
  analogWrite(INPUT1, constrain(pwm_float2int(pwm),0,5));
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
//  Serial.println(out);
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
