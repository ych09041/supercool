#include <PID_v1.h>
#define encoder0PinA 2
#define encoder0PinB 3

int incomingByte = 0;  // for incoming serial data


// Controller variables
double Setpoint, Input, Output;
double Kp = 5, Ki = 1, Kd = 0;
double minPoint = -10;
double maxPoint = 10;
double val = 0;
char heading = 'a';
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);

// Velocity Computation Variables
volatile unsigned long encoder0Pos = 0;
unsigned long lastTime, currTime;
double currPos, lastPos, velocity, ticktodeg;
int SampleTime = 5; //5  msec

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
  myPID.SetSampleTime(SampleTime);
  myPID.SetControllerDirection(DIRECT);
  myPID.SetOutputLimits((double) - 1.5, (double) maxPoint);

  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);

  // encoder pin on interrupt 1 (pin 3)
  attachInterrupt(1, doEncoderB, CHANGE);

  // encoder pin on interrupt 0 (pin 2)
  attachInterrupt(0, doEncoderA, CHANGE);

  Serial.begin (9600);
  currPos, lastPos, velocity  = 0;
  ticktodeg = 1.0 / (12.0 * 120.0);

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



void loop() {

  currPos = encoder0Pos * ticktodeg;
  currTime = millis();

  if (Serial.available() > 0) {
    // read the incoming byte:
    incomingByte = Serial.read();
  }

  if (incomingByte == 97) {
    Setpoint = currPos + 5.0;
  }
  if (incomingByte == 100) {
    Setpoint = currPos - 5.0;
  }

  Input = currPos;
  myPID.Compute();

  if (Output > 0) {
    val = myMap(Output, minPoint, maxPoint, 0, 1.0);
    motor_forward_raw(val);
    heading = 'a';
  }
  else {
    val = myMap(-Output, minPoint, maxPoint, 0, 1.0);
    motor_reverse_raw(val);
    heading = 'd';
  }
  
  if(currTime%1000 < SampleTime)
  {
    Serial.println((int)val, DEC);
    Serial.println(Output,DEC);
    Serial.println(currPos);

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
  int out = (int)(in * 255.0);
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
