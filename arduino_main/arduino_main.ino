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
  
  Serial.println("Ready!");

  
}

void loop() {
  // put your main code here, to run repeatedly:
  digitalWrite(LED_G,HIGH);
  digitalWrite(LED_R,LOW);
  digitalWrite(LED_B,LOW);
  motor_forward_raw(0.2);
  delay(2000);
  digitalWrite(LED_G,LOW);
  digitalWrite(LED_R,HIGH);
  digitalWrite(LED_B,HIGH);
  motor_reverse_raw(0.2);
  delay(2000);
  
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

//---------------------------------------------------------------



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



