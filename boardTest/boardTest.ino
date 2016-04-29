int INPUT1 = 5; //motor High-side Right-side
int INPUT2 = 6; //motor High-side Left-side
int INPUT3 = 9; //motor Low-side Left-side
int INPUT4 = 10; //motor Low-side Right-side
// 2 and 4 should be HIGH at the same time, 1 and 3 same time. NEVER OTHERWISE.
// 3 and 4 HIGH together for braking. 1 and 2 LOW at this time.

int LED_R = 11;
int LED_G = 12;
int LED_B = 13;

#define BUTTON_L 1
#define BUTTON_R 0
bool LED_G_ON = 0;
bool LED_B_ON = 0;
 


void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(INPUT1, OUTPUT);
  digitalWrite(INPUT1, LOW);
  pinMode(INPUT2, OUTPUT);
  digitalWrite(INPUT2, LOW);
  pinMode(INPUT3, OUTPUT);
  digitalWrite(INPUT3, LOW);
  pinMode(INPUT4, OUTPUT);
  digitalWrite(INPUT4, LOW);

  pinMode(BUTTON_L, INPUT_PULLUP);
  pinMode(BUTTON_R, INPUT_PULLUP);
  
  pinMode(LED_R,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_B,OUTPUT);


}

void loop() {
  // put your main code here, to run repeatedly:
//  digitalWrite(LED_G,HIGH);
//  digitalWrite(LED_R,LOW);
//  digitalWrite(LED_B,LOW);
//  motor_forward_raw(1);
//  delay(2000);
//  digitalWrite(LED_G,LOW);
//  digitalWrite(LED_R,HIGH);
//  digitalWrite(LED_B,HIGH);
//  motor_reverse_raw(1);
//  delay(2000);
//  

  if (digitalRead(BUTTON_L) == LOW && !LED_G_ON) {
    LED_G_ON = 1;
    digitalWrite(LED_G, HIGH);
  } else if (LED_G_ON) {
    LED_G_ON = 0;
    digitalWrite(LED_G, LOW);
  }

  if (digitalRead(BUTTON_R) == LOW && !LED_B_ON) {
    LED_B_ON = 1;
    digitalWrite(LED_B, HIGH);
  } else if (LED_B_ON) {
    LED_B_ON = 0;
    digitalWrite(LED_B, LOW);
  }
}

void motor_forward_raw(float pwm) { // pwm var range 0.0-1.0
  analogWrite(INPUT1, 0);
  analogWrite(INPUT2, 0);
  analogWrite(INPUT3, 0);
  analogWrite(INPUT4, 0);
  analogWrite(INPUT4, 255);
  analogWrite(INPUT2, constrain(pwm_float2int(pwm),0,18));  
}

void motor_reverse_raw(float pwm) {
  analogWrite(INPUT1, 0);
  analogWrite(INPUT2, 0);
  analogWrite(INPUT3, 0);
  analogWrite(INPUT4, 0);
  analogWrite(INPUT3, 255);
  analogWrite(INPUT1, constrain(pwm_float2int(pwm),0,18)); 
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











