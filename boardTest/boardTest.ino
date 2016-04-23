int INPUT1 = 5; //motor High-side Right-side
int INPUT2 = 6; //motor High-side Left-side
int INPUT3 = 9; //motor Low-side Left-side
int INPUT4 = 10; //motor Low-side Right-side
// 2 and 4 should be HIGH at the same time, 1 and 3 same time. NEVER OTHERWISE.
// 3 and 4 HIGH together for braking. 1 and 2 LOW at this time.

int LED_R = 11;
int LED_G = 12;
int LED_B = 13;


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
  
  pinMode(LED_R,OUTPUT);
  pinMode(LED_G,OUTPUT);
  pinMode(LED_B,OUTPUT);
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











