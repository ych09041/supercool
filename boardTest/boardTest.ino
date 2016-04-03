int INPUT1 = 5; //motor High-side Right-side
int INPUT2 = 6; //motor High-side Left-side
int INPUT3 = 9; //motor Low-side Left-side
int INPUT4 = 10; //motor Low-side Right-side

int LED_R = 11;
int LED_G = 12;
int LED_B = 13;


void setup() {
  // put your setup code here, to run once:
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
  delay(1);
  digitalWrite(LED_R,HIGH);
  delay(1);
  digitalWrite(LED_B,HIGH);
  delay(1);
  digitalWrite(LED_G,LOW);
  delay(1);
  digitalWrite(LED_R,LOW);
  delay(1);
  digitalWrite(LED_B,LOW);
  delay(1);
  
  
  
}
