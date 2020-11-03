// This program is to ballance wheel speed

  // Motor shield configuration
  int pwm_a = 3;    // Right Wheel speed
  int pwm_b = 11;   // Left wheel speed
  int dir_a = 12;   // Right wheel direction
  int dir_b = 13;   // Left wheel direction
  
 // Variables:
  float wheelA;  // Right wheel
  float wheelB;  // Left wheel

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);  // communication rate
  
  // Set control pins for the wheels
  pinMode(pwm_a, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);
  
  // Set both moters to 0 - stop
  analogWrite(pwm_a, 0);   
  analogWrite(pwm_b, 0);
  
  }

void loop() {
  
  
  // Wheel speed equations
  wheelA = 80;   
  wheelB = 100;

  
  digitalWrite(dir_a, HIGH);
  digitalWrite(dir_b, HIGH);
  analogWrite(pwm_a, wheelA);   
  analogWrite(pwm_b, wheelB); }
  



