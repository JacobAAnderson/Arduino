

void setup() {
  
pinMode(5, OUTPUT);  // Green LED
pinMode(6, OUTPUT);  // Yellow LED
pinMode(7, OUTPUT);  // Red LED
pinMode(8, INPUT);   // Left Bumper
pinMode(9, INPUT);   // Right Bumper

attachInterrupt(digitalPinToInterrupt(2), BUMP, LOW); // Bumper interupt
  
}

void loop() {
 
  digitalWrite(5, HIGH);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);  
  
}

void BUMP() {
 
  digitalWrite(5,LOW);
  digitalWrite(7,HIGH);
  
  if (digitalRead(8) == 1 ) {digitalWrite(6, HIGH);}
  
}
