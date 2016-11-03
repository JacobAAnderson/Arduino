

void setup(){Serial.begin(9600); 
  Serial.println(" ----- Starting ----- ");
  Serial.println("");

  pinMode(9,OUTPUT);
  digitalWrite(9,LOW);
  delay(500);
}

void loop() {
  
  Serial.println("Ramping Up");
  
  for(int i=0; i<256; i++){ digitalWrite(9,i); Serial.println(i); delay(100);}
  
  Serial.println("Max Speed");
  digitalWrite(9,HIGH);
  delay(100);
  
  Serial.println("Slowing Down");
  
  for(int i=255; i>=0; i--) { digitalWrite(9,i); Serial.println(i); delay(100);}
  
  digitalWrite(9,LOW);
  delay(100);
  
  Serial.println("Off");
  Serial.println("");
  
}
  

  
