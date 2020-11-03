int lightPin = A1;
int LEDpin = 13;

void setup() { Serial.begin(9600);
  pinMode( LEDpin, OUTPUT );

}

void loop() {
  
  int lightLevel = analogRead( lightPin );
  
  lightLevel = map(lightLevel, 0, 900, 0, 255);
  
  lightLevel = constrain(lightLevel, 0, 255);
  
  analogWrite( LEDpin, lightLevel);
  
  Serial.print("Sensor Level: "); Serial.println(lightLevel);

}
