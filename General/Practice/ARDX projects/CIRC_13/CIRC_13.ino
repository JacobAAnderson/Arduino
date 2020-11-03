/* Vibration sensor,
   Turns an LED on and off when sensor detects vibration
   ---------------------------------------------------------
   Pin out:
     Vibration sensor - A0
     LED              - 13
*/


const int LEDpin     = 13;
const int sensorPin  = A0;
const int threshold  =  1;

int sensorReading = 0;
int LEDstate = LOW;

//-------------------------------------------------------------------------------------
void setup() { Serial.begin(9600);
  
  pinMode( LEDpin, OUTPUT );

}


//-------------------------------------------------------------------------------------
void loop() {
  
  sensorReading = analogRead(sensorPin);
  
  if ( sensorReading >= threshold ) { LEDstate = !LEDstate; }
  
  digitalWrite(LEDpin, LEDstate);
  
  Serial.print("Sensor Reading: "); Serial.println(sensorReading);
  Serial.print("LED State: ");      Serial.println(LEDstate);
  Serial.println("");
  
  delay(100);

}
