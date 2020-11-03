// This is a basic code that can be used to recive a keyboard input over the serial line

int value = 0; // set values you need to zero

void setup() { Serial.begin(9600);    // start serial at 9600 baud

}

void loop() {
 
  if(Serial.available()) {value = Serial.parseInt(); }   // Parse an Integer from Serial
   
 Serial.println(value); // Print Value from serial input
 
 delay(100); 
   

}

