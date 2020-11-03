/* Practice Code. 
   This code runs a simple while loop while the evaluated expretion in a non 0 number
   The Evaluated number is recived over the serial line.
*/


int value = 2;

void setup() { Serial.begin(9600);}

void loop() {
  
  while(value) { Serial.println(" Keep going ");
                 if (Serial.available()) { value = Serial.parseInt();}
                 Serial.println(value);
                }
                
  Serial.println("End");
}
