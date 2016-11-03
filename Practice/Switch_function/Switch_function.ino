/* This is a practice Code.
   This code uses the switch - case function to coose whitch subrutien to start running.
   The Switch - case function recives a Serial input value to case value.
*/

int value = 0; // set values you need to zero

void setup() { Serial.begin(9600);    // start serial at 9600 baud

}

void loop() {
 
  if(Serial.available()) {value = Serial.parseInt(); }   // Parse an Integer from Serial
   
 Serial.print(" Recived Value: "), Serial.print(value); // Print Value from serial input
 
 
 switch(value) { case 1:  Case_1();
                 case 2:  Case_2();
                 default: Serial.println(" \tNope!"); }
 
 delay(100); 
   

}

// --------------------------------------------------------------------------------------------------------
// ===== Subrutiens =======================================================================================
// --------------------------------------------------------------------------------------------------------

void Case_1() { Serial.print(" \tYes ");}

void Case_2() { Serial.print(" \tMaybe ");}
