

void setup() { Serial.begin(9600); Serial.println(""); }

void loop() {  Serial.println("---- Start calculation ----"); Serial.println("");
 
  int value1, value2;
  
  // Collect 1st value from serial input -------------------------------------------
   Serial.print("Enter 1st value"); 
   while(!Serial.available()) { }  
   
   value1 = Serial.parseInt(); // Parse an Integer from Serial
   Serial.print("\tValue 1: "), Serial.println(value1);
   
 // Collect 2nd value from serial input --------------------------------------------  
   Serial.print("Enter 2nd value"); 
   while(!Serial.available()) { }  
   
   value2 = Serial.parseInt(); // Parse an Integer from Serial
   Serial.print("\tValue 2: "), Serial.println(value2);
  
 // Calculate product -------------------------------------------------------------- 
   int out = Function( value1, value2);
  
  Serial.print("\t\tProduct: "), Serial.println(out);
  Serial.println("");
  delay(1000);
}
  
/* ----------------------------------------------------------------------------------
=================== Subrutiens ======================================================
-----------------------------------------------------------------------------------*/

  int Function(int type, int axis) {
    int value = type * axis;
    return value;

}
