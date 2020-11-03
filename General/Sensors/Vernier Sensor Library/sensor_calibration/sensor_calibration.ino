// Vernier Sensor Calibration Script

# define pin A0

#define VOLTAGE(V) (V * 5.0 /1025.0)  // Function that converts an analog reading to its coresponding voltage


void setup() { 
  
  Serial.begin(9600);
 
  pinMode(pin, INPUT);
  pinMode(7, OUTPUT);
  digitalWrite(7, HIGH);

 // analogReference(EXTERNAL);
  

}

void loop() {

  Serial.print("Voltage: ");
  Serial.print(VOLTAGE(analogRead(pin)));
  Serial.print("\t");

/*  Calibrate Salinity Sensor -------------------------------------- */
  const float Sal_slope = 16.3;
  const float Sal_int = 0.0;
 
  float Salinity = VOLTAGE(analogRead(pin)) * Sal_slope + Sal_int;

  Serial.print("Salinity: ");
  Serial.print(Salinity);
  Serial.println(" [ppt]");

 
/*   Calibrate pH sensor -------------------------------------------
  const float pH_slope = -4.237;
  const float pH_int = 14.083;

  float pH =  VOLTAGE(analogRead(pin)) * pH_slope + pH_int;

  Serial.print("pH: ");
  Serial.println(pH);
*/

 delay(1000);

}
