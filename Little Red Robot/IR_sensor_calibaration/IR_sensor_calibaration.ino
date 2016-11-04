// This script calibrates two IR senssors and maps the output values over a arange from 0 to 150 

// Input pins
int LeftIRpin = A0;    
int RightIRpin = A1;

// Output pins
int GreenLed = 5;

// variables:
int RawIRleft = 0;    // the sensor values
int RawIRright = 0;
int IRleft = 0;       // Mapped IR values  
int IRright =0;
int LeftIRmin = 1023;        // Calibration values
int LeftIRmax = 0;
int RightIRmin = 1023;
int RightIRmax = 0;


void setup() {Serial.begin(9600);
   calibrateIR(); // Run the IR calibration subrutien
}

void loop() {
  mapIR(); // Run the IR maping subrutien
  
  delay(500);
}
//======================= Subrutiens ===================================

void calibrateIR() { // IR calibration subrutien
  while (millis() < 5000) {
    digitalWrite(GreenLed, HIGH);
    
    RawIRleft  = analogRead(LeftIRpin);  // Read IR sensors
    RawIRright = analogRead(RightIRpin);

    if (RawIRleft  > LeftIRmax)  {LeftIRmax  = RawIRleft;}   // record the maximum sensor value
    if (RawIRright > RightIRmax) {RightIRmax = RawIRright;}
    
    if (RawIRleft  < LeftIRmin)  {LeftIRmin  = RawIRleft;}  // record the minimum sensor value
    if (RawIRright < RightIRmin) {RightIRmin = RawIRright;}
  }
  digitalWrite(GreenLed, LOW);
  Serial.println("  ");
  Serial.print("Max left IR value: \t");    // Print results
  Serial.print(LeftIRmax);
  Serial.print("\t Min left IR value: \t");
  Serial.print(LeftIRmin);
  
  Serial.print("\t Max right IR value: \t");
  Serial.print(RightIRmax);
  Serial.print("\t Min right IR value: \t");
  Serial.println(RightIRmin);
}

void mapIR() {  // Subtutien that maps the IR value on a range from 0 to 150
  // read the sensor:
  
  
  RawIRleft  = analogRead(LeftIRpin);
  RawIRright = analogRead(RightIRpin);
  
  // apply the calibration to the sensor reading
  IRleft  = map(RawIRleft,  LeftIRmin,  LeftIRmax,  0, 150);
  IRright = map(RawIRright, RightIRmin, RightIRmax, 0, 150);

  // in case the sensor value is outside the range seen during calibration
  IRleft  = constrain(IRleft,  0, 150);
  IRright = constrain(IRright, 0, 150);

  // fade the LED using the calibrated value:
  
  Serial.print("Raw left IR data: \t");
  Serial.print(RawIRleft);
  Serial.print(" \t Mapped left IR value: ");
  Serial.print(IRleft);
  Serial.print("\t Raw right IR data: \t");
  Serial.print(RawIRright);
  Serial.print(" \t Mapped right IR value: ");
  Serial.println(IRright);
}
