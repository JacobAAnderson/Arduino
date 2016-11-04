/* This script calibrates two IR senssors, smooths their reading by averaging 
their readings and maps the output values over a arange from 0 to 150
*/

// Input pins
int LeftIRpin = A0;    // IR Sensors
int RightIRpin = A1;

// Output pins
int ledG = 5;  // Green LED

// variables:
int IRindex = 0;        //Index number for the vector value being changed
int LeftIRtotal = 0;    // Sum of the Left IR vector
int RightIRtotal = 0;   // Sum of the right IR vector
int RawIRleft = 0;      // the sensor values
int RawIRright = 0;
int IRleft = 0;         // Mapped IR values  
int IRright =0;
int LeftIRmin = 1023;   // Calibration values
int LeftIRmax = 0;
int RightIRmin = 1023;
int RightIRmax = 0;

// Vectors to store IR readins
const int IRvec = 5;   // number of spaces in the vectors
int LeftIRvec[IRvec];  // Left IR sensor vector
int RightIRvec[IRvec]; // Right IR sensor vector


void setup() {Serial.begin(9600);
   
   pinMode(LeftIRpin, INPUT);  // Reading from Port IR sensor
   pinMode(RightIRpin, INPUT); // Read from starboard IR sensor
   pinMode(ledG, OUTPUT);      // Signal to green LED
   
   VectorSetup(); // Runs a loop that sets all IR values to 0 
   CalibrateIR(); // Run the IR calibration subrutien
}

void loop() {
  
  RawIRleft  = analogRead(LeftIRpin);
  RawIRright = analogRead(RightIRpin);
  
  Serial.print("Raw left IR data: ");
  Serial.print(RawIRleft);
  Serial.print("\tRaw right IR data: ");
  Serial.print(RawIRright);
  
  SmoothIR();  // Runs the IR sensor smoothing subrutien
  mapIR();     // Run the IR maping subrutien
  
  //delay(500);
}
//======================= Subrutiens ===============================================================

void CalibrateIR() { // IR calibration subrutien
  
  while (millis() < 5000) {
    digitalWrite(ledG, HIGH);
    
    RawIRleft  = analogRead(A0);
    RawIRright = analogRead(A1);
    
    if (RawIRleft  > LeftIRmax)  {LeftIRmax  = RawIRleft;}   // record the maximum sensor value
    if (RawIRright > RightIRmax) {RightIRmax = RawIRright;}
    
    if (RawIRleft  < LeftIRmin)  {LeftIRmin  = RawIRleft;}  // record the minimum sensor value
    if (RawIRright < RightIRmin) {RightIRmin = RawIRright;}
  }
  digitalWrite(ledG, LOW);
  Serial.println("  ");
  Serial.print("Max left IR value: ");    // Print results
  Serial.print(LeftIRmax);
  Serial.print("\tMin left IR value: ");
  Serial.print(LeftIRmin);
  
  Serial.print("\tMax right IR value: ");
  Serial.print(RightIRmax);
  Serial.print("\tMin right IR value: ");
  Serial.println(RightIRmin);
}

void VectorSetup (){ // initialize all vector values to 0
  for (int zero = 0; zero < IRvec; zero++)  
    LeftIRvec[zero] = 0,
    RightIRvec[zero] = 0;
}


void SmoothIR(){  // Sensor averaging subrutien
      
// Aveage left IR sensor readings
   LeftIRtotal = LeftIRtotal - LeftIRvec[IRindex];      // Subtract the last reading from the vector
   LeftIRvec[IRindex] = analogRead(LeftIRpin);          // Read from the sensors
   LeftIRtotal = LeftIRtotal + LeftIRvec[IRindex];      // Add the reading from to the total.
   IRleft = LeftIRtotal / IRvec;                        // Calculate average sensor value
   
// Averager right IR sensor readings
   RightIRtotal = RightIRtotal - RightIRvec[IRindex];   // Subtract the last reading from the vector
   RightIRvec[IRindex] = analogRead(RightIRpin);        // Read from the sensors
   RightIRtotal = RightIRtotal + RightIRvec[IRindex];   // Add the reading from to the total.
   IRright = RightIRtotal / IRvec;                      // Calculate average sensor value
   
   IRindex = IRindex + 1;               // Advance to the next position in the array
   if (IRindex >= IRvec) IRindex = 0;   // Wrap around to the begining o fthe aray once your at the end

  Serial.print("\tSmoothed left IR: ");
  Serial.print(IRleft);
  Serial.print("\tSmoothed right IR: ");
  Serial.print(IRright);    
}

void mapIR() {  // Subtutien that maps the IR value on a range from 0 to 150
  // read the sensor:
 
  // apply the calibration to the sensor reading
  IRleft  = map(IRleft,  LeftIRmin,  LeftIRmax,  0, 150);
  IRright = map(IRright, RightIRmin, RightIRmax, 0, 150);

  // in case the sensor value is outside the range seen during calibration
  IRleft  = constrain(IRleft,  0, 150);
  IRright = constrain(IRright, 0, 150);

  Serial.print("\tMapped left IR value: ");
  Serial.print(IRleft);
  Serial.print("\tMapped right IR value: ");
  Serial.println(IRright);
}
