// This skrip recordes and averages the values for two IR sensors respectivly.

// Vectors to store IR readins
const int IRvec = 5;   // number of spaces in the vectors
int LeftIRvec[IRvec];  // Left IR sensor vector
int RightIRvec[IRvec]; // Right IR sensor vector
int IRindex = 0;        //Index number for the vector value being changed
int LeftIRtotal = 0;    // Sum of the Left IR vector
int RightIRtotal = 0;   // Sum of the right IR vector
int IRleft = 0;         // Averaged value of the left IR sensor reading
int IRright = 0;        // Averaged value of the right IR sensor readings


// Input pins
int LeftIRpin = A0;
int RightIRpin = A1;


void setup () { Serial.begin(9600);
  
  for (int zero = 0; zero < IRvec; zero++)  // initialize all vector values to 0
    LeftIRvec[zero] = 0,
    RightIRvec[zero] = 0;
}

void loop(){
  
  IRsensor(); // Runs the sensor averaging programing as a subsrutien
  delay(500);
}

void IRsensor(){  // Sensor averaging subrutien
   
   
  float RawIRleft;        // Variables used to display the raw IR readings
  float RawIRright;
   
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
      
// Print Sensor values
   RawIRleft  = analogRead(LeftIRpin);   // Raw left IR value
   RawIRright = analogRead(RightIRpin);  // Raw right IR value
   Serial.print("Raw left IR:  ");
   Serial.print(RawIRleft);
   Serial.print("\t Averaged left IR:  ");
   Serial.print(IRleft);
   Serial.print("\t\t Raw right IR:  ");
   Serial.print(RawIRright);
   Serial.print("\t Averaged right IR:  ");
   Serial.println(IRright);
   
}
