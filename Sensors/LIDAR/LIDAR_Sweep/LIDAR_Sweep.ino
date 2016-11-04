// Libraries
#include <Servo.h>; // Servo library
#include <I2C.h>    // I2C library

// Defined Values
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

// Name Servos:
Servo Lidar;

// Define Variables;
int DISTANCE;           // Raw distance recived from LIDAR
int LIDARbinMax;        // Max value of LIDAR averaging bins
int MaxBin = 0;         // Bin number of Max LIDAR averaging bin
int MaxBinAngle;        // Angle coresponding to Max average bin
int LIDARbinMin;        // Min value of LIDAR averaging bins
int MinBin = 0;         // Bin number of Min LIDAR averaging bin
int MinBinAngle;        // Angle coresponding to Min average bin
int place;              // Location of the minimum value elimant in the vectoer. Eliment 1 of the vector is 0.
int LIDARangle = 0;     // Angle of the minimum value from center
int LIDARmin;           // Minimum value of the vector. Set very high initialy so that logic statments will reduce
                           //the value to the minimum value of the vector
//  Vector variables 
const int LIDARvec = 11;  // Number of elements in LIDARdistances vector
const int binNumber = 9;  // Number of elements in LIDARbin vector

// Vectors
int LIDARdistances[LIDARvec];  // Vector to store LIDAR measurments
int LIDARbins[binNumber];      // Vector for LIDAR averaging bins

// ==== Set up routien =====================================================================================================================
// =========================================================================================================================================
void setup(){Serial.begin(9600); //Opens serial connection at 9600bps.   
  Serial.print(""); // empty line
  // === Set up LIDAR =================================================================
    
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails

  Lidar.attach(6); // Attache Lidar Servo to pin 6
  
  Lidar.write(85);  // Centers Lidar detector
  delay (10);
  
  // === Set up LIDAR vecor ==========================================================
  for (int zero = 0; zero < LIDARvec; zero++) {LIDARdistances[zero] = 0;}
  
  PrintLIDARvector(); 
  Serial.println("Set up done");
  
}
// ==== Main Loop ===========================================================================================================================
// ===========================================================================================================================================
void loop(){ 
    
  LIDARsweep();
    
  // Print Distance
  PrintLIDARvector();
  LIDARanalysis();
  LIDARbinAnalysis();
 delay (1000);
}



// ===== Subrutiens ==========================================================================================================================
// ===========================================================================================================================================

void LIDARsweep(){ // LIDAR servo is far right at 0 and far left at 180
  
  Serial.println("LIDAR is scanning...");
  int a =0;
  for ( int SweepAngle=35; SweepAngle < 145 ; SweepAngle+=10 )
  { Lidar. write(SweepAngle);
    delay(100);
    
    LIDAR();
    LIDARdistances[a] = DISTANCE;
    
    delay (400);
    a=a+1; 
  }
  Lidar.write(85);
   
}

// ==== Get distance from LIDAR =======================================================
void LIDAR() {
  // Write 0x04 to register 0x00
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
  }

  byte distanceArray[2]; // array to store distance bytes from read function
  
  // Read 2byte distance from register 0x8f
  nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    delay(1); // Wait 1 ms to prevent overpolling
  }
  int distance = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  DISTANCE = distance;
}
  
// === Print Lidar Vector ==================================================================

void PrintLIDARvector(){
 
  Serial.print(" LIDAR Vector: [ ");
  for (int i=0;i<(LIDARvec-1);i++)
  { Serial.print(LIDARdistances[i]);
    Serial.print(", ");}
    Serial.print(LIDARdistances[(LIDARvec-1)]);
    Serial.println(" ]");
}

// ==== Analyse Lidar data for minimum value and its angle ==============================================
void LIDARanalysis(){
  LIDARmin = 1000;
  // Find the minimum Value in vector
  for(int b =0; b < LIDARvec; b++){  
    if( LIDARdistances[b] < LIDARmin){ LIDARmin = LIDARdistances[b]; place = b;}
   } 
  
  // Calculate angle in degrees from center. + angle is to the left, - angle is to the right
  if      ( place <= 4) { LIDARangle = (place-5)*10;}
  else if ( place ==  5) { LIDARangle = 0;}
  else                  { LIDARangle = -(5-place)*10;}
 
 // Display the calculated values
   Serial.print("Vector Min: ");
   Serial.print(LIDARmin);
   Serial.print("\t Place: ");
   Serial.print(place);
   Serial.print("\t Angle: ");
   Serial.println(LIDARangle);
   Serial.println(""); // Empty line

}
// ---- LIDAR bin  analysis -------------------------------------------------------------------------------
// Takes the readings in the LIDAR vector and averages 3 readings togeter into a bin.
// The bins over lap by two element and corespond to angles conterclock wise around the
// front of the robot starting far right.

void LIDARbinAnalysis(){
 
 // Calculate LIDAR bins
 for(int bin = 0; bin < 9; bin++){
 LIDARbins[bin]= (LIDARdistances[bin]+LIDARdistances[(bin+1)]+LIDARdistances[(bin+2)])/3;  
  Serial.print ("Bin ");
  Serial.print (bin);
  Serial.print (": ");
  Serial.println (LIDARbins[bin]); 
 }
 

 LIDARbinMax = 0;        // Set LIDAR Max and min values to quatantied that will be exceeded by actual reading
 LIDARbinMin = 10000;
 
 // Find minimum bin value and corespondind bin number
 for(int bin = 0; bin<9; bin++) {
  if(LIDARbinMin > LIDARbins[bin]) {LIDARbinMin = LIDARbins[bin]; MinBin = bin;}
  if(LIDARbinMax < LIDARbins[bin]) {LIDARbinMax = LIDARbins[bin]; MaxBin = bin;}
 }
 
 // Assign anblges to the bin places    + Angles are left of center, - Angles are right of center
 if      (MinBin <=3)  {MinBinAngle = -(50-MinBin*10);}
 else if (MinBin == 4) {MinBinAngle = 0;}
 else                  {MinBinAngle = MinBin*10-40;}
 
 if      (MaxBin <=3)  {MaxBinAngle = -(50-MaxBin*10);}
 else if (MaxBin == 4) {MaxBinAngle = 0;}
 else                  {MaxBinAngle = MaxBin*10-40;}

  // Dispaly values
  Serial.print("Minimum bin value: "); Serial.print(LIDARbinMin); Serial.print("\t Bin#: "); Serial.print (MinBin); Serial.print("\t Angle: "); Serial.println(MinBinAngle);
  Serial.print("Maximum bin value: "); Serial.print(LIDARbinMax); Serial.print("\t Bin#: "); Serial.print (MaxBin); Serial.print("\t Angle: "); Serial.println(MaxBinAngle);
  Serial.println(""); // Blank Line
}



