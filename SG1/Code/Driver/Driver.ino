/* This code is a stepping stone to progrming a robot that can follow the signal from a radio beacon
   This code enablea a four wheeled vehical with continuous motor servos and a magnetic compass to drive
   in the direction of a predefined magnetic bearing.
   It also inclued LIDAR obstacle detection and avoidance
   Created March 27th, 2015
   Jacob Anderson
   For: Space Grant Robot 1 "SG1"
*/

// Libraries
#include <Servo.h>;    // Servo library
#include <Wire.h>      // Reference the I2C Library for the compass
#include <HMC5883L.h>  // Reference the HMC5883L Compass Library
#include <I2C.h>       // I2C library for LIDAR

// Defined values
#define    Bearing 0    // Bearing to be followed by robot

#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.


// Servos
Servo A;     // Wheel A  Back left wheel
Servo B;     // Wheel B  Back right wheel
Servo C;     // Wheel C  Front left wheel
Servo D;     // Wheel D  Front right whell
Servo Lidar; // Servo that positions LIDAR

// Define LEDs
int LEDg = 7;    // Green  LED attach to pin 7
int LEDy = 8;    // Yellow LED attache to pin 8
int LEDr = 9;    // Red LED attach to pin 9

// Variables
HMC5883L compass;   // Store our compass as a variable.
int error = 0;      // Record any errors that may occur in the compass.
float HEADING;      // Heading in degrsse from compass
float DeltaC;       // Variable that calculated the differance between the compass heading and the bearing
 
int LSM = 77;   // Median left  side wheel speed. Left  wheels:  Forward < 90
int RSM = 103;  // Median right side wheel speed. Right wheels:  Forward > 90
int leftWheels;    // Varialbe used to control the right wheels' speed
int rightWheels;   // Variable used to control the left wheels' speed
int Radius;        // Turning radious: + Radius will turn left, - Radius will turn right

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
const int LIDARvec = 13;  // Number of elements in LIDARdistances vector
const int binNumber = 11;  // Number of elements in LIDARbin vector

// Vectors
int LIDARdistances[LIDARvec];  // Vector to store LIDAR measurments
int LIDARbins[binNumber];      // Vector for LIDAR averaging bins


// == set up routien================================================================================================================================
// =================================================================================================================================================
void setup(){  Serial.begin(9600);  Serial.println(""); // empty line
  Serial.println("Setting up");

// -- Servos -----------------------------------------------------------------------  
  A. attach(2);      // Attach Back  left  wheel to pin 2
  B. attach(3);      // Attach Back  right wheel to pin 3
  C. attach(4);      // Attach Front left  wheel to pin 4
  D. attach(5);      // Attach Front right wheel to pin 5
  Lidar. attach(6);  // Attach LIDAR Servo to pin 6
  
    // Stop driving motors: 
    // Servo write from 0 to 180, 
    // 0  to  90 is counter clock wise, 90 to 180 is clockwise
  A. write(90);   
  B. write(90);
  C. write(90);
  D. write(90);
  Lidar.write(85); // Center Lidar Servo, Lidar centers with an input of 85, 90 is a little off center.

// -- LEDs, Set up LED pins as outputs -----------------------------------------------
  pinMode(LEDg, OUTPUT); 
  pinMode(LEDy, OUTPUT); 
  pinMode(LEDr, OUTPUT); 
    
  // turn LEDs on to indicat that they are working
  digitalWrite(LEDg, HIGH);
  digitalWrite(LEDy, HIGH);
  digitalWrite(LEDr, HIGH);
  
  delay (5000);
  
  digitalWrite(LEDg, LOW);
  digitalWrite(LEDy, LOW);
  digitalWrite(LEDr, LOW);
  
  delay(1000);
  
// -- Compass ------------------------------------------------------------------------
  Serial.println("Starting the I2C interface.");
  Wire.begin(); // Start the I2C interface.

  Serial.println("Constructing new HMC5883L");
  compass = HMC5883L(); // Construct a new HMC5883 compass.
    
  Serial.println("Setting scale to +/- 1.3 Ga");
  error = compass.SetScale(1.3); // Set the scale of the compass.
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));
  
  Serial.println("Setting measurement mode to continous.");
  error = compass.SetMeasurementMode(Measurement_Continuous); // Set the measurement mode to Continuous
  if(error != 0) // If there is an error, print it out.
    Serial.println(compass.GetErrorText(error));

// -- LIDAR ----------------------------------------------------------------------------
I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails

  Lidar.attach(6); // Attache Lidar Servo to pin 6
  
  Lidar.write(85);  // Centers Lidar detector
  delay (10);
  
Serial.println(""); // Blank Line

// -- Zero-out Vectors -----------------------------------------------------------------
 for (int zero = 0; zero < LIDARvec; zero++) {LIDARdistances[zero] = 0;} // LIDAR Vector
     PrintLIDARvector(); 

Serial.println(""); // Blank LIne     
Serial.println("Set up done");
}

// == Main loop ====================================================================================================================================
// =================================================================================================================================================
void loop() { Serial.println("");
// --- Collect Data from instruments ----------------------------------------------------   
  // - LIDAR data -
  LIDARsweep();       // Populate LIDAR vector
  LIDARanalysis();    // Find Minimum value of LIDAR vector
  LIDARbinAnalysis(); // 3 bin averaging of LIDAR data, find Max and Min bin values and coresponding angles
  delay(1);
  
  // Get data from IR sensors
  
  Compass(); // Get robots heading from the compass
   
// --- Navigate ------------------------------------------------------------------------  
  
// --- Drive ---------------------------------------------------------------------------  
  FindRadius();
  DriveForward();
  delay(100);
  
}
// =========== Subrutiens ========================================================================================================
// ===============================================================================================================================
// ==== Colect data subrutiens ===========================================================================================================================
// ---- Compass -------------------------------------------------------------------------------------------
void Compass(){
  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis();  //why no orange?***********
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  
  // Values are accessed like so:
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  
  // Once you have your heading, you must then add your 'Declination Angle', which is the 'Error' of the magnetic field in your location.
  // Find yours here: http://www.magnetic-declination.com/
  // Mine is: 2� 37' W, which is 2.617 Degrees, or (which we need) 0.0456752665 radians, I will use 0.0457
  // If you cannot find your Declination, comment out these two lines, your compass will be slightly off.
  float declinationAngle = 0.0108;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 
  HEADING = headingDegrees;
  
  Serial.print("Compass Heading: "); Serial.print(HEADING);
  delay(66);
}
// ==== LIDAR Subroutiens =================================================================================
// ---- Get distance from LIDAR -----------------------------------------------------------
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
  
  if (distance > 0) { DISTANCE = distance;}
  else              { DISTANCE = 5000;}
}
  
// ---- Populate LIDAR Vector------------------------------------------------------
void LIDARsweep(){ // LIDAR servo is far right at 0 and far left at 180
  
  Serial.println("LIDAR is scanning...");
  int a =0;
  for ( int SweepAngle=25; SweepAngle < 155 ; SweepAngle+=10 )
  { Lidar. write(SweepAngle);
    delay(300);
    
    LIDAR();
    LIDARdistances[a] = DISTANCE;
    
    delay (100);
    a=a+1; 
  }
  Lidar.write(85); 
  PrintLIDARvector();  // Display LIDAR vector
}

// ---- Analyse Lidar data for minimum value and its angle ----------------------------------------
void LIDARanalysis(){
  LIDARmin = 1000;
  // Find the minimum Value in vector
  for(int b =0; b < LIDARvec; b++){  
    if( LIDARdistances[b] < LIDARmin){ LIDARmin = LIDARdistances[b]; place = b;}
   } 
  
  // Calculate angle in degrees from center. + angle is to the left, - angle is to the right
  if      ( place <= 5) { LIDARangle = (place-6)*10;}
  else if ( place ==  6) { LIDARangle = 0;}
  else                  { LIDARangle = -(6-place)*10;}
 
 // Display the calculated values
   Serial.print(" Vector Min: "); Serial.print(LIDARmin); Serial.print("\t Place: "); Serial.print(place); Serial.print("\t Angle: "); Serial.println(LIDARangle);
   Serial.println(""); // Empty line

}
// ---- LIDAR bin  analysis -------------------------------------------------------------------------------
// Takes the readings in the LIDAR vector and averages 3 readings togeter into a bin.
// The bins over lap by two element and corespond to angles conterclock wise around the
// front of the robot starting far right.

void LIDARbinAnalysis(){
 
 // Calculate LIDAR bins
 for(int bin = 0; bin < binNumber; bin++){
 LIDARbins[bin]= (LIDARdistances[bin]+LIDARdistances[(bin+1)]+LIDARdistances[(bin+2)])/3;  
  Serial.print (" Bin "); Serial.print (bin); Serial.print (": "); Serial.println (LIDARbins[bin]); 
 }
 Serial.println(""); // Blank Line

 LIDARbinMax = 0;        // Set LIDAR Max and min values to quatantied that will be exceeded by actual reading
 LIDARbinMin = 10000;
 
 // Find minimum bin value and corespondind bin number
 for(int bin = 0; bin<binNumber; bin++) {
  if(LIDARbinMin > LIDARbins[bin]) {LIDARbinMin = LIDARbins[bin]; MinBin = bin;}
  if(LIDARbinMax < LIDARbins[bin]) {LIDARbinMax = LIDARbins[bin]; MaxBin = bin;}
 }
 
 // Assign anblges to the bin places    + Angles are left of center, - Angles are right of center
 if      (MinBin <=4)  {MinBinAngle = -(50-MinBin*10);}
 else if (MinBin == 5) {MinBinAngle = 0;}
 else                  {MinBinAngle = MinBin*10-50;}
 
 if      (MaxBin <=4)  {MaxBinAngle = -(50-MaxBin*10);}
 else if (MaxBin == 5) {MaxBinAngle = 0;}
 else                  {MaxBinAngle = MaxBin*10-50;}

  // Dispaly values
  Serial.print(" Minimum bin value: "); Serial.print(LIDARbinMin); Serial.print("\t Bin#: "); Serial.print (MinBin); Serial.print("\t Angle: "); Serial.println(MinBinAngle);
  Serial.print(" Maximum bin value: "); Serial.print(LIDARbinMax); Serial.print("\t Bin#: "); Serial.print (MaxBin); Serial.print("\t Angle: "); Serial.println(MaxBinAngle);
  Serial.println(""); // Blank Line
}

// ---- Print Lidar Vector -------------------------------------------------------------------------

void PrintLIDARvector(){
 
  Serial.print(" LIDAR Vector: [ ");
  for (int i=0;i<(LIDARvec-1);i++) { Serial.print(LIDARdistances[i]); Serial.print(", ");}
    Serial.print(LIDARdistances[(LIDARvec-1)]);
    Serial.println(" ]");
}
// ==== Navigation subroutiens ===================================================================================================

// ===== Driving routien =========================================================================================================
// ---- Find turning radius ------------------------------------------------------------
void FindRadius() // Use Compass heading to calculate a turning radious
{
  // Turning radious: + Radius will turn left, - Radius will turn right
    
  if      (Bearing < 180 && (Bearing+180)<HEADING )      {DeltaC = Bearing - HEADING;}
  else if (180 < Bearing && HEADING < (Bearing - 180))   {DeltaC = Bearing - HEADING;}
  else                                                   {DeltaC = HEADING - Bearing;} // Corects for sign when Compass heading is in second quadrent
  
  
  Radius = map(DeltaC, -45, 45, -7, 7 );    // Map out turing radious.
  Radius = constrain(Radius, -7, 7);        // Constrain Turing radius to + or - 7 to prevent jack-knifing
  
  Serial.print("\tDeltaC: "); Serial.print(DeltaC); Serial.print("\t Radius"); Serial.println(Radius);
}

// ---- Drive Forward ------------------------------------------------------------------
void DriveForward(){
  
  Serial.println(""); // Blank line
  Serial.println("Driving!!");
  // Assign wheel speed values
  // Turning radious: + Radius will turn left, - Radius will turn right
  // LSM = 77   RSM = 103
  
    leftWheels  = (LSM + Radius);
    rightWheels = (RSM + Radius);
  
  // Drive
  A. write(LSM);   // Wheel A  Back  left  wheel:  Forward < 90
  B. write(RSM);   // Wheel B  Back  right wheel:  Forward > 90
  C. write(leftWheels);   // Wheel C  Front left  wheel:  Forward < 90
  D. write(rightWheels);  // Wheel D  Front right whell:  Forward > 90
  
}



