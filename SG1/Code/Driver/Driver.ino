/* This code is a stepping stone to progrming a robot that can follow the signal from a radio beacon and avoid obstacles
   This code enablea a four wheeled vehical with continuous motor servos and a magnetic compass to drive
   in the direction of a predefined magnetic bearing.
   It uses a LiDAR and IR sensors for obstacle avoidance
   
   Sensor data is included at the end of this script
   
   Created March 27th, 2015
   Jacob Anderson
   For: Space Grant Robot 1 "SG1"
*/

// Libraries
#include <Servo.h>;    // Servo library
#include <I2C.h>       // I2C library
#include <Wire.h>      // Reference the I2C Library
#include <HMC5883L.h>  // Reference the HMC5883L Compass Library

// Defined values
#define Bearing (270)               // Direction of travel Change this number to go different directions
#define LIDARLite_ADDRESS   0x62    // Default I2C Address of LIDAR-Lite.
#define RegisterMeasure     0x00    // Register to write to initiate ranging.
#define MeasureValue        0x04    // Value to initiate ranging.
#define RegisterHighLowB    0x8f    // Register to get both High and Low bytes in 1 call.

// -- Declare Inputs and Outputs ------------------------------------------------------------------------------------
  // Servos
Servo A;     // Wheel A  Back left wheel
Servo B;     // Wheel B  Back right wheel
Servo C;     // Wheel C  Front left wheel
Servo D;     // Wheel D  Front right whell
Servo Lidar; // Servo that positions LIDAR

  // IR Sensors
int LeftIR   = A0;  // Left IR sensor assigned to pin A0
int CenterIR = A1;  // Center IR sensor assigned to pin A1
int RightIR  = A2;  // Right IR sensor assigned to pin A2

  // LEDs
int LEDg = 7;    // Green  LED attach to pin 7
int LEDy = 8;    // Yellow LED attache to pin 8
int LEDr = 9;    // Red LED attach to pin 9

// -- Variables -------------------------------------------------------------------------------------------------------
int BEARING;   // Adjusted direction of travle from navigation rutien
int OffSet;    // Adjusment based on LIDAR data

  // Compass
HMC5883L compass;   // Store our compass as a variable.
int error = 0;      // Record any errors that may occur in the compass.
float HEADING;      // Heading in degrsse from compass
float DeltaC;       // Turning differential

  // Driving
int LSM = 77;   // Median left  side wheel speed. Left  wheels:  Forward < 90
int RSM = 103;  // Median right side wheel speed. Right wheels:  Forward > 90
int leftWheels;    // Varialbe used to control the right wheels' speed
int rightWheels;   // Variable used to control the left wheels' speed
int Radius;        // Turning radious: + Radius will turn left, - Radius will turn right

  // LIDAR
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
                           
  // IR Sensors
int LeftIRave;    int LeftIRhigh;    int LeftIRlow;    int LeftIR_0;   // Average on flat ground 280
int CenterIRave;  int CenterIRhigh;  int CenterIRlow;  int CenterIR_0; // Average on flat ground 316
int RightIRave;   int RightIRhigh;   int RightIRlow;   int RightIR_0;  // Average on flat ground 278  
                           
// -- Vector variables -------------------------------------------------------------------------------------------------------------------------------------------
// ---------------------------------------------------------------------------------------------------------------------------------------------------------------
const int LIDARvec = 13;  // Number of elements in LIDARdistances vector
const int binNumber = 11;  // Number of elements in LIDARbin vector
const int IR_vec_number = 10;    // Number of elements in the vectors
int LeftIRtotal = 0;    // Sum of each IR Vector
int CenterIRtotal = 0;
int RightIRtotal = 0;
int index = 0;

// -- Vectors ---------------------------------------------------------------------------------------------------------
int LIDARdistances[LIDARvec];  // Vector to store LIDAR measurments
int LIDARbins[binNumber];      // Vector for LIDAR averaging bins
int LeftIRvec[IR_vec_number];    // Left IR vector
int CenterIRvec[IR_vec_number];  // Center IR vector
int RightIRvec[IR_vec_number];   // Right IR vector


// == Set up routien================================================================================================================================
//==================================================================================================================================================

void setup(){  Serial.begin(9600); Serial.print(""); // empty line

// --- Set up Inputs anf OutPuts ---------------------------------------------------------------------
  // Servos  
  A. attach(2);      // Attach Back  left  wheel to pin 2
  B. attach(3);      // Attach Back  right wheel to pin 3
  C. attach(4);      // Attach Front left  wheel to pin 4
  D. attach(5);      // Attach Front right wheel to pin 5
  Lidar. attach(6);  // Attach LIDAR Servo to pin 6
  
    // Stop servos: 
    A. write(90);   // Servo write from 0 to 180,
    B. write(90);   // 0  to  90 is counter clock wise, 90 to 180 is clockwise
    C. write(90);
    D. write(90);
    Lidar.write(85); // Center Lidar Servo, Lidar centers with an input of 85, 90 is a little off center.

  // IR Sensors
  pinMode(LeftIR,   INPUT);
  pinMode(CenterIR, INPUT);
  pinMode(RightIR,  INPUT);

  // LEDs, Set up LED pins as outputs
  pinMode(LEDg, OUTPUT); 
  pinMode(LEDy, OUTPUT); 
  pinMode(LEDr, OUTPUT); 
    
    // turn LEDs on to indicat that they are working
    digitalWrite(LEDg, HIGH);
    digitalWrite(LEDy, HIGH);
    digitalWrite(LEDr, HIGH);
  
      delay (500);
  
    digitalWrite(LEDg, LOW);
    digitalWrite(LEDy, LOW);
    digitalWrite(LEDr, LOW);
  
      delay(10);
// -- Serial comunication set up ---------------------------------------------------------------------------------------------------------
// The compass and LIDAR both comunicate via I2C serial comunication.
   // -- Compass ---------------------------------------------------------------------------------------------------------------------
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

  // --- Set up LIDAR ------------------------------------------------------------------------------------------------------------------ 
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails

  Lidar.attach(6); // Attache Lidar Servo to pin 6
  
  Lidar.write(85);  // Centers Lidar detector
  delay (10);
  
// -- Zero-out Vectors ----------------------------------------------------------------------------------------------------------------------  

  // === Zero-out LIDAR vecor ==========================================================
  for (int zero = 0; zero < LIDARvec; zero++) {LIDARdistances[zero] = 0;}
  PrintLIDARvector(); 
  
  // === Zero-out IR vectors
  for (int Zero = 0; Zero < IR_vec_number; Zero++) {
   LeftIRvec[Zero]   = 0;
   CenterIRvec[Zero] = 0;
   RightIRvec[Zero]  = 0; }
   
// -- Set up initial values for variables --------------------------------------------------------------------------   
    // IR Sensors
  LeftIR_0 = 280;      
  CenterIR_0 = 316;
  RightIR_0 = 278;
  
  LeftIRhigh   = LeftIR_0   - 50;  LeftIRlow   = LeftIR_0   + 50;  Serial.print("LeftIRhig: ");   Serial.print(LeftIRhigh);    Serial.print("\tLeftIRlow: ");   Serial.println(LeftIRlow);       
  CenterIRhigh = CenterIR_0 - 50;  CenterIRlow = CenterIR_0 + 50;  Serial.print("CenterIRhig: "); Serial.print(CenterIRhigh);  Serial.print("\tCenterIRlow: "); Serial.println(CenterIRlow);
  RightIRhigh  = RightIR_0  - 50;  RightIRlow  = RightIR_0  + 50;  Serial.print("RightIRhig: ");  Serial.print(RightIRhigh);   Serial.print("\tRightIRlow: ");  Serial.println(RightIRlow);
 
Serial.println("Set up done");
delay(10);
}

// == Main loop ==============================================================================================================================================================
// ===========================================================================================================================================================================

void loop() {
// --- Get intrumantation data ---------------------------------------------------------------------------------------------------------    
  // Retreve the Compass heading, LIDAR data and analysed values, IR sensor values.
  Compass();  // Get Compass Heading
  Serial.print("Heading: \t"); Serial.println(HEADING);
  delay(10);
  
  LIDARsweep(); PrintLIDARvector();     // Get Lidar Data and Print Distances
  LIDARanalysis(); LIDARbinAnalysis();  // Analyse LIDAR data
   Serial.print("Vector Min: "); Serial.print(LIDARmin); Serial.print("\t Place: "); Serial.print(place); Serial.print("\t Angle: "); Serial.println(LIDARangle);
   Serial.println(""); // Empty line
   Serial.print("Minimum bin value: "); Serial.print(LIDARbinMin); Serial.print("\t Bin#: "); Serial.print (MinBin); Serial.print("\t Angle: "); Serial.println(MinBinAngle);
   Serial.print("Maximum bin value: "); Serial.print(LIDARbinMax); Serial.print("\t Bin#: "); Serial.print (MaxBin); Serial.print("\t Angle: "); Serial.println(MaxBinAngle);
   Serial.println(""); // Blank Line
 
  delay(10);
  
  ReadIR(); // Get and Display IR data
   Serial.print("Left IR: "); Serial.print(LeftIRave); Serial.print("\tCenter IR: "); Serial.print(CenterIRave); Serial.print("\tRight IR: "); Serial.println(RightIRave);
   Serial.println(""); // Blank Line
   
  delay(10);
 
 // --- Navigate ------------------------------------------------------------------------------------------------------------------------------------------------------- 
  /*   Use the compass heading, LIDAR data and IR values to navigate.
  ------ Proces --------------------------------------------------------------------------------------------------------------------------------------------------------
         1: If the minimum LIDAR distance is under a certain value ( object is very close ) the robot should drive backwards.
         2: If the IR sensors indicate a obstical (hole or large bump) infront of the robot it should drive backwards.
             The robot should also turn away from these obstacles.
         3: If the IR sensors indicat not obstacles and the LIDAR min is more than the above value but 
             less than an intermediat amount ( objects at mid rang) amount, the robot should drive away from the detctd object.
         4: If the LIDAR and IR sensors indicate no obsatles, drive on a  compass bearing
  
                  Values:  "Bearing"                                  - Defined bearing 
                           "BEARING"                                  - Bearing calculated by robot
                           "Heading"                                  - Compass Heading
                           "LIDARmin" @ "LIDARangle"                  - Closest distance detected by lidar & Coresponding angle in degrees from the center of the robot. 
                                                                        + angles are to the left - angles are to the right
                           "LIDARbinMin" & "MinBinAngle"              - Closest 3 bin average from LIDAR data and coresponding angle
                           "LIDARbinMax" & "MaxBinAngle"              - Furthest 3 bin average
                           "LeftIRave" "LeftIRhigh" "LeftIRlow"       - Values assigneed to the left IR sensore
                           "CenterIRave" "CenterIRhigh" "CenterIRlow" - Values assigneed to Center IR sensor
                           "RightIRave" "RghtIRhigh" "RightIRlow"     - IR sensor readings
                           "Radius"                                   - Turning differential +value turns left, - values turn right
  
  */
// --- 1: If LIDARmin is less than 50, drive backwards --------------------------------------------------------------------------------------------------------  
  if      ( LIDARmin <50 ) {DriveBackwards();} 

// --- 2: If IR sensors detects an obstacle, drive straight backwards then turn away from the object ------------------------------------------------------  
  else if (LeftIRave < LeftIRhigh || LeftIRave> LeftIRlow)            {DriveBackwards(); Radius = -3; delay(100);} 
  
  else if (CenterIRave < CenterIRhigh || CenterIRave > CenterIRlow)   {DriveBackwards(); Radius =  0; delay(100);}
  
  else if (RightIRave < RightIRhigh || RightIRave > RightIRlow)       {DriveBackwards(); Radius =  3; delay(100);}
  
// --- 3: LIDAR detects object at midrange, drive around object ---------------------------------------------------------------------------------------------- 
              /*   if      (BEARING < 180 && (BEARING+180)<HEADING )      {DeltaC = BEARING - HEADING;}
                   else if (180 < BEARING && HEADING < (BEARING - 180))   {DeltaC = BEARING - HEADING;}
                   else                                                   {DeltaC = HEADING - BEARING;} // Corects for sign when Compass heading is in second quadrent
              */


else if (LIDARbinMax < 300) { OffSet = MinBinAngle + MaxBinAngle ;
                              OffSet = constrain(OffSet, -45, 45); 
                              BEARING = Bearing + OffSet; 
                              DriveForward();
                            }
  
// --- 4: No obstacles detected, follow bearing --------------------------------------------------------------------------------------------------------------
  else {BEARING = Bearing; DriveForward();}
   
 
  
}
// =========== Subrutiens ====================================================================================================================================
// ===========================================================================================================================================================
// ==== Instrumentation Subroutiens ======================================================================================================
// --------------------------------------------------------------------------------------------------------------------------------------  
  // ==== Compass =============================================================

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
 
   delay(66); // Buffer time for compass Serial comunication
}

  // ==== LIDAR Subroutiens ===============================================================================

void LIDARsweep(){ // LIDAR servo is far right at 0 and far left at 180
  
  // Serial.println("LIDAR is scanning...");
  int a =0;
  for ( int SweepAngle=25; SweepAngle < 155 ; SweepAngle+=10 )
  { Lidar. write(SweepAngle);
    delay(200);
    
    LIDAR();
    LIDARdistances[a] = DISTANCE;
    
    delay (100);
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
  
  if (distance > 0) {DISTANCE = distance;}
  else              {DISTANCE = 5000;}
}
  
// === Print Lidar Vector ==================================================================

void PrintLIDARvector(){  // Prints the Lidar vector
 
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
  if      ( place <= 5) { LIDARangle = (place-6)*10;}
  else if ( place ==  6) { LIDARangle = 0;}
  else                  { LIDARangle = -(6-place)*10;}
 
 // Display the calculated values
 /*  Serial.print("Vector Min: ");
   Serial.print(LIDARmin);
   Serial.print("\t Place: ");
   Serial.print(place);
   Serial.print("\t Angle: ");
   Serial.println(LIDARangle);
   Serial.println(""); // Empty line
*/
}
// ---- LIDAR bin  analysis -------------------------------------------------------------------------------
// Takes the readings in the LIDAR vector and averages 3 readings togeter into a bin.
// The bins over lap by two element and corespond to angles conterclock wise around the
// front of the robot starting far right.

void LIDARbinAnalysis(){
 
 // Calculate LIDAR bins
 for(int bin = 0; bin < 9; bin++){
 LIDARbins[bin]= (LIDARdistances[bin]+LIDARdistances[(bin+1)]+LIDARdistances[(bin+2)])/3;  
 // Serial.print ("Bin "); Serial.print (bin); Serial.print (": "); Serial.println (LIDARbins[bin]); 
 }
 
 LIDARbinMax = 0;        // Set LIDAR Max and min values to quatantied that will be exceeded by actual reading
 LIDARbinMin = 10000;
 
 // Find minimum bin value and corespondind bin number
 for(int bin = 0; bin<9; bin++) {
  if(LIDARbinMin > LIDARbins[bin]) {LIDARbinMin = LIDARbins[bin]; MinBin = bin;}
  if(LIDARbinMax < LIDARbins[bin]) {LIDARbinMax = LIDARbins[bin]; MaxBin = bin;}
 }
 
 // Assign anblges to the bin places    + Angles are left of center, - Angles are right of center
 if      (MinBin <=4)  {MinBinAngle = -(MinBin*10)-50;}
 else if (MinBin == 5) {MinBinAngle = 0;}
 else                  {MinBinAngle = 50 - MinBin*10;}
 
 if      (MaxBin <=4)  {MaxBinAngle = -(50-MaxBin*10);}
 else if (MaxBin == 5) {MaxBinAngle = 0;}
 else                  {MaxBinAngle = MaxBin*10-50;}

  // Dispaly values
 /* Serial.print("Minimum bin value: "); Serial.print(LIDARbinMin); Serial.print("\t Bin#: "); Serial.print (MinBin); Serial.print("\t Angle: "); Serial.println(MinBinAngle);
  Serial.print("Maximum bin value: "); Serial.print(LIDARbinMax); Serial.print("\t Bin#: "); Serial.print (MaxBin); Serial.print("\t Angle: "); Serial.println(MaxBinAngle);
  Serial.println(""); // Blank Line
  */
}
  // ==== IR Sensors - Subroutiens =====================================================================================
  // --- Read IR sensors ---------------------------------------------------------------------------------------
void ReadIR(){
  
    LeftIRtotal   = LeftIRtotal   - LeftIRvec[index];      // Subtract the last reading from the array
    CenterIRtotal = CenterIRtotal - CenterIRvec[index];
    RightIRtotal  = RightIRtotal  - RightIRvec[index];
    
   LeftIRvec[index] = analogRead(LeftIR);      // Read from the sensors
   CenterIRvec[index] = analogRead(CenterIR);
   RightIRvec[index] = analogRead(RightIR);
   
   LeftIRtotal   = LeftIRtotal   + LeftIRvec[index];      // Subtract the last reading from the array
   CenterIRtotal = CenterIRtotal + CenterIRvec[index];
   RightIRtotal  = RightIRtotal  + RightIRvec[index];
    
   index = index + 1;                       // Advance to the next position in the array
   
   if (index >= IR_vec_number) index = 0;  // Wrap around to the begining o fthe aray once your at the end
   
   LeftIRave   = LeftIRtotal   / IR_vec_number;        // Calculate average sensor values
   CenterIRave = CenterIRtotal / IR_vec_number;
   RightIRave  = RightIRtotal  / IR_vec_number;
   
   /*
   Serial.print("Left IR ave: ");         // Display averages
   Serial.print(LeftIRave);
   Serial.print("\t Center IR ave: ");
   Serial.print(CenterIRave);
   Serial.print("\t Right IR ave: ");
   Serial.println(RightIRave);
   */
}

// --- Display IR Vectors ----------------------------------------------------------------------------------------
void PrintIRvectors(){
 
  Serial.print("Left IR Vector: [ ");
  for (int i=0;i<(IR_vec_number-1);i++)
  { Serial.print(LeftIRvec[i]);
    Serial.print(", ");}
    Serial.print(LeftIRvec[(IR_vec_number-1)]);
    Serial.print(" ]");
    Serial.print("\t Left IR ave: ");         // Display averages
    Serial.println(LeftIRave);
    
    Serial.print("Center IR Vector: [ ");
  for (int i=0;i<(IR_vec_number-1);i++)
  { Serial.print(CenterIRvec[i]);
    Serial.print(", ");}
    Serial.print(CenterIRvec[(IR_vec_number-1)]);
    Serial.print(" ]");
    Serial.print("\t Center IR ave: ");
    Serial.println(CenterIRave);
    
    Serial.print("Right IR Vector: [ ");
  for (int i=0;i<(IR_vec_number-1);i++)
  { Serial.print(RightIRvec[i]);
    Serial.print(", ");}
    Serial.print(RightIRvec[(IR_vec_number-1)]);
    Serial.print(" ]");
    Serial.print("\t Right IR ave: ");
    Serial.println(RightIRave);
    
    Serial.println("");
}
  
// ===== Driving routien ===========================================================================================================================================
// -----------------------------------------------------------------------------------------------------------------------------------------------------------------
void FindRadius(){ // Use Compass heading to calculate a turning radious
  // Turning radious: + Radius will turn left, - Radius will turn right
    
  if      (BEARING < 180 && (BEARING+180)<HEADING )      {DeltaC = BEARING - HEADING;}
  else if (180 < BEARING && HEADING < (BEARING - 180))   {DeltaC = BEARING - HEADING;}
  else                                                   {DeltaC = HEADING - BEARING;} // Corects for sign when Compass heading is in second quadrent
  
  
  Radius = map(DeltaC, -45, 45, -10, 10 );    // Map out turing radious.
  Radius = constrain(Radius, -8, 8);        // Constrain Turing radius to + or - 7 to prevent jack-knifing
}
// ----------------------------------------------------------------------------------------------------------------
void DriveForward(){Serial.println("** Driving forward **");
  FindRadius();
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

// --------------------------------------------------------------------------------------------------------------
void DriveBackwards() { Serial.println("** Driving Backwards **");
 // Positive radius goes right
 // Need to be modified
 
    leftWheels  = (LSM + Radius);
    rightWheels = (RSM + Radius);
 
  B. write(leftWheels);   // Wheel A  Back  left  wheel:  Forward < 90
  A. write(rightWheels);  // Wheel B  Back  right wheel:  Forward > 90
  D. write(LSM);   // Wheel C  Front left  wheel:  Forward < 90
  C. write(RSM);  // Wheel D  Front right whell:  Forward > 90
}


// Sensor Info ///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

/*    
Average IR Values - robt on flat ground

  Out Side:

  Flat ground  : Concteet in shade  Concreet in sun    Dirt in shade   Grass in shade    No thing              Sun
  Center       :  326                  319                  312            322                5        Varys significantly
  Right        :  300                  303                  300            278                5        Varys significantly


LIDAR distance info:

  If the LIDAR does not get a retun then it will register a reading of 0
  The largest observed reading in 4400
  Does not work well for distances less than 10cm


Some Average Distance Readings
  Out Side:
    Distance    : Sunny Textured surface     Sunny Smooth Black surface      Smooth surface in the shade
    ------------:----------------------------------------------------------------------------------------
    12ft, 366cm :       345                    12ft - 360                     12ft - 361
     6ft, 183cm :       178                     6ft - 175                      6ft - 177
     4ft, 122cm :       120                     4ft - 114                      4ft - 122
     3ft,  91cm :        92                     3ft -  90                      3ft -  90
     2ft,  61cm :        60                     2ft -  57                      2ft -  60
     1ft,  30cm :        30                     1ft -  35                      1ft -  30
  
  In Side: 
    Distance    :  Flat surface        Same flat surface 
                                      at a 30 degree angle
    -----------------------------------------------------------------
    12ft, 366cm :        365                  330
    6ft, 183cm :        183                  185
    4ft, 122cm :        125                  125
    3ft,  91cm :         92                   98
    2ft,  61cm :         60                   63
    1ft,  30cm :         31                   32

*/
