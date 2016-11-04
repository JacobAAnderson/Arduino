// Driving Code for SG1

// Libraries
#include <Servo.h>;    // Servo library
#include <Wire.h>      // Reference the I2C Library
#include <HMC5883L.h>  // Reference the HMC5883L Compass Library

// Defined values =======================================
// #define Bearing 270  // Defined bearing for the robot to follow. Not used if beacon is in play

// Name servos: =========================================
Servo A;     // Wheel A  Back left wheel
Servo B;     // Wheel B  Back right wheel
Servo C;     // Wheel C  Front left wheel
Servo D;     // Wheel D  Front right whell
Servo Lidar; // Servo that positions LIDAR

// Define LEDs ===========================================
int LEDg = 7;    // Green  LED attach to pin 7
int LEDy = 8;    // Yellow LED attache to pin 8
int LEDr = 9;    // Red LED attach to pin 9

// Variables ==============================================
  // Compass
HMC5883L compass;   // Store our compass as a variable.
int error = 0;      // Record any errors that may occur in the compass.
float HEADING;      // Heading in degrsse from compass
float DeltaC;      // Diferance between compass heading and the designated bearing

  // Beacon
int vectorSerial;      // Vector from Beacon
int Bearing;           // Bearing for the robot to follow
int InitialBeacon = 0; // Bearing reading at the begining of the course
int DeltaB;            // Beacon differential
 
  // Driving
int LSM = 77;   // Median left  side wheel speed. Left  wheels:  Forward < 90
int RSM = 103;  // Median right side wheel speed. Right wheels:  Forward > 90
int leftWheels;    // Varialbe used to control the right wheels' speed
int rightWheels;   // Variable used to control the left wheels' speed
int Radius;        // Turning radious: + Radius will turn left, - Radius will turn right

// == set up routien================================================================================================================================
// =================================================================================================================================================
void setup(){  Serial.begin(9600);

 // -- Servos --------------------------------------------------------  
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

 // -- LEDs, Set up LED pins as outputs --------------------------------
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
  
   // -- Compass --------------------------------------------------------------
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
   
// == Retrive Initial Bearing =========================================================================
  
  for (int index = 0; index < 10; index++) {
    GetBeacon();
    InitialBeacon = InitialBeacon + Bearing;}
    
    InitialBeacon = InitialBeacon / 10;

Serial.println("Set up done");

}

// == Main loop ====================================================================================================================================
// =================================================================================================================================================
void loop() {
  
  GetBeacon(); // Get bearing from beacon forthe robot to follow
  
  Compass();  // Get compass heading (direction the robot is facing)
  Serial.print("\t Heading: ");
  Serial.print(HEADING);
 
 // BeaconDifferencial();
 
  FindRadius(); // Use the bearing and compass heading to deterimin what direction the robot should move
  Serial.print("\t DeltaC: ");
  Serial.print(DeltaC);
  Serial.print("\t Radius: ");
  Serial.println(Radius);
  
  DriveForward();  // Go
  Serial.println("\t Driving ");
  delay(100);
  
  
  
  Serial.println("\t Main loop done, Repeat.");
}
// =========== Subrutiens ========================================================================================================
// ===============================================================================================================================

// ---- Driving routien ------------------------------------------------------
void DriveForward(){
  
  // Assign wheel speed values
  // Turning radious: + Radius will turn left, - Radius will turn right
  // LSM = 77   RSM = 103
  
    leftWheels  = (LSM + Radius); // Equation for the left  wheel speed
    rightWheels = (RSM + Radius); // Equation for the right wheel speed
  
  // Drive
  A. write(LSM);   // Wheel A  Back  left  wheel:  Forward < 90
  B. write(RSM);   // Wheel B  Back  right wheel:  Forward > 90
  C. write(leftWheels);   // Wheel C  Front left  wheel:  Forward < 90
  D. write(rightWheels);  // Wheel D  Front right whell:  Forward > 90
  
}

// ==== Navigation routiens ===================================================================================

// ---- Compass ----------------------------------------------------------------------------
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
  
// * Declination angle for Durango, CO.: 9.5 deg = 0.1658 rad
// * Declination angle for The Great Sandunes National Park: 8.5 deg = 0.14853 rad
  
  float declinationAngle = 0.1658; // * Declination Angle *
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

   delay(66); // Delay to prevent overloading the compass
}

// ---- Beacon routien ------------------------------------------------------------------------
void GetBeacon(){
  // Start a read from the Serial Interface
  // look for the next valid integer in the incoming serial stream:
  if (Serial.available() > 0) {vectorSerial = Serial.parseInt(); }
  else                        {vectorSerial = 202;} //Setting vectorSerial to a "stale" value to help the robot know the vector has not been updated.  
   
  if(vectorSerial == 90 || vectorSerial > 180) {Bearing = Bearing; }
  else if (vectorSerial < 90 )                 {Bearing = vectorSerial*2 + 180;}
  else                                         {Bearing = vectorSerial*2 - 180;}
  
  // Display bearing from radio beacon
  Serial.print("From Beacon Receiver over Serial: "); 
  Serial.print(vectorSerial);
  Serial.print("\t Bearing: ");
  Serial.println(Bearing);
  delay(2000);
}
// ---- Beacon differnce corection -----------------------------------------------------------
void BeaconDifferencial() { Serial.print("Calculateing Beacon differential");

  DeltaB = InitialBeacon - Bearing;
  Bearing = Bearing + DeltaB;
}

// ---- Detumine which way to go based on the compass and beacon data ------------------------
void FindRadius(){ // Use Compass heading to calculate a turning radious

  // Turning radious: + Radius will turn left, - Radius will turn right  
  if      (Bearing < 180 && (Bearing+180)<HEADING )      {DeltaC = Bearing - HEADING;}
  else if (180 < Bearing && HEADING < (Bearing - 180))   {DeltaC = Bearing - HEADING;}
  else                                                   {DeltaC = HEADING - Bearing;} // Corects for sign when Compass heading is in second quadrent
  
  Radius = map(DeltaC, -45, 45, -7, 7 );    // Map out turing radious.
  Radius = constrain(Radius, -7, 7);        // Constrain Turing radius to + or - 7 to prevent jack-knifing
}
  
  




