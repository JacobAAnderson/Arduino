/* This code is a stepping stone to progrming a robot that can follow the signal from a radio beacon
   This code enablea a four wheeled vehical with continuous motor servos and a magnetic compass to drive
   in the direction of a predefined magnetic bearing.
   Created March 27th, 2015
   Jacob Anderson
   For: Space Grant Robot 1 "SG1"
*/

// Libraries
#include <Servo.h>;    // Servo library
#include <Wire.h>      // Reference the I2C Library
#include <HMC5883L.h>  // Reference the HMC5883L Compass Library

// Defined values
#define Bearing 270 // Change this number to go different directions

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
float DeltaC;

int LSM = 77;   // Median left  side wheel speed. Left  wheels:  Forward < 90
int RSM = 103;  // Median right side wheel speed. Right wheels:  Forward > 90
int leftWheels;    // Varialbe used to control the right wheels' speed
int rightWheels;   // Variable used to control the left wheels' speed
int Radius;        // Turning radious: + Radius will turn left, - Radius will turn right

// == set up routien================================================================================================================================
void setup(){  Serial.begin(9600);

  // == Servos ==============================================  
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

  // == LEDs, Set up LED pins as outputs ====================
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
  
   // == Compass =============================================
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
  
Serial.println("Set up done");
// == Set up initial values for variables ===================
 }

// == Main loop ====================================================================================================================================

void loop() {
    
  Compass();
  Serial.print("Heading: \t");
  Serial.print(HEADING);
  Serial.print("\t DeltaC: \t");
  Serial.print(DeltaC);
  
  FindRadius();
  Serial.print("\t Radius");
  Serial.println(Radius);
  DriveForward();
  delay(100);
  
}
// =========== Subrutiens ========================================================================================================
// ===============================================================================================================================
// ===== Driving routien ================================================
void DriveForward(){
  
  Serial.println("Driving");
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

  // Output the data via the serial port.
  // Output(raw, scaled, heading, headingDegrees);

  // Normally we would delay the application by 66ms to allow the loop
  // to run at 15Hz (default bandwidth for the HMC5883L).
  // However since we have a long serial out (104ms at 9600) we will let
  // it run at its natural speed.
   delay(66);
}

void Output(MagnetometerRaw raw, MagnetometerScaled scaled, float heading, float headingDegrees)
{
   Serial.print("Raw:\t");
   Serial.print(raw.XAxis);
   Serial.print("   ");   
   Serial.print(raw.YAxis);
   Serial.print("   ");   
   Serial.print(raw.ZAxis);
   Serial.print("   \tScaled:\t");
   
   Serial.print(scaled.XAxis);
   Serial.print("   ");   
   Serial.print(scaled.YAxis);
   Serial.print("   ");   
   Serial.print(scaled.ZAxis);

   Serial.print("   \tHeading:\t");
   Serial.print(heading);
   Serial.print(" Radians   \t");
   Serial.print(headingDegrees);
   Serial.println(" Degrees   \t");
   delay(1000);
}

// ==== Navigation routiens ===============================================================

void FindRadius() // Use Compass heading to calculate a turning radious
{
  // Turning radious: + Radius will turn left, - Radius will turn right
    
  if      (Bearing < 180 && (Bearing+180)<HEADING )      {DeltaC = Bearing - HEADING;}
  else if (180 < Bearing && HEADING < (Bearing - 180))   {DeltaC = Bearing - HEADING;}
  else                                                   {DeltaC = HEADING - Bearing;} // Corects for sign when Compass heading is in second quadrent
  
  
  Radius = map(DeltaC, -45, 45, -7, 7 );    // Map out turing radious.
  Radius = constrain(Radius, -7, 7);        // Constrain Turing radius to + or - 7 to prevent jack-knifing
  
}
  
  




