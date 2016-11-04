/*  Basic driving code for Space Grant Robot 1
    Jacob Anderson 8/31/2015
    
    Notes: -------------------------------------------------------------------
     Wheel A  Back  left  wheel:  Forward < 90
     Wheel B  Back  right wheel:  Forward > 90
     Wheel C  Front left  wheel:  Forward < 90
     Wheel D  Front right whell:  Forward > 90
     
     Turning radious: + Radius will turn left, - Radius will turn right
*/


// Libraries -----------------------------------------------------------------------------------------------------
#include <I2C.h>
#include <Servo.h>

// Definitions ---------------------------------------------------------------------------------------------------
#define    HMC5883L            0x1E          // Compass Address
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

// Name Servos ---------------------------------------------------------------------------------------------------
Servo A;     // Wheel A  Back left wheel
Servo B;     // Wheel B  Back right wheel
Servo C;     // Wheel C  Front left wheel
Servo D;     // Wheel D  Front right whell
Servo Lidar; // Servo that positions LIDAR

// Global variables ----------------------------------------------------------------------------------------------
int LSM =  82;   // Median left  side wheel speed. Left  wheels:  Forward < 90
int RSM = 103;   // Median right side wheel speed. Right wheels:  Forward > 90



// == set up routien==============================================================================================
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
  
 // Set up I2C devices ----------------------------------------------------------------------------------------------------- 
  I2c.begin();
  I2c.write(HMC5883L,0x02,0x00);  // Configure device for continuous mode
  delay(100);                     // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50);                // Sets a timeout to ensure no locking up of sketch if I2C communication fails


// == Assign inital values to varialbes


 Serial.println(" ------- Setup Done ----------"), Serial.println("");  
}




// == Main loop =================================================================

void loop() {
/*  
DriveForward(0);
delay(100);

for (int Speed = 0;  Speed < 31; Speed++)  {DriveForward( Speed ); delay(100);}
for (int Speed = 30; Speed >= 0; Speed--)  {DriveForward( Speed ); delay(100);}

Stop();
delay(500);

DriveBackwards();
*/

Turn(3);
  
}






/*----------------------------------------------------------------------------------------------------------------------------
========================= Subrutiens =========================================================================================
----------------------------------------------------------------------------------------------------------------------------*/

// Driving rutiens -----------------------------------------------------------------------------------------------------------
void DriveForward( int Speed ) {

  int leftWheels  = LSM - Speed;
  int rightWheels = RSM + Speed;
  
  A. write(leftWheels);   // Wheel A  Back  left  wheel:  Forward < 90
  B. write(rightWheels);  // Wheel B  Back  right wheel:  Forward > 90
  C. write(leftWheels);   // Wheel C  Front left  wheel:  Forward < 90
  D. write(rightWheels);  // Wheel D  Front right whell:  Forward > 90
}

void DriveBackwards() {
 
  A. write(RSM);  // Wheel A  Back  left  wheel:  Forward < 90
  B. write(LSM);  // Wheel B  Back  right wheel:  Forward > 90
  C. write(RSM);  // Wheel C  Front left  wheel:  Forward < 90
  D. write(LSM);  // Wheel D  Front right whell:  Forward > 90
}

void Turn( int radius ) {
  
  // Assign wheel speed values
  // Turning radious: + Radius will turn left, - Radius will turn right
  // LSM = 77   RSM = 103
  
  int backLeft  = LSM + radius;
  int backRight = RSM + radius;
  int frontLeft  = LSM + radius;
  int frontRight = RSM + radius;
  
  if (radius > 0) { frontRight = frontRight + 1;}
  else            { frontLeft  = frontLeft - 1; }
                    
                   
  // Drive
  A. write(backLeft);    // Wheel A  Back  left  wheel:  Forward < 90
  B. write(backRight);   // Wheel B  Back  right wheel:  Forward > 90
  C. write(frontLeft);   // Wheel C  Front left  wheel:  Forward < 90
  D. write(frontRight);  // Wheel D  Front right whell:  Forward > 90
  
 
}


void Stop(){
  
  A. write(90);   
  B. write(90);
  C. write(90);
  D. write(90);
  
}
