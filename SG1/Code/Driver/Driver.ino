/*
    Jacob Anderson 8/31/2015
    
    Notes: -------------------------------------------------------------------
     Wheel A  Back  left  wheel:  Forward < 90    Arduino Pin 2
     Wheel B  Back  right wheel:  Forward > 90    Arduino Pin 3
     Wheel C  Front left  wheel:  Forward < 90    Arduino Pin 4
     Wheel D  Front right whell:  Forward > 90    Arduino Pin 5

     Servo write from 0 to 180: 
      0  to  90 is counter clockwise
      90 to 180 is clockwise
     
     Turning radious: + Radius will turn left, - Radius will turn right
*/


// Libraries -----------------------------------------------------------------------------------------------------
#include <Servo.h>


// Name Servos ---------------------------------------------------------------------------------------------------
Servo A;     // Wheel A  Back left wheel
Servo B;     // Wheel B  Back right wheel
Servo C;     // Wheel C  Front left wheel
Servo D;     // Wheel D  Front right whell

// Global variables ----------------------------------------------------------------------------------------------
// This are the atuned wheel inputs that will make the robot drive straight
int LSM =  82;   // Median left  side wheel speed. Left  wheels:  Forward < 90
int RSM = 103;   // Median right side wheel speed. Right wheels:  Forward > 90



// == set up routien==============================================================================================
void setup(){  Serial.begin(9600);
  
// == Attach Servos ==============================================  
  A. attach(2);      // Attach Back  left  wheel to pin 2
  B. attach(3);      // Attach Back  right wheel to pin 3
  C. attach(4);      // Attach Front left  wheel to pin 4
  D. attach(5);      // Attach Front right wheel to pin 5

  
// Stop driving motors: 
   
  A. write(90);   
  B. write(90);
  C. write(90);
  D. write(90);



 Serial.println(" ------- Setup Done ----------"), Serial.println("");  
}




// == Main loop =================================================================

void loop() { 
  
  /* Do not run this program on the roboat as is, it wont do much. 
   * At the least, add some delays inbetween calling the subrutiens.
    */

DriveForward(0); // Call the drive forward subrutien. The number in parrenthasies can be changed to alter the robot's speed.

DriveBackwards(); // Call the drive backwards subrutien. Do not put any numbers inside the parrenthasies.

Turn(3); // Call the turning subrutien. The number in parrenthases tels it how sharply to turn. + numbers go left, - numbers go right.

Stop(); // Call the stop subrutien. Do not put any numbers in the parrenthasies.

  
}






/*----------------------------------------------------------------------------------------------------------------------------
========================= Subrutiens =========================================================================================
----------------------------------------------------------------------------------------------------------------------------*/

// Driving rutiens -----------------------------------------------------------------------------------------------------------
void DriveForward( int Speed ) { // This tells the robot to drive forward at a sertain speed

  int leftWheels  = LSM - Speed;
  int rightWheels = RSM + Speed;
  
  A. write(leftWheels);   // Wheel A  Back  left  wheel:  Forward < 90
  B. write(rightWheels);  // Wheel B  Back  right wheel:  Forward > 90
  C. write(leftWheels);   // Wheel C  Front left  wheel:  Forward < 90
  D. write(rightWheels);  // Wheel D  Front right whell:  Forward > 90
}

void DriveBackwards() { // This tells the robot to drive back watrds, 1 set speed
 
  A. write(RSM);  // Wheel A  Back  left  wheel:  Forward < 90
  B. write(LSM);  // Wheel B  Back  right wheel:  Forward > 90
  C. write(RSM);  // Wheel C  Front left  wheel:  Forward < 90
  D. write(LSM);  // Wheel D  Front right whell:  Forward > 90
}

void Turn( int radius ) { // This turns the robot.
  
  // Assign wheel speed values
  // Turning radious: + Radius will turn left, - Radius will turn right
  // LSM = 77   RSM = 103

  
  int backLeft  = LSM + radius;
  int backRight = RSM + radius;
  int frontLeft  = LSM + radius;
  int frontRight = RSM + radius;

  // This tells the outside front wheel to turn a little bit faster than the other wheels which pulls it around a little better.
  if (radius > 0) { frontRight = frontRight + 1;}
  else            { frontLeft  = frontLeft - 1; }
                    
                   
  // Drive
  A. write(backLeft);    // Wheel A  Back  left  wheel:  Forward < 90
  B. write(backRight);   // Wheel B  Back  right wheel:  Forward > 90
  C. write(frontLeft);   // Wheel C  Front left  wheel:  Forward < 90
  D. write(frontRight);  // Wheel D  Front right whell:  Forward > 90
  
 
}


void Stop(){ // This stops the robot
  
  A. write(90);   
  B. write(90);
  C. write(90);
  D. write(90);
  
}
