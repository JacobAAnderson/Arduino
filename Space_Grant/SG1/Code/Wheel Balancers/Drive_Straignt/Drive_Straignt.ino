// Libraries
#include <Servo.h>;

// Defined values
#define LSM (77);   // Median left  side wheel speed. Left  wheels:  Forward < 90
#define RSM (103);  // Median right side wheel speed. Right wheels:  Forward > 90

// Servos
Servo A;     // Wheel A  Back left wheel
Servo B;     // Wheel B  Back right wheel
Servo C;     // Wheel C  Front left wheel
Servo D;     // Wheel D  Front right whell
Servo Lidar; // Servo that positions LIDAR

// Variables
int leftWheels;    // Varialbe used to control the right wheels' speed
int rightWheels;   // Variable used to control the left wheels' speed


// == set up routien=============================================================
void setup(){
  
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

// == Assign inital values to varialbes
  leftWheels = LSM;
  rightWheels = RSM;
}

// == Main loop =================================================================

void loop() {
  
  A. write(leftWheels);   // Wheel A  Back  left  wheel:  Forward < 90
  B. write(rightWheels);  // Wheel B  Back  right wheel:  Forward > 90
  C. write(leftWheels);   // Wheel C  Front left  wheel:  Forward < 90
  D. write(rightWheels);  // Wheel D  Front right whell:  Forward > 90
}
