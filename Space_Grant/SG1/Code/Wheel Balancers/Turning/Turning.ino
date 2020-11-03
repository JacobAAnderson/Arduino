// Libraries
#include <Servo.h>;

// Defined values


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
int LSM = 77;   // Median left  side wheel speed. Left  wheels:  Forward < 90
int RSM = 103;  // Median right side wheel speed. Right wheels:  Forward > 90
int leftWheels;    // Varialbe used to control the right wheels' speed
int rightWheels;   // Variable used to control the left wheels' speed
int Radius;        // Turning radious: + Radius will turn left, - Radius will turn right

// == set up routien=============================================================
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
  
  delay (1000);
  
  digitalWrite(LEDg, LOW);
  digitalWrite(LEDy, LOW);
  digitalWrite(LEDr, LOW);
  
  delay(1000);

// == Set up initial values for variables ===================
 }

// == Main loop =================================================================

void loop() {
  digitalWrite(LEDg, HIGH);
  Drive();
  delay (1000);
  
  digitalWrite(LEDg, LOW);
  digitalWrite(LEDy, HIGH);
  digitalWrite(LEDr, LOW);
  Radius = -7;    // Turning radious: + Radius will turn left, - Radius will turn right
  Drive();
  delay(1000);
  
  digitalWrite(LEDg, LOW);
  digitalWrite(LEDy, LOW);
  digitalWrite(LEDr, HIGH);
  Radius = 5;
  Drive();
  delay(2000);
  
  digitalWrite(LEDg, HIGH);
  digitalWrite(LEDy, LOW);
  digitalWrite(LEDr, LOW);
  Radius = 0;
  Drive();
  delay(1000);
  
}
// =========== Subrutiens ==============================================================================

// ===== Driving routien ================================================
void Drive(){
  
  // Assign wheel speed values
  // Turning radious: + Radius will turn left, - Radius will turn right
  // LSM = 77   RSM = 103
  if (Radius < 0) {
   leftWheels  = (LSM + Radius);    
   rightWheels = (RSM + Radius);
  }
  else { leftWheels  = (LSM + Radius);
         rightWheels = (RSM + Radius);
  }
  
  // Drive
  A. write(LSM);   // Wheel A  Back  left  wheel:  Forward < 90
  B. write(RSM);   // Wheel B  Back  right wheel:  Forward > 90
  C. write(leftWheels);   // Wheel C  Front left  wheel:  Forward < 90
  D. write(rightWheels);  // Wheel D  Front right whell:  Forward > 90
  
  Serial.print(Radius);
  Serial.print ("     ");
  Serial.print (leftWheels);
  Serial.print ("     ");
  Serial.println (rightWheels);
}
