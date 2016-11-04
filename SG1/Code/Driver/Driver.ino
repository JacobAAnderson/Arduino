/* Basic Driving Code for SG1  */


// Libraries
#include <Servo.h>;    // Motor servo library
#include <Wire.h>;     // Serial comunication library
#define address 0x1E   // Compass Addrss

// Define sensor inputs
int IRleft  = A0;  // Left Cliff Sensor
int IRmid   = A1;  // Midle Clif Sensor
int IRright = A2;  // Right Cliff sensor

// Define Servos
Servo A;     // Wheel A  Back left wheel
Servo B;     // Wheel B  Back right wheel
Servo C;     // Wheel C  Front left wheel
Servo D;     // Wheel D  Front right whell
Servo Lidar; // Servo that positions LIDAR

// Define LEDs
int LEDg = 8;    // Green  LED attach to pin 8
int LEDy = 9;    // Yellow LED attache to pin 9
int LEDr = 10;   // Red LED attach to pin 10
int LEDb = 11;   // Red LED for Beacon signal confimation attach to pin 11

// Define varialbes


// == Setup rutien =========================================================================================
void setup() { Serial.begin(9600);

// == Servos ==============================================  
  A. attach(2);      // Attach Back  left  wheel to pin 2
  B. attach(3);      // Attach Back  right wheel to pin 3
  C. attach(4);      // Attach Front left  wheel to pin 4
  D. attach(5);      // Attach Front right wheel to pin 5
  Lidar. attach(6);  // Attach LIDAR Servo to pin 6
  
  // Stop driving motors: Servo write from 0 to 180, 0  to  90 is counter clock wise, 90 to 180 is clockwise
  A. write(90);   
  B. write(90);
  C. write(90);
  D. write(90);
  Lidar.write(85); // Center Lidar Servo, Lidar centers with an input of 85, 90 is a little off center.
  
// == LEDs, Set up LED pins as outputs ====================
  pinMode(LEDg, OUTPUT); 
  pinMode(LEDy, OUTPUT); 
  pinMode(LEDr, OUTPUT); 
  pinMode(LEDb, OUTPUT);
  
  // turn LEDs on to indicat that they are working
  digitalWrite(LEDg,HIGH);
  digitalWrite(LEDy,HIGH);
  digitalWrite(LEDr,HIGH);
  digitalWrite(LEDb,HIGH);
  
// == Sensors, Set up sensor pins as inputs ================
  pinMode(IRleft,  INPUT);
  pinMode(IRmid,   INPUT);
  pinMode(IRright, INPUT);
  
// == Compass ===============================================
  //Code from https://www.sparkfun.com/tutorials/301
  //Initialize Serial and I2C communications
  Wire.begin();
  
  //Put the HMC5883 IC into the correct operating mode
  Wire.beginTransmission(address); //open communication with HMC5883
  Wire.write(0x02); //select mode register
  Wire.write(0x00); //continuous measurement mode
  Wire.endTransmission();
}


// == Main Loop ===============================================================================================
void loop() {

  Compass();
  Runmotors(50); // Runs motors for 50 micro Seconds  
  

}


// ===========================  Subrutiens  ================================================================
// ====================================================================================
void Stopmotors(int input){
// Servo write from 0 to 180, 0  to  90 is counter clock wise, 90 to 180 is clockwise
  A. write(90);
  B. write(90);
  C. write(90);
  D. write(90);
}

// ====================================================================================
void Runmotors(int input){
// Servo write from 0 to 180, 0  to  90 is counter clock wise, 90 to 180 is clockwise
  A. write(90);
  B. write(90);
  C. write(90);
  D. write(90);
}

// ===================================================================================
void Compass(){
  // Compass
int x,y,z; //triple axis data

  //Tell the HMC5883L where to begin reading data
  Wire.beginTransmission(address);
  Wire.write(0x03); //select register 3, X MSB register
  Wire.endTransmission();
  
 
 //Read data from each axis, 2 registers per axis
  Wire.requestFrom(address, 6);
  if(6<=Wire.available()){
    x = Wire.read()<<8; //X msb
    x |= Wire.read(); //X lsb
    z = Wire.read()<<8; //Z msb
    z |= Wire.read(); //Z lsb
    y = Wire.read()<<8; //Y msb
    y |= Wire.read(); //Y lsb
  }
  
  //Print out values of each axis
  Serial.print("x: ");
  Serial.print(x);
  Serial.print("  y: ");
  Serial.print(y);
  Serial.print("  z: ");
  Serial.println(z);
  
  delay(250); 
}

