// This script combins IR sensors, bumper sensors and a magnetic compass to drive a robot.

// Inputs and outputs:
  // Wheels
  int pwm_a = 3;    // Right Wheel speed
  int pwm_b = 11;   // Left wheel speed
  int dir_a = 12;   // Right wheel direction
  int dir_b = 13;   // Left wheel direction

  // Sensors
  int LeftIRpin = A0;   // Left IR sensor  
  int RightIRpin = A1;  // Right IR sensor
  
  int leftBumper =  2;  // Left bumper switch
  int rightBumper = 4;  // Rigth Bumper switch  
 
  // Lights
  int ledG = 5;  // Green LED indicates that sensors are calibrating or the path is clear.
  int ledY = 6;  // Yelow LED indicates an obsital has been detected and the robot will slow down.
  int ledR = 7;  // Red LED indicates that the robot is stoping and backing up.
  
// Variables:
  int RawIRleft = 0;      // Raw IR sensor values
  int RawIRright = 0;

  int LeftIRmin = 1023;   // IR sensor calibration values
  int LeftIRmax = 0;
  int RightIRmin = 1023;
  int RightIRmax = 0;
  
  int IRleft = 0;         // Mapped IR values  
  int IRright =0;
  
  int wheelA;  // Right wheel
  int wheelB;  // Left wheel

// Vectors to store IR readins
  const int IRvec = 5;   // number of spaces in the vectors
  int LeftIRvec[IRvec];  // Left IR sensor vector
  int RightIRvec[IRvec]; // Right IR sensor vector
  
  int IRindex = 0;        //Index number for the IR vectors' value being changed
  int LeftIRtotal = 0;    // Sum of the Left IR vector
  int RightIRtotal = 0;   // Sum of the right IR vector
  
   
void setup() {Serial.begin(9600); 
  
// Pin inputs and outputs
  // Set control pins for the wheels
  pinMode(pwm_a, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);
  
  // Set up pins for sensors
  pinMode(LeftIRpin, INPUT);  //Reading from IR sensors
  pinMode(RightIRpin, INPUT);
  
  pinMode(leftBumper, INPUT); // Reading from bumper switches
  pinMode(rightBumper, INPUT);
  
  // Set up pins for LEDs
  pinMode(ledG, OUTPUT);
  pinMode(ledY, OUTPUT);
  pinMode(ledR, OUTPUT);

// Run calibration and startup subrutiens  
  Stop(); // Stop the wheels
  VectorSetup(); // Runs a loop that sets all IR values to 0
  CalibrateIR(); // Run the IR calibration subrutien
}

void loop() {

// Data from IR sensors
  SmoothIR();  // Runs the IR sensor smoothing subrutien
  mapIR();     // Run the IR maping subrutien

// Bump sensors  
  leftBumper = digitalRead(2);
  rightBumper = digitalRead(4);  
      
// Driving the robot with IR sensor and compass
 if(IRleft < 25) {wheelA = 100;}  // Wheel speed equations
 else {wheelA = (100 - IRleft);}
 wheelA = constrain( wheelA, 0, 100);
 
 if(IRright < 25) {wheelB = 120;}
 else { wheelB = (120 - IRright);}
 wheelB = constrain(wheelB, 0, 120);
    
  // Driving sequence
  if(IRleft > 60 && IRright > 60)  { backUp();  }   // Obstical infront of robot, back up and turn left
  else if (leftBumper == 1)  {  LeftBumper(); }       // Left bumper hit an obstical, back up turn right
  else if (rightBumper == 1) {  RightBumper(); }      // Right bumper hit an obstical, back up turn left
  
  else {  digitalWrite(ledG, HIGH);  // No obstical 
          digitalWrite(ledY, LOW );
          digitalWrite(ledR, LOW );
    
          digitalWrite(dir_a, HIGH);
          digitalWrite(dir_b, HIGH);
          analogWrite(pwm_a, wheelA);   
          analogWrite(pwm_b, wheelB); } 
         
  Serial.print("Wheel A: ");
  Serial.print(wheelA);
  Serial.print("\tWheel B: ");
  Serial.println(wheelB); 
}

//======= Sub-rutiens ==========================================================================================
// ===== Sensor data subrutiens ==================================

void CalibrateIR() { // IR calibration subrutien
  
  while (millis() < 5000) {
    digitalWrite(ledG, HIGH);
    
    RawIRleft  = analogRead(A0);
    RawIRright = analogRead(A1);
    
    if (RawIRleft  > LeftIRmax)  {LeftIRmax  = RawIRleft;}   // record the maximum sensor value
    if (RawIRright > RightIRmax) {RightIRmax = RawIRright;}
    
    if (RawIRleft  < LeftIRmin)  {LeftIRmin  = RawIRleft;}  // record the minimum sensor value
    if (RawIRright < RightIRmin) {RightIRmin = RawIRright;}
  }
  digitalWrite(ledG, LOW);
  
  Serial.println("  ");
  Serial.print("Max left IR value: ");    // Print results
  Serial.print(LeftIRmax);
  Serial.print("\tMin left IR value: ");
  Serial.print(LeftIRmin);
  
  Serial.print("\tMax right IR value: ");
  Serial.print(RightIRmax);
  Serial.print("\tMin right IR value: ");
  Serial.println(RightIRmin);
}
//=========================================================================

void VectorSetup (){ // initialize all vector values to 0
  for (int zero = 0; zero < IRvec; zero++)  
    LeftIRvec[zero] = 0,
    RightIRvec[zero] = 0;
}
//=========================================================================

void SmoothIR(){  // Sensor averaging subrutien
      
// Aveage left IR sensor readings
   LeftIRtotal = LeftIRtotal - LeftIRvec[IRindex];      // Subtract the last reading from the vector
   LeftIRvec[IRindex] = analogRead(LeftIRpin);          // Read from the sensors
   LeftIRtotal = LeftIRtotal + LeftIRvec[IRindex];      // Add the reading from to the total.
   IRleft = LeftIRtotal / IRvec;                        // Calculate average sensor value
   
// Averager right IR sensor readings
   RightIRtotal = RightIRtotal - RightIRvec[IRindex];   // Subtract the last reading from the vector
   RightIRvec[IRindex] = analogRead(RightIRpin);        // Read from the sensors
   RightIRtotal = RightIRtotal + RightIRvec[IRindex];   // Add the reading from to the total.
   IRright = RightIRtotal / IRvec;                      // Calculate average sensor value
   
   IRindex = IRindex + 1;               // Advance to the next position in the array
   if (IRindex >= IRvec) IRindex = 0;   // Wrap around to the begining o fthe aray once your at the end

  Serial.print("\tSmoothed left IR: ");
  Serial.print(IRleft);
  Serial.print("\tSmoothed right IR: ");
  Serial.print(IRright);  
}
//========================================================================================

void mapIR() {  // Subtutien that maps the IR value on a range from 0 to 150
  // read the sensor:
 
  // apply the calibration to the sensor reading
  IRleft  = map(IRleft,  LeftIRmin,  LeftIRmax,  0, 150);
  IRright = map(IRright, RightIRmin, RightIRmax, 0, 150);

  // in case the sensor value is outside the range seen during calibration
  IRleft  = constrain(IRleft,  0, 150);
  IRright = constrain(IRright, 0, 150);

  Serial.print("\tMapped left IR value: ");
  Serial.print(IRleft);
  Serial.print("\tMapped right IR value: ");
  Serial.println(IRright);
}

//======================================================================================
// ====== Sub-rutiens for backing up and turning around obstacles. =====================

void backUp()  // Back up and turn left when both IR sensors detect an object infront of the robot
{   digitalWrite(ledG, LOW); 
    digitalWrite(ledY, LOW);
    digitalWrite(ledR, HIGH);  // Red LED on
    
    Stop();
    BackUp();
    Stop();  
  
    digitalWrite(dir_a, HIGH);  // Turn left
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, 100);    
    analogWrite(pwm_b, 0); 
    delay(250);
}   
//===================================================
   
void LeftBumper()  // Back up and turn right 
{   digitalWrite(ledG, LOW); 
    digitalWrite(ledY, HIGH); // Yellow LED on
    digitalWrite(ledR, LOW);
    
    Stop();
    BackUp();
    Stop();    
    
    digitalWrite(dir_a, HIGH);  // Turn right
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, 0);    
    analogWrite(pwm_b, 100); 
    delay(500);              }   

//====================================================    
void RightBumper()
{   digitalWrite(ledG, LOW); 
    digitalWrite(ledY, HIGH); // Yellow LED on
    digitalWrite(ledR, LOW);
    
    Stop();
    BackUp();
    Stop();
    
    digitalWrite(dir_a, HIGH);  // Turn left
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, 100);    
    analogWrite(pwm_b, 0); 
    delay(500);              }

//===================================================   
void Stop() // stops the wheels
{   analogWrite(pwm_a, 0);    
    analogWrite(pwm_b, 0);
    delay(100);   
}

//==================================================
void BackUp() // Backs up the robot
{   digitalWrite(dir_a, LOW);  // Back up
    digitalWrite(dir_b, LOW);
    analogWrite(pwm_a, 100);    
    analogWrite(pwm_b, 120);
    delay(1000);
}
