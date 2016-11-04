// This script combins IR sensors, bumper sensors and a magnetic compass to drive a robot.

// Compass
  // Reference the I2C Library
  #include <Wire.h>
  // Reference the HMC5883L Compass Library
  #include <HMC5883L.h>

  // Store our compass as a variable.
  HMC5883L compass;
  // Record any errors that may occur in the compass.
  int error = 0;

// Inputs and outputs:
  // Wheels
  int pwm_a = 3;    // Right Wheel speed
  int pwm_b = 11;   // Left wheel speed
  int dir_a = 12;   // Right wheel direction
  int dir_b = 13;   // Left wheel direction

  // Sensors
  int IRright = A0;    // Right IR Sensor
  int IRleft  = A2;    // Left IR Sensor
  int leftBumper =  2; // Left bumper switch
  int rightBumper = 4; // Rigth Bumper switch  
 
  // Lights
  int ledG = 5;  // Green LED indicates that the path is clear.
  int ledY = 6;  // Yelow LED indicates an obsital has been detected and the robot will slow down.
  int ledR = 7;  // Red LED indicates that the robot is stoping and backing up.
  
 // Variables:
  float wheelA;  // Right wheel
  float wheelB;  // Left wheel
  float Bearing; // Magnetic reading for the robot to follow.
  float dC;       // Varialbe for the diiferance between the bearing and compass heading



//====== Setup routine ===============================================================================
void setup()
{
// Compass, here we will configure the microcontroller and compass. 
  // Initialize the serial port.
  Serial.begin(9600);

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
 
//==== Direction of travel ===============================================================================
  Bearing = 90; // Tell robot to travel East  

// Pin inputs and outputs
  // Set control pins for the wheels
  pinMode(pwm_a, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);
  
  // Set up pins for sensors
  pinMode(IRleft, INPUT);  //Reading from Port IR sensor
  pinMode(IRright, INPUT); // Read from starboard IR sensor
  
  pinMode(leftBumper, INPUT); // Reading from bumper switches
  pinMode(rightBumper, INPUT);
  
  // Set up pins for LEDs
  pinMode(ledG, OUTPUT);
  pinMode(ledY, OUTPUT);
  pinMode(ledR, OUTPUT);
  
  // Set both moters to 0 - stop
  analogWrite(pwm_a, 0);   
  analogWrite(pwm_b, 0);
}

//==== Main program loop. ===========================================================
void loop()
{
// Compass  
  // Retrive the raw values from the compass (not scaled).
  MagnetometerRaw raw = compass.ReadRawAxis();
  // Retrived the scaled values from the compass (scaled to the configured scale).
  MagnetometerScaled scaled = compass.ReadScaledAxis();
  
  // Values are accessed like so:
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)

  // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float heading = atan2(scaled.YAxis, scaled.XAxis);
  
  
  float declinationAngle = 0.0457;
  heading += declinationAngle;
  
  // Correct for when signs are reversed.
  if(heading < 0)
    heading += 2*PI;
    
  // Check for wrap due to addition of declination.
  if(heading > 2*PI)
    heading -= 2*PI;
   
  // Convert radians to degrees for readability.
  float headingDegrees = heading * 180/M_PI; 

  // Output the data via the serial port.
  Output(raw, scaled, heading, headingDegrees);

// Data from IR sensors
  IRleft = analogRead(A0);
  IRright = analogRead(A2);
  leftBumper = digitalRead(2);
  rightBumper = digitalRead(4);  
      
// Driving the robot with IR sensor and compass
  // Compass corection varialbe
  if(Bearing < 180 && (Bearing+180)< headingDegrees) { dC = Bearing - headingDegrees; }
  else if (Bearing > 180 && (Bearing - 180) > headingDegrees) { dC = Bearing - headingDegrees; }
  else { dC = headingDegrees - Bearing; }
  Serial.print("DC:\t");
  Serial.print(dC);
  
  
  // Wheel speed equations
  wheelA = abs(100*(1 + 0.25*sin(dC*M_PI/180)));   
  wheelB = abs(130*(1 - 0.25*sin(dC*M_PI/180)));
  
  // Driving sequence
  if(IRleft > 250 && IRright > 250)  { backUp();  } // Obstical infront of robot, back up and turn left
  else if (leftBumper > 0) {  LeftBumper(); }        // Left bumper hit an obstical, back up turn right
  else if (rightBumper > 0) {  RightBumper(); }      // Right bumper hit an obstical, back up turn left
  
  else {  digitalWrite(ledG, HIGH);  // No obstical 
          digitalWrite(ledY, LOW );
          digitalWrite(ledR, LOW );
    
          digitalWrite(dir_a, HIGH);
          digitalWrite(dir_b, HIGH);
          analogWrite(pwm_a, wheelA);   
          analogWrite(pwm_b, wheelB); }  
}

//================== Sub-rutiens  ==========================================================================

//================== Output the data down the serial port =======================================
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

//============ Sub-rutiens for backing up and turning around obstacles.=======================

void backUp()  // Back up and turn left when IR sensors detect an object infront of the robot
{   digitalWrite(ledG, LOW); 
    digitalWrite(ledY, LOW);
    digitalWrite(ledR, HIGH);  // Red LED on
    
    digitalWrite(dir_a, HIGH);  // Stop
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, 0);    
    analogWrite(pwm_b, 0);
    delay(100);   
    
    digitalWrite(dir_a, LOW);  // Back up
    digitalWrite(dir_b, LOW);
    analogWrite(pwm_a, 100);    
    analogWrite(pwm_b, 120);
    delay(1000); 
    
    digitalWrite(dir_a, HIGH);  // Stop
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, 0);    
    analogWrite(pwm_b, 0);
    delay(100);   
  
    digitalWrite(dir_a, HIGH);  // Turn left
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, 100);    
    analogWrite(pwm_b, 0); 
    delay(250);              }   
    
//========================================================
   
void LeftBumper()  // Baco up and turn right 
{   digitalWrite(ledG, LOW); 
    digitalWrite(ledY, HIGH); // Yellow LED on
    digitalWrite(ledR, LOW);
    
    analogWrite(pwm_a, 0);  // Stop  
    analogWrite(pwm_b, 0);
    delay(100);   
    
    digitalWrite(dir_a, LOW);  // Back up
    digitalWrite(dir_b, LOW);
    analogWrite(pwm_a, 100);    
    analogWrite(pwm_b, 120);
    delay(1000); 
    
    analogWrite(pwm_a, 0);   // Stop 
    analogWrite(pwm_b, 0);
    delay(100);   
  
    digitalWrite(dir_a, HIGH);  // Turn right
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, 0);    
    analogWrite(pwm_b, 100); 
    delay(250);              }   

//===================================================

void RightBumper()
{   digitalWrite(ledG, LOW); 
    digitalWrite(ledY, HIGH); // Yellow LED on
    digitalWrite(ledR, LOW);
    
    digitalWrite(dir_a, HIGH);  // Stop
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, 0);    
    analogWrite(pwm_b, 0);
    delay(100);   
    
    digitalWrite(dir_a, LOW);  // Back up
    digitalWrite(dir_b, LOW);
    analogWrite(pwm_a, 100);    
    analogWrite(pwm_b, 120);
    delay(1000); 
    
    digitalWrite(dir_a, HIGH);  // Stop
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, 0);    
    analogWrite(pwm_b, 0);
    delay(100);   
  
    digitalWrite(dir_a, HIGH);  // Turn left
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, 100);    
    analogWrite(pwm_b, 0); 
    delay(250);              }
