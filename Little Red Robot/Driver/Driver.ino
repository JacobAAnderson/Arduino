// This script combins IR sensors, bumper sensors and a magnetic compass to drive a robot.

// Compass Code
  #include <Wire.h> // Reference the I2C Library
  #include <HMC5883L.h> // Reference the HMC5883L Compass Library
  HMC5883L compass; // Store our compass as a variable.
  int error = 0;  // Record any errors that may occur in the compass.

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
  float dC;       // Varialbe for the diiferance between the bearing and compass heading
  
  #define Bearing 90// Direction of travel //==//==//==//==//==//==//==//==//==//==//==//==//==//==// 
   
void setup()
{ 
  Serial.begin(9600); // Initialize the serial port.
  
  // Compass Code - Out setup routine, here we will configure the microcontroller and compass.
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
  
  Stop(); // Stop the wheels
}

// Our main program loop.
void loop()
{
// Compass Code 
  MagnetometerRaw raw = compass.ReadRawAxis(); // Retrive the raw values from the compass (not scaled).
  MagnetometerScaled scaled = compass.ReadScaledAxis(); // Retrived the scaled values from the compass (scaled to the configured scale).
  
  // Values are accessed like so:
  int MilliGauss_OnThe_XAxis = scaled.XAxis;// (or YAxis, or ZAxis)
  float heading = atan2(scaled.YAxis, scaled.XAxis); // Calculate heading when the magnetometer is level, then correct for signs of axis.
  float declinationAngle = 0.0457; // 2.618 degrees East declination
  heading += declinationAngle;
  
  if(heading < 0)  // Correct for when signs are reversed.
    heading += 2*PI;
    
  if(heading > 2*PI) // Check for wrap due to addition of declination.
    heading -= 2*PI;
   
  float headingDegrees = heading * 180/M_PI; // Convert radians to degrees for readability.

  Output(raw, scaled, heading, headingDegrees); // Output the data via the serial port.

// Data from IR sensors
  IRleft = analogRead(A0);
  IRright = analogRead(A2);
  leftBumper = digitalRead(2);
  rightBumper = digitalRead(4);  
      
// Driving the robot with IR sensor and compass
  // Compass varialbe "dC"
  if(Bearing < 180 && (Bearing+180)< headingDegrees) { dC = Bearing - headingDegrees; }
  else if (Bearing > 180 && (Bearing - 180) > headingDegrees) { dC = Bearing - headingDegrees; }
  else { dC = headingDegrees - Bearing; }
  Serial.print("DC:\t");
  Serial.print(dC);
  
  
  // Wheel speed equations
  wheelA = abs( 150 - IRleft * exp(-0.0025 * IRleft) + 0.5*dC*exp(-0.03*(IRleft+IRright)));   
  wheelB = abs( 150 - IRright* exp(-0.0025 * IRright)- 0.5*dC*exp(-0.03*(IRleft+IRright)));
  
  // Driving sequence
  if(IRleft > 250 && IRright > 250)  { backUp();  }  // Obstical infront of robot, back up and turn left
  else if (leftBumper == 1)  {  LeftBumper(); }        // Left bumper hit an obstical, back up turn right
  else if (rightBumper == 1) {  RightBumper(); }      // Right bumper hit an obstical, back up turn left
  
  else {  digitalWrite(ledG, HIGH);  // No obstical 
          digitalWrite(ledY, LOW );
          digitalWrite(ledR, LOW );
    
          digitalWrite(dir_a, HIGH);
          digitalWrite(dir_b, HIGH);
          analogWrite(pwm_a, wheelA);   
          analogWrite(pwm_b, wheelB); }  
}

//======= Sub-rutiens ========================================================

// Compass Code - Output the data down the serial port.
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

// Sub-rutiens for backing up and turning around obstacles.
void backUp()  // Back up and turn left when IR sensors detect an object infront of the robot
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
   
void LeftBumper()  // Baco up and turn right 
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
    
void Stop() // stops the wheels
{   analogWrite(pwm_a, 0);    
    analogWrite(pwm_b, 0);
    delay(100);   
}

void BackUp() // Backs up the robot
{   digitalWrite(dir_a, LOW);  // Back up
    digitalWrite(dir_b, LOW);
    analogWrite(pwm_a, 100);    
    analogWrite(pwm_b, 120);
    delay(1000);
}
