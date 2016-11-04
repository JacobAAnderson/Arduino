/* This scrip combines driving the robot with use of a IR sensor for indicating obsticals
*/

// Inputs and outputs:
  // Wheels
  int pwm_a = 3;    // Right Wheel speed
  int pwm_b = 11;   // Left wheel speed
  int dir_a = 12;   // Right wheel direction
  int dir_b = 13;   // Left wheel direction

  // Sensors
  int IRright = A0;    // Right IR Sensor
  int IRleft  = A2;    // Left IR Sensor
  
 
  // Lights
  int ledG = 5;  // Green LED indicates that the path is clear.
  int ledY = 6;  // Yelow LED indicates an obsital has been detected and the robot will slow down.
  int ledR = 7;  // Red LED indicates that the robot is stoping and backing up.
  
 // Variables:
  float wheelA;  // Right wheel
  float wheelB;  // Left wheel

void setup() {
  // put your setup code here, to run once:
  
  Serial.begin(9600);  // communication rate
  
  // Set control pins for the wheels
  pinMode(pwm_a, OUTPUT);
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);
  
  // Set up pins for sensors
  pinMode(IRleft, INPUT);  //Reading from Port IR sensor
  pinMode(IRright, INPUT); // Read from starboard IR sensor
  
   
  // Set up pins for LEDs
  pinMode(ledG, OUTPUT);
  pinMode(ledY, OUTPUT);
  pinMode(ledR, OUTPUT);
  
  // Set both moters to 0 - stop
  analogWrite(pwm_a, 0);   
  analogWrite(pwm_b, 0);
  
  }

void loop() {
  // put your main code here, to run repeatedly: 
  
// Calculate IR1 voltage
  IRleft = analogRead(A0);
  IRright = analogRead(A2);
      
  Serial.print("Left IR:   ");
  Serial.print(IRleft);
  Serial.print("   Right IR:   ");
  Serial.print(IRright);
  
// Driving the robot with IR sensor
  
  // Wheel speed equations
  wheelA = abs( 150 - IRleft * exp(-0.003 * IRleft));   
  wheelB = abs( 150 - IRright* exp(-0.003 * IRright));
  
  Serial.print("   Wheel A:   ");
  Serial.print(wheelA);
  Serial.print("   Wheel B:   ");
  Serial.println(wheelB);
  
  
  
  // If there is not an obstical in fron of the robot drive via wheel equations
  if(IRleft > 250 && IRright > 250)  { backUp();  }
 
  
  // If an obstical is detected, back up and turn left
  else { 
    digitalWrite(ledG, HIGH);
    digitalWrite(ledY, LOW );
    digitalWrite(ledR, LOW );
  
    digitalWrite(dir_a, HIGH);
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, wheelA);   
    analogWrite(pwm_b, wheelB); }
  
}

//  Sub-rutiens  ///////////////////////////////////////////

void backUp()
{   digitalWrite(ledG, LOW); 
    digitalWrite(ledY, LOW);
    digitalWrite(ledR, HIGH);  // Red LED on
    
    digitalWrite(dir_a, HIGH);  // Stop
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, 0);    
    analogWrite(pwm_b, 0);
    delay(700);   
    
    digitalWrite(dir_a, LOW);  // Back up
    digitalWrite(dir_b, LOW);
    analogWrite(pwm_a, 100);    
    analogWrite(pwm_b, 120);
    delay(1000); 
    
    digitalWrite(dir_a, HIGH);  // Stop
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, 0);    
    analogWrite(pwm_b, 0);
    delay(700);   
  
    digitalWrite(dir_a, HIGH);  // Turn left
    digitalWrite(dir_b, HIGH);
    analogWrite(pwm_a, 100);    
    analogWrite(pwm_b, 0); 
    delay(500);              }   
   

