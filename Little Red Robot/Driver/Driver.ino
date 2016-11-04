/*  Jacob Anderson 
 *  October 14th, 2015
 *  
 *  This code drives the Space Grant 0 robot
 *  This version has: Drives two brushed DC motoers with an H-drive shield
 *                    IR sensors perform obstacle avoidance
 *                    IR caliberation rutien at start up
 *           
 *  Future development: IMU
 *                      Intrupt on pin 2 for bumpers - Needs refinment
 *  ----------------------------------------------------------------------------------------------------------------
 *  Pins:  2 - Bumper interupt                        A0 - Right IR sensor
 *         3 - PWM for Right Wheel "a"                A1 - Left IR sensor
 *         5 - Green LED
 *         6 - Yellow LED
 *         7 - Red LED
 *         8 - Right Bumper
 *         9 - Left Bumper
 *         
 */

  // Motor shield configuration
  int pwm_a = 3;    // Right Wheel speed
  int pwm_b = 11;   // Left wheel speed
  int dir_a = 12;   // Right wheel direction
  int dir_b = 13;   // Left wheel direction
  
  int wam = 90;   // Wheel a mean speed
  int wbm = 110;  // Wheel b mean speed
  
 // Set up IR sensor variables
 int IRcal[] = {0, 0, 0, 0};  // [Right IR max, Right IR min, Left IR max, Left IR min ]
 
 int rightIRrange, leftIRrange;
 int rIRhigh, lIRhigh;
 int rIRmid, lIRmid;
 int rIRlow, lIRlow;

 float Time = 0;
 float Time_0 = 0;


void setup() {

  attachInterrupt(digitalPinToInterrupt(2), BUMP, LOW); // Bumper interupt
 
  pinMode(A0, INPUT); // IR sensor pins
  pinMode(A1, INPUT);

  pinMode(pwm_a, OUTPUT); // Set control pins for the wheels
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  // LED pins
  pinMode(5, OUTPUT); // Green  
  pinMode(6, OUTPUT); // Yellow
  pinMode(7, OUTPUT); // Red

  pinMode(8, INPUT);   // Left Bumper
  pinMode(9, INPUT);   // Right Bumper
  
  
  // Set both moters to 0 - stop
  analogWrite(pwm_a, 0);   
  analogWrite(pwm_b, 0);

  calibrateIR();

 rightIRrange = IRcal[0] - IRcal[1];
 leftIRrange  = IRcal[2] - IRcal[3];
 
 rIRhigh = IRcal[1] + rightIRrange/3;
 lIRhigh = IRcal[3] + leftIRrange/3;

 rIRmid  = IRcal[1] + rightIRrange/5;
 lIRmid  = IRcal[3] + leftIRrange/5;

 rIRlow  = IRcal[1] + rightIRrange/7;
 lIRlow  = IRcal[3] + leftIRrange/7;

  LEDstate(LOW, LOW, LOW);      // Tun LEDs on for half a second to indiacte that startup is complete
  LEDstate(HIGH, HIGH, HIGH);
  delay(500);
  LEDstate(LOW, LOW, LOW);

  }

void loop() {
  
  int rightIR = GetIR(0);
  int leftIR  = GetIR(2); 
  int turn = 0;
  int sped = 0;

  Time = millis()/1000.0;

  
  if (rightIR >= rIRhigh && leftIR >= lIRhigh ) { 
    Stop();

    if ( rightIR = leftIR ) { turn = 0; }
    else if ( rightIR < leftIR ) { turn = 10; }
    else { turn = -10; }

    BackUp( turn );
 }


else if (rightIR >= rIRmid || leftIR >= lIRmid) { 
  turn = rightIR - leftIR;
  turn = map(turn, -100, 100, -10, 10);

  sped = 200 - (leftIR + rightIR)/2 ;
  sped = constrain( sped, -5, 25);

  Drive( turn, sped );
 }
                                      
else if ( rightIR >= rIRlow && leftIR >= lIRlow ) { 
  turn = 0;
  sped = 100 - (leftIR + rightIR)/2 ;
  sped = constrain( sped, 0, 50);
  Drive( turn, sped );
 }

else {
  turn = 0;
  sped = 100 - (leftIR + rightIR)/2 ;
  sped = constrain( sped, 0, 100);
  Drive( turn, sped );
  
}


   Time_0 = Time;
}


/*------------------------------------------------------------------------------------------------------------------
============ Subrutiens ============================================================================================
----------------------------------------------------------------------------------------------------------------- */

int GetIR(int pin) { // Reads IR sensors and returns a 10 reading average
   int total = 0;
    if      (pin == 0) {pin = A0;}
    else               {pin = A1;}

    for(int i=0; i<10; i++) { int plus = analogRead(pin);
                              total= total + plus;
                             }

    int average = total/10;
    return average;
    } 

void calibrateIR () { // Calibration rutien for the IR sensors
  
  LEDstate(LOW, HIGH, LOW);
  
  int leftmax  = 0, leftmin  = 1000;
  int rightmax = 0, rightmin = 1000;
  
  
  while (millis() < 5000) {
   int right = GetIR(0);
   int left  = GetIR(2);
  
  if ( right > rightmax ) { rightmax = right; }
  if ( right < rightmin ) { rightmin = right; }

  if ( left > leftmax ) { leftmax = left;}
  if ( left < leftmin ) { leftmin = left;}
   
  }

  IRcal[0] = rightmax;
  IRcal[1] = rightmin;
  IRcal[2] = leftmax;
  IRcal[3] = leftmin;

  LEDstate(LOW, LOW, LOW);
}

void Drive ( int turn, int sped ){ // Drive forward
  
  // Wheel speed equations - Robot will left if turn is a positive number
 int wheelA = wam + turn + sped;   
 int wheelB = wbm - turn + sped;

  
  digitalWrite(dir_a, HIGH); analogWrite(pwm_a, wheelA); // Right wheel
  digitalWrite(dir_b, HIGH); analogWrite(pwm_b, wheelB); // Left Wheel

  LEDstate(HIGH, LOW, LOW);

  }

void BackUp( int turn ){ // Drive Backwards

   int wheelA = wam + turn;   
   int wheelB = wbm - turn;
 
   digitalWrite(dir_a, LOW); analogWrite(pwm_a, wheelA); // Right wheel
   digitalWrite(dir_b, LOW); analogWrite(pwm_b, wheelB); // Left Wheel

   delay(500);
 }

void Stop() { // Stops the robot
  analogWrite(pwm_a, 0);   
  analogWrite(pwm_b, 0);
             
  LEDstate(LOW, HIGH, LOW);
              
  delay(250);
}

void BUMP() { // Interupt on Pin 2 for bumper inpacts
  noInterrupts()
// pin 8 is left bumper, pin 9 is right bumper

  analogWrite(pwm_a, 0);   // Stop 
  analogWrite(pwm_b, 0);
 
  LEDstate(LOW, LOW, HIGH);
  delay(500);
  

int turn = 0;
  
 if (digitalRead(8) == HIGH && digitalRead(9) == HIGH ) { turn = 0; } // Both bumpers
 else if ( digitalRead(8) == HIGH ) { turn =  20;} // Left bumper
 else { turn = -20; } // Right bumper

 while ( digitalRead(8) == HIGH || digitalRead(9) == HIGH) { // while the bumper is depressed go back straight

        digitalWrite(dir_a, LOW); analogWrite(pwm_a, wam); // Right wheel
        digitalWrite(dir_b, LOW); analogWrite(pwm_b, wbm); // Left Wheel
       }

   digitalWrite(dir_a, LOW); analogWrite(pwm_a, (wam + turn)); // Right wheel
   digitalWrite(dir_b, LOW); analogWrite(pwm_b, (wbm - turn)); // Left Wheel

   delay(1000);

BackUp(turn);

interrupts()
}

void LEDstate(int green, int yellow, int red ) { // Opperates the LEDs

  digitalWrite(5, green);
  digitalWrite(6, yellow);
  digitalWrite(7, red);
  
}


