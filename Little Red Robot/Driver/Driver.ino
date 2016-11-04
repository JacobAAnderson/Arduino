/*  Jacob Anderson 
 *  October 14th, 2015
 *  
 *  This code drives the Space Grant 0 robot
 *  This version has: Drives two brushed DC motoers with an H-drive shield
 *                    IR sensors perform obstacle avoidance
 *                    IR caliberation rutien at start up
 *           
 *  Future development: Intrupt on pin 2 for bumpers - Needs refinmnet
 *                      IMU
 *  
 *  ----------------------------------------------------------------------------------------------------------------
 *  Pins:   0 - Rx - Free                              A0 - Right IR sensor
 *          1 - Tx - Free                              A1 - Left IR sensor
 *          2 - Bumper interupt                        A2 - Free
 *          3 - PWMA for Right Wheel "a"               A3 - Free
 *          4 - Free                                   A4 - Free
 *          5 - Green LED                              A5 - Free
 *          6 - Yellow LED
 *          7 - Red LED
 *          8 - Right Bumper
 *          9 - Left Bumper
 *         10 - Free
 *         11 - PWWB for Left wheel "b"
 *         12 - DIRA - Direction for Right wheel 
 *         13 - DIRB - Direction for Left wheel
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
 
 rIRhigh = IRcal[1] + rightIRrange/2;
 lIRhigh = IRcal[3] + leftIRrange/2;

 rIRmid  = IRcal[1] + rightIRrange/4;
 lIRmid  = IRcal[3] + leftIRrange/4;

 rIRlow  = IRcal[1] + rightIRrange/6;
 lIRlow  = IRcal[3] + leftIRrange/6;

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


  
  if (rightIR > rIRhigh && leftIR > lIRhigh ) { // Both IR sensors See somthing close --> Backup and Pivot
    
    Stop();

    if ( rightIR = leftIR ) { turn = 0; }
    else if ( rightIR < leftIR ) { turn = 10; } // Turn right
    else { turn = -10; }  // left

  while ( rightIR > rIRhigh || leftIR > lIRhigh ) {

        rightIR = GetIR(0);
        leftIR  = GetIR(2);
    
        BackUp( turn );
      }
  
  Pivot( turn );
 }

else if ( rightIR > rIRhigh || leftIR > lIRhigh ) { // One IR sensor sees somthing closs --> Turn Sharp

  turn = rightIR - leftIR;
  turn = map(turn, -100, 100, -20, 20);
  sped = 0;
  
  Drive( turn, sped );
  
}

else if (rightIR > rIRmid || leftIR > lIRmid) { // IR sensor sees somthing mid range --> Turn, Speed up or slow down depending on readings
  turn = rightIR - leftIR;
  turn = map(turn, -100, 100, -15, 15);

  sped = 200 - (leftIR + rightIR)/2 ;
  sped = constrain( sped, 0, 25);

  Drive( turn, sped );
 }
                                      
else if ( rightIR > rIRlow && leftIR > lIRlow ) { // Ir sensor sees somthing far --> Drive straight, Increes speed
  turn = 0;
  sped = 100 - (leftIR + rightIR)/2 ;
  sped = constrain( sped, 5, 50);
  Drive( turn, sped );
 }

else { // IR sesnsors dont see anything --> Drive straing, Go faster
  turn = 0;
  sped = 100 - (leftIR + rightIR)/2 ;
  sped = constrain( sped, 0, 100);
  Drive( turn, sped );
  
 }

}


/*------------------------------------------------------------------------------------------------------------------
============ Subrutiens ============================================================================================
----------------------------------------------------------------------------------------------------------------- */


void Drive ( int turn, int sped ){ // Drive forward --> Green LED
  
  // Wheel speed equations - Robot will left if turn is a positive number
 int wheelA = wam + turn + sped;   
 int wheelB = wbm - turn + sped;

  
  digitalWrite(dir_a, HIGH); analogWrite(pwm_a, wheelA); // Right wheel
  digitalWrite(dir_b, HIGH); analogWrite(pwm_b, wheelB); // Left Wheel

  LEDstate(HIGH, LOW, LOW);

  }

void BackUp( int turn ){ // Drive Backwards --> Yellow LED

   int wheelA = wam + turn;   
   int wheelB = wbm - turn;
 
   digitalWrite(dir_a, LOW); analogWrite(pwm_a, wheelA); // Right wheel
   digitalWrite(dir_b, LOW); analogWrite(pwm_b, wheelB); // Left Wheel

  LEDstate(LOW, HIGH, LOW);
 }

void Stop() { // Stops the robot, 1s delay --> No LED
  analogWrite(pwm_a, 0);   
  analogWrite(pwm_b, 0);
             
  LEDstate(LOW, LOW, LOW);
              
  delay(1000);
}

void Pivot( int piv) { // Pivots the robot - 2s delay --> Green and Yellow LEDs

LEDstate(HIGH, HIGH, LOW);

  int stateA = LOW;
  int stateB = LOW;
  
  if (piv >= 0 ) stateA = HIGH;  // pivot Clock wise
  else stateB = HIGH;            // pivot counter clock wise
  
  
   digitalWrite(dir_a, stateA ); analogWrite(pwm_a, wam); // Right wheel
   digitalWrite(dir_b, stateB ); analogWrite(pwm_b, wbm); // Left Wheel

   delay(900);
  
}



void BUMP() { // Interupt on Pin 2 for bumper inpacts, 10.5s delay --> Red LED
//  detachInterrupt(digitalPinToInterrupt(2));
// pin 8 is left bumper, pin 9 is right bumper

  analogWrite(pwm_a, 0);   // Stop 
  analogWrite(pwm_b, 0);
 
  LEDstate(LOW, LOW, HIGH);
  

int turn = 0;
  
 if (digitalRead(8) == HIGH && digitalRead(9) == HIGH ) { turn = 0; } // Both bumpers
 else if ( digitalRead(8) == HIGH ) { turn =  20;}  // Left bumper
 else if ( digitalRead(9) == HIGH ) { turn = -20;}  // right Bumper
 else { turn = 0; } //  Bumper no longer depressed

 while ( digitalRead(8) == HIGH || digitalRead(9) == HIGH) { // while the bumper is depressed go back straight

        LEDstate(LOW, HIGH, HIGH);
        
        digitalWrite(dir_a, LOW); analogWrite(pwm_a, wam); // Right wheel
        digitalWrite(dir_b, LOW); analogWrite(pwm_b, wbm); // Left Wheel
       }

  LEDstate(HIGH, LOW, HIGH);

   digitalWrite(dir_a, LOW); analogWrite(pwm_a, (wam + turn)); // Right wheel
   digitalWrite(dir_b, LOW); analogWrite(pwm_b, (wbm - turn)); // Left Wheel

 

  if (turn > 0 ) {// Pivot right
    digitalWrite(dir_a, HIGH); analogWrite(pwm_a, wam ); // Right wheel
    digitalWrite(dir_b, LOW); analogWrite(pwm_b, wbm ); // Left Wheel
  }

  else { // Pivot left
    digitalWrite(dir_a, LOW); analogWrite(pwm_a, wam + turn); // Right wheel
    digitalWrite(dir_b, HIGH); analogWrite(pwm_b, wbm - turn); // Left Wheel
  }
 


// attachInterrupt(digitalPinToInterrupt(2), BUMP, LOW);
}




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

void calibrateIR () { // Calibration rutien for the IR sensors --> Yellow LED at start up
  
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

void LEDstate(int green, int yellow, int red ) { // Opperates the LEDs

  digitalWrite(5, green);
  digitalWrite(6, yellow);
  digitalWrite(7, red);
  
}


