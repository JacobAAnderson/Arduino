/*  Jacob Anderson
 *  Oct 12th, 2015
 *   
 *  This code drives the Space Grant 0 robot
 *  It uses IR sensors and an inturuped on pin 2 for the bumpers to perform obstacle avoidance
 *  
 *  ----------------------------------------------------------------------------------------------------------------
 *  Pins:  2 - Bumper interupt                        A0 - Right IR sensor
 *         3 - PWM for Right Wheel "a"                A1 - Left IR sensor
 *         5 - Green LED
 *         6 - Yellow LED
 *         7 - Red LED
 *         8 - Right Bumper
 *         9 - Left Bumper
 */

  // Motor shield configuration
  int pwm_a = 3;    // Right Wheel speed
  int pwm_b = 11;   // Left wheel speed
  int dir_a = 12;   // Right wheel direction
  int dir_b = 13;   // Left wheel direction
  int wam = 90;
  int wbm = 110;


  int hit = 0;
  
 

void setup() { Serial.begin(9600);  

// I/O Setup -----------------------------------------------------------------------------------------------------------

  attachInterrupt(digitalPinToInterrupt(2), BUMP, LOW); // Bumper interupt
 
  pinMode(A0, INPUT); // IR sensor pins
  pinMode(A1, INPUT);

  pinMode(pwm_a, OUTPUT); // Set control pins for the wheels
  pinMode(pwm_b, OUTPUT);
  pinMode(dir_a, OUTPUT);
  pinMode(dir_b, OUTPUT);

  pinMode(5, OUTPUT);  // LED pins
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);

  pinMode(8, INPUT);   // Left Bumper
  pinMode(9, INPUT);   // Right Bumper
  
  
// Set both moters to 0 - stop -------------------------------------------------------------------------------------
  analogWrite(pwm_a, 0);   
  analogWrite(pwm_b, 0);

  LEDstate(LOW, LOW, LOW);
  LEDstate(HIGH, HIGH, HIGH);

  delay(500);

  LEDstate(LOW, LOW, LOW);
  
  }

void loop() {
  
  int rightIR = GetIR(0)*1.1;
  int leftIR  = GetIR(2); 
  int turn = 0;
  int sped = 0;


switch (hit) { case 1: BackUp( turn );
               default: break;  
              }
  
  
  if (rightIR > 300 && leftIR > 300 ) { 
    Stop();

    if ( rightIR = leftIR ) { turn = 0; }
    else if ( rightIR < leftIR ) { turn = 10; }
    else { turn = -10; }

    BackUp( turn );
 }


else if (rightIR > 100 || leftIR > 100) { 
  turn = rightIR - leftIR;
  turn = map(turn, -100, 100, -10, 10);

  sped = 200 - (leftIR + rightIR)/2 ;
  sped = constrain( sped, -5, 25);

  Drive( turn, sped );
 }
                                      
else if ( rightIR > 50 && leftIR > 50 ) { 
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

hit = 0;
  
}


/*------------------------------------------------------------------------------------------------------------------
============ Subrutiens ============================================================================================
----------------------------------------------------------------------------------------------------------------- */

int GetIR(int pin){ // Reads IR sensors and returns a 10 reading average
   int total = 0;
    if      (pin == 0) {pin = A0;}
    else               {pin = A1;}

    for(int i=0; i<10; i++) { int plus = analogRead(pin);
                              total= total + plus;
                             }

    int average = total/10;
    return average;
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

  hit = 1;

}

void LEDstate(int green, int yellow, int red ) { // Opperates the LEDs

  digitalWrite(5, green);
  digitalWrite(6, yellow);
  digitalWrite(7, red);
  
}


