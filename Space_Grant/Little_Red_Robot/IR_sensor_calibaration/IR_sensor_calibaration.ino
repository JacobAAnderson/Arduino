/*  This Code is designed to read two IR sensors. 
 *   It includeds a calibration rutien
 *   The sensor reading are averaged over a range of 10 readings
 */


int IRcal[] = {0, 0, 0, 0};

void setup() { Serial.begin(9600);

pinMode(A0, INPUT);
pinMode(A1, INPUT);

calibrateIR();
Serial.print("IR calibration: ");
for (int i=0; i<4; i++) { Serial.print(IRcal[i]), Serial.print(" , ");}

Serial.println("\n----------------- Setup Done -----------------\n");
}

void loop() {
  
  Serial.println("");
  Serial.println("Get IR Data --------------------------");
  Serial.println("");
  
  Serial.println("Get Right IR");
  int rightIR = GetIR(0);
  Serial.print("Right IR: "), Serial.println(rightIR), Serial.println("");
  
  Serial.println("Get Left IR");
  int leftIR = GetIR(2);
  Serial.print("left IR: "), Serial.println(leftIR), Serial.print("");
  delay(5000);
  
}

int GetIR(int pin){ int total = 0;

                    if      (pin == 0) {pin = A0;}
                    else               {pin = A1;}

                    for(int i=0; i<10; i++) { int plus = analogRead(pin);
                                              total= total + plus;
                                             }

                    int average = total/10;
                    return average;
                   }

void calibrateIR () { // Calibration rutien for the IR sensors
  Serial.println("Calibrating IR sensors\n");
  
  int leftmax  = 0, leftmin  = 1000;
  int rightmax = 0, rightmin = 1000;
  
  
  while (millis() < 4000) {
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
}



                   
