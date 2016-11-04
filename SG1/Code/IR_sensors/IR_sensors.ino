

void setup() { Serial.begin(9600);

pinMode(A0, INPUT);
pinMode(A1, INPUT);
pinMode(A2, INPUT);



}

void loop() {
  
  Serial.println("");
  Serial.println("Get IR Data --------------------------");
  Serial.println("");
  
  Serial.println("Get Right IR");
  delay(500); 
  int rightIR = GetIR(0);
  Serial.print("Right IR: "), Serial.println(rightIR), Serial.println("");
  delay(500);
  
  Serial.println("Get Center IR");
  delay(500);
  int centerIR = GetIR(1);
  Serial.print("Center IR: "), Serial.println(centerIR), Serial.println("");
  delay(500);
  
  Serial.println("Get Left IR");
  delay(500);
  int leftIR = GetIR(2);
  Serial.print("left IR: "), Serial.println(leftIR), Serial.print("");
  delay(1000);
  
}

int GetIR(int pin){ int total = 0;

                    if      (pin == 0) {pin = A2;}
                    else if (pin == 1) {pin = A1;}
                    else               {pin = A0;}
                    
                    
                    for(int i=0; i<10; i++) { int plus = analogRead(pin);
                                              Serial.println(plus); 
                                              total= total + plus;
                                             }
              
                    int average = total/10;
                    Serial.print("Average: "), Serial.println(average);
                    return average;
                   }
