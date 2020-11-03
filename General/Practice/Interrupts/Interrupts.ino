
volatile int x = 0;

void setup() {
  Serial.begin(9600);
  attachInterrupt( 0, Interupt_1, LOW );
  pinMode(2, INPUT);

}

void loop() {
 
  while ( x < 5000 ) { 
                        Serial.println(x);
                        x++;
                        delay(100);   
                      }
  
  x = 0;
  
  
}




void Interupt_1() { Serial.println("You Pushed My Button!!!!!!!");
                    
                    while( !digitalRead(2)  ) { Serial.println(x);
                                                x--;
                                                delay(100);
                                               }
                  
                   }
