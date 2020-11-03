/* This is a code that writen for project # 5 of the ARDX starter kit.
   This code drives a serial to paralele register. The register is used to light up LEDs.
   This code also utalizes a random number generator to produce random number for the register.
*/


int data = 2;
int clock = 3;
int latch = 4;

void setup(){ Serial.begin(9600);

 pinMode(data, OUTPUT);
 pinMode(clock, OUTPUT);
 pinMode(latch, OUTPUT);
}
 
void loop() { Serial.println(" Starting Loop");
  
  int delayTime = 500;
 /* 
  // Count up form 0 to 255
  for(int i=0; i<256; i++){ upDateLEDs(i); delay(delayTime); Serial.println(i);} 
  
  // Count down from 255 to 0
  for(int i=255; i>0; i--){ upDateLEDs(i); delay(delayTime); Serial.println(i);} 
*/

 // Generate a random number
 int i = random(0,255); 
 upDateLEDs(i), delay(delayTime), Serial.println(i);

  Serial.print(""); 
 }
 
 void upDateLEDs (int value){
   digitalWrite(latch, LOW);
   
   shiftOut(data, clock, MSBFIRST, value);
   
   digitalWrite(latch, HIGH);
   
 }
