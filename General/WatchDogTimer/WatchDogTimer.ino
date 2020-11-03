/*
Watchdog Timer Basic Example 
10 June 2011
Nicolas Larsen
*/

#include <avr/wdt.h> 

int loop_count = 0;

void setup() {
Serial.begin(9600); 
Serial.println("Starting up..."); 

pinMode(13,OUTPUT); 
digitalWrite(13,HIGH);

delay (500);

watchdogSetup(); 
}

void watchdogSetup(void){
  
cli();        // disable all interrupts 
wdt_reset();  // reset the WDT timer 

/*
WDTCSR configuration:
WDIE = 1: Interrupt Enable 
WDE = 1 :Reset Enable
WDP3 = 0 :For 2000ms Time-out 
WDP2 = 1 :For 2000ms Time-out 
WDP1 = 1 :For 2000ms Time-out 
WDP0 = 1 :For 2000ms Time-out
*/

// Enter Watchdog Configuration mode: 
WDTCSR |= B00011000;
// Set Watchdog settings:
WDTCSR = B01101001;

sei(); // Enable all interrupts

}

void loop(){
  
  for (int i = 0; i <= loop_count;i++){  digitalWrite(13,HIGH); 
                                         delay(100); 
                                         digitalWrite(13,LOW); 
                                         delay(100);
                                        }

  loop_count++;

  wdt_reset();

  Serial.print(loop_count);
  Serial.print(". Watchdog fed in approx. "); 
  Serial.print(loop_count*200); 
  Serial.println(" milliseconds.");

}

// Watchdog timer interrupt.
ISR(WDT_vect) {  // Include your code here - be careful not to use functions they may cause the interrupt to hang and // prevent a reset.

   Serial.println("Woof - Woof!!!"); // Do not use Serial.print statement in real code!!!

}




