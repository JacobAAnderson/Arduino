#include <Wire.h>

#define LED_PIN 13
byte x = 0;

void setup(){ Serial.begin(9600); Serial.println("Starting");
  
  Wire.begin(); // Start I2C Bus as Master
//  pinMode(LED_PIN, OUTPUT);
//  digitalWrite(LED_PIN, LOW);

}
void loop() { Serial.println("loop");
  Serial.println(x);
  Wire.beginTransmission(9); // transmit to device #9
  Wire.write(x);              // sends x 
  Wire.endTransmission();    // stop transmitting
  x++;
  if (x > 5) x=0;
  delay(450);
}
