/* Jacob Anderson
 *  Jan 20th, 2016
 *  This is a practice scrip to read the temperature from a TMP36 temperature sensor
 */

int tempPIN = A0;

void setup() {
  Serial.begin(9600);

  pinMode(tempPIN, INPUT);
  
}

void loop() {

  float Temperature = (analogRead(tempPIN) * 5.0 / 1024.0 - 0.5)*100.0;

  Serial.println(Temperature);
  delay(1000);
  
}
