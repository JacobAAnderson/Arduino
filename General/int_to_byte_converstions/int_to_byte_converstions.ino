


void setup() {Serial.begin(9600);


int myDelay = 22000;  // this is for milliseconds
byte high = highByte(myDelay);
byte low = lowByte(myDelay);





byte myTwoBytes[2] = { high , low };

 int val = word( myTwoBytes[0], myTwoBytes[1]);
 
 Serial.println(val);
 Serial.print( highByte(val));
 Serial.print( ",");
 Serial.println(lowByte(val));
  

}

void loop() {



  


}
