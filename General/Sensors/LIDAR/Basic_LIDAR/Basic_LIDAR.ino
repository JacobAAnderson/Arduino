/* 
http://pulsedlight3d.com
This sketch demonstrates getting distance with the LIDAR-Lite Sensor
It utilizes the 'Arduino I2C Master Library' from DSS Circuits:
http://www.dsscircuits.com/index.php/articles/66-arduino-i2c-master-library 
You can find more information about installing libraries here:
http://arduino.cc/en/Guide/Libraries
*/

#include <I2C.h>
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.


void setup(){
  Serial.begin(9600); //Opens serial connection at 9600bps.     
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails
}

void loop(){ Serial.println("top");
  // Write 0x04 to register 0x00
  uint8_t nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.write(LIDARLite_ADDRESS,RegisterMeasure, MeasureValue); // Write 0x04 to 0x00
    delay(1); // Wait 1 ms to prevent overpolling
  }

  byte distanceArray[2]; // array to store distance bytes from read function
  
  // Read 2byte distance from register 0x8f
  nackack = 100; // Setup variable to hold ACK/NACK resopnses     
  while (nackack != 0){ // While NACK keep going (i.e. continue polling until sucess message (ACK) is received )
    nackack = I2c.read(LIDARLite_ADDRESS,RegisterHighLowB, 2, distanceArray); // Read 2 Bytes from LIDAR-Lite Address and store in array
    delay(1); // Wait 1 ms to prevent overpolling
  }
  int distance = (distanceArray[0] << 8) + distanceArray[1];  // Shift high byte [0] 8 to the left and add low byte [1] to create 16-bit int
  
  // Print Distance
  Serial.println(distance);
  delay(100);
}

/* LIDAR distance info:
If the LIDAR does not get a retun then it will register a reading of 0
The largest observed reading in 4400
Does not work well for distances less than 10cm


Some Average Distance Readings
Out Side:
  Distance    : Sunny Textured surface     Sunny Smooth Black surface      Smooth surface in the shade
  ------------:----------------------------------------------------------------------------------------
  12ft, 366cm :       345                    12ft - 360                     12ft - 361
   6ft, 183cm :       178                     6ft - 175                      6ft - 177
   4ft, 122cm :       120                     4ft - 114                      4ft - 122
   3ft,  91cm :        92                     3ft -  90                      3ft -  90
   2ft,  61cm :        60                     2ft -  57                      2ft -  60
   1ft,  30cm :        30                     1ft -  35                      1ft -  30
  
In Side: 
  Distance    :  Flat surface        Same flat surface 
                                    at a 30 degree angle
  -----------------------------------------------------------------
  12ft, 366cm :        365                  330
   6ft, 183cm :        183                  185
   4ft, 122cm :        125                  125
   3ft,  91cm :         92                   98
   2ft,  61cm :         60                   63
   1ft,  30cm :         31                   32

*/
