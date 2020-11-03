/*
  SD card basic file example

 This example shows how to create and destroy an SD card file
 The circuit:
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created   Nov 2010
 by David A. Mellis
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */
#include <SPI.h>
#include <SD.h>

#define File_to_Remove "Boat1.txt"

// 



void setup() { Serial.begin(9600);

  while (!Serial);

  Serial.print("Initializing SD card...");

  if (!SD.begin(4)) { Serial.println("initialization failed!");
                      return;
                     }
                     
  Serial.println("initialization done.");

  if (SD.exists(File_to_Remove)) Serial.println("File Exists");
  else  Serial.println("File doesn't exist.");

  

  // delete the file:
  Serial.println("Removing example.txt...");
  SD.remove(File_to_Remove);

  if (SD.exists(File_to_Remove)) Serial.println("File removal was unsuccesfull");
  else Serial.println("File removeal was succesfull!!!");

}

void loop() {
  
}



