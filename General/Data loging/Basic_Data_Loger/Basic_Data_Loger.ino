/*
  SD card datalogger

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

#include <SPI.h>
#include <SD.h>

#define SD_FILE "Boat1.txt"

const byte chipSelect = 4;

void setup() {
  // Open serial communications and wait for port to open:
  Serial.begin(9600);
  while (!Serial) ; // wait for serial port to connect. Needed for native USB port only
  

  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) { Serial.println("Card failed, or not present");// don't do anything more:
                               return;
                              }
                              
  // Set up a header ----------------------------------------------------------------------------------------------------------
  File dataFile = SD.open(SD_FILE, FILE_WRITE);
  dataFile.println("Start Readings:\nR1 \tR2 \tR3");
  dataFile.close();

  Serial.println("card initialized.");
  Serial.println("------------------------ Set Up Done -------------------------------");
}



void loop() {

while(millis()<200) {  
                       // make a string for assembling the data to log:
                       String dataString = "";

                       // read three sensors and append to the string:
                       for (int analogPin = 0; analogPin < 3; analogPin++) { int sensor = random(10,100) ; //analogRead(analogPin);
                                                                             dataString += String(sensor);
                                                                             if (analogPin < 2) dataString += ",\t";
                                                                             }
  

                       // open the file. note that only one file can be open at a time,
                       // so you have to close this one before opening another.
                       File dataFile = SD.open(SD_FILE, FILE_WRITE);

                       // if the file is available, write to it:
                       if (dataFile) {
                                        dataFile.println(dataString);
                                        dataFile.close();
                                        // print to the serial port too:
                                        Serial.println(dataString);
                                        }
                        
                        // if the file isn't open, pop up an error:
                        else Serial.println("error opening datalog.txt");
                      }

Serial.println("\n\nReading Data File \n\n");                      

File dataFile = SD.open(SD_FILE);


if (dataFile) {  while (dataFile.available()) Serial.write(dataFile.read()); // read from the file until there's nothing else in it:
                
                dataFile.close(); // close the file:
              } 
              
else Serial.println("error opening test.txt");// if the file didn't open, print an error:

while(1);
                      
}









