/*  File to create a space betwenn enetries in a text file on an SD card

 Chip Select - pin 4

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
  dataFile.println("\n\n--------------------------------------------------------------------------------------------------------------------------------\n\n");
  dataFile.close();

  Serial.println("card initialized.");
  Serial.println("------------------------ Done -------------------------------");
}


void loop() {}


