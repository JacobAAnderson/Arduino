
#include <SPI.h>
#include <SD.h>
#include <Adafruit_GPS.h>

#define chipSelect 4
#define SD_File "GPS_log.txt"


HardwareSerial mySerial = Serial1;
Adafruit_GPS GPS(&Serial1);


void setup()  { Serial.begin(115200);

  while (!Serial);

  
  // Set Up GPS -------------------------------------------------------------------------------------------
  GPS.begin(9600);
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
  GPS.sendCommand(PGCMD_ANTENNA);  // Request updates on antenna status, comment out to keep quiet
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);


  // Set Up SD card -------------------------------------------------------------------------------------
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) { Serial.println("Card failed, or not present");
                               return;
                              }
  Serial.println("card initialized.");

  Serial.println("Waiting for GPS Fix");
  while(!GPS.fix) GPS.parse(GPS.lastNMEA());
  
  String data = "\n\n";
  data += String(GPS.month);
  data += "/";
  data += String(GPS.day);
  data += "/";
  data += String(GPS.year);
  data += "\nTime \tGPS \tGPS fix \t\tGPS Lat \tGPS Lon";

  File dataFile = SD.open(SD_File, FILE_WRITE);
  dataFile.println(data);
  //dataFile.println("Time \tGPS \tGPS fix \t\tGPS Lat \tGPS Lon");
  dataFile.close();
 

  Serial.println("-------------  Set Up Done  -----------------\n\n");
}


// Interrupt is called once a millisecond, looks for any new GPS data, and stores it
SIGNAL(TIMER0_COMPA_vect) { char c = GPS.read();}



uint32_t timer = millis();

void loop() {

  // if a sentence is received, we can check the checksum, parse it...
  if (GPS.newNMEAreceived()) { if (!GPS.parse(GPS.lastNMEA()))  return;   }

  // if millis() or timer wraps around, we'll just reset it
  if (timer > millis())  timer = millis();

  // approximately every 2 seconds or so, print out the current stats
  if (millis() - timer > 2000) { 
    
    timer = millis(); // reset the timer
    
    Serial.print("\nTime: ");
    Serial.print(GPS.hour, DEC); Serial.print(':');
    Serial.print(GPS.minute, DEC); Serial.print(':');
    Serial.print(GPS.seconds, DEC); Serial.print('.');
    Serial.println(GPS.milliseconds);
    Serial.print("Date: ");
    Serial.print(GPS.day, DEC); Serial.print('/');
    Serial.print(GPS.month, DEC); Serial.print("/20");
    Serial.println(GPS.year, DEC);
    Serial.print("Fix: "); Serial.print((int)GPS.fix);
    Serial.print(" quality: "); Serial.println((int)GPS.fixquality);
    
    if (GPS.fix) { Serial.print("Location: ");
                   Serial.print(GPS.latitudeDegrees, 6);
                   Serial.print(", "); 
                   Serial.println(GPS.longitudeDegrees, 6);
      
                   Serial.print("Speed (knots): "); Serial.println(GPS.speed);
                   Serial.print("Angle: "); Serial.println(GPS.angle);
                   Serial.print("Altitude: "); Serial.println(GPS.altitude);
                   Serial.print("Satellites: "); Serial.println((int)GPS.satellites);

                   String data = "";

                   data += String(GPS.hour);
                   data += ":";
                   data += String(GPS.minute);
                   data += ":";
                   data += String(GPS.seconds);
                   data += "  \t";
                   
                   if (GPS.fix) data += "Fix";
                   else data += "No Fix";
                   
                   data += "\t";
                   data += String(GPS.fixquality);
                   data += "\t\t";
                   data += String(GPS.latitudeDegrees,8);
                   data += "\t";
                   data += String(GPS.longitudeDegrees,8);

                   File dataFile = SD.open(SD_File, FILE_WRITE);
                   dataFile.println(data);
                   dataFile.close();
                   
                  }
     }

     while (millis() > 10000) ;
}
