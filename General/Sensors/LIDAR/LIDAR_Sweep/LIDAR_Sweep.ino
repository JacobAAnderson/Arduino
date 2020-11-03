// Libraries
#include <Servo.h>; // Servo library
#include <I2C.h>    // I2C library

// Defined Values
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

// Name Servos:
Servo Lidar;

// Global Variables;
int LIDARmin;           // Minimum reading from LIDAR
int LIDARangle = 0;     // Angle of the minimum value from center the value to the minimum value of the vector

int MaxBinAngle;        // Angle coresponding to Max average bin
int MinBinAngle;        // Angle coresponding to Min average bin

// Vector variables 
const int LIDARvec  = 21;      // Number of elements in LIDARdistances vector
const int binNumber = 19;      // Number of elements in LIDARbin vector

// Vectors
int LIDARdistances[LIDARvec];     // Vector to store LIDAR measurments
int LIDARAngles [ LIDARvec ];  // Vector of Sweep angles coresponding to the distance readings
int LIDARbins[binNumber];         // Vector for LIDAR averaging bins




/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
==== Set up routien ================================================================================================================================================================
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup(){ Serial.begin(9600);  
              Serial.println("");
              Serial.println("-------------------------------------------------");
  
  // Set up I2C ---------------------------------------------------------------------------------------------------------------
    
  I2c.begin();      // Opens & joins the irc bus as master
  delay(100);       // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50);  // Sets a timeout to ensure no locking up of sketch if I2C communication fails

  // Set up servos ------------------------------------------------------------------------------------------------------------
  Lidar.attach(6); // Attache Lidar Servo to pin 6
  Lidar.write(85);  // Centers Lidar detector
  delay (10);
  
  // Set up LIDAR vecor -------------------------------------------------------------------------------------------------------
  for (int zero = 0; zero < LIDARvec; zero++) {LIDARdistances[zero] = 0;}
  PrintLIDARvector(); 
  
  
  // Set up LIDAR Angles vector -----------------------------------------------------------------------------------------------
  int a = 0;
  for (int angle = -50; angle <= 50; angle += 5) { LIDARAngles[a] = angle; a++;}    // + angle is to the left, - angle is to the right
  
  Serial.print(" Angle Vector: [ ");
  for (int i=0;i<(LIDARvec-1);i++) { Serial.print(LIDARAngles[i]), Serial.print(", ");}
  Serial.print(LIDARAngles[(LIDARvec-1)]), Serial.println(" ]");
                                    

  Serial.println("");
  Serial.println("----------- Set up done -------------");
  Serial.println("");
  
}





/*---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
======= Main Loop =================================================================================================================================================================
---------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/


void loop(){ 
    
  LIDARsweep();
    
  // Print Distance
  PrintLIDARvector();
  LIDARanalysis();
  LIDARbinAnalysis();
  
 delay (1000);
}





/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
======== Subrutiens ================================================================================================================================================================
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

//---- Sweep LIDAR -------------------------------------------------------------------------------------------------------------------------------------------------
void LIDARsweep(){ // LIDAR servo is far right at 0 and far left at 180
  
                   Serial.println("LIDAR is scanning...");
                   int a =0;
                   int angle = -50;
                   for ( int SweepAngle=35; SweepAngle < 140 ; SweepAngle += 5 ) { Lidar. write(SweepAngle); 
                                                                                   delay(100);
    
                                                                                   LIDARdistances[a] = random(1,300); // GetDistance();
    
                                                                                   delay (100);
                                                                   
                                                                                   Serial.print ("Sweep Angle: "),      Serial.print(SweepAngle);
                                                                                   Serial.print ("   \tLIDAR Angle: "), Serial.print(angle);
                                                                                   Serial.print ("   \tDistance: " ),   Serial.println(LIDARdistances[a]);
                                                                   
                                                                                   a++;
                                                                                   angle += 5; 
                                                                                  }
                   Lidar.write(85);
                   Serial.println("");
   
                  }



//---- Get distance from LIDAR ---------------------------------------------------------------------------------------------------------------------------------------
int GetDistance() {
  
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
 
                    if (distance == 0) { distance = 3000;}
                    else               { distance = distance;}
 
                    return distance;
                  }
 
 
  
//---- Print Lidar Vector --------------------------------------------------------------------------------------------------------------------------------------------
void PrintLIDARvector(){ Serial.print("LIDAR Vector: [ ");
                         for (int i=0;i<(LIDARvec-1);i++) { Serial.print(LIDARdistances[i]), Serial.print(", ");}
                         Serial.print(LIDARdistances[(LIDARvec-1)]), Serial.println(" ]");
                        }




//---- Analyse Lidar data for minimum value and its angle -----------------------------------------------------------------------------------------------------------
void LIDARanalysis(){
                      int place;
                      LIDARmin = 10000;
  
                      // Find the minimum Value in vector
                      for(int a =0; a < LIDARvec; a++){ if( LIDARdistances[a] < LIDARmin) { LIDARmin = LIDARdistances[a]; place = a;} } 
  
                      LIDARangle = LIDARAngles[place];    // Angle that coresponds to the minimum value
  
                      // Display the calculated values
                       Serial.print("Vector Min: "), Serial.print(LIDARmin);
                       Serial.print("\t Angle: "),   Serial.println(LIDARangle);
                       Serial.println(""); // Empty line
                      }
                      
                      
                      
//---- LIDAR bin  analysis ----------------------------------------------------------------------------------------------------------------------------------------
/* Takes the readings in the LIDAR vector and averages 3 readings togeter into a bin.
   The bins over lap by two element and corespond to angles conterclock wise around the
   front of the robot starting far right.
*/
void LIDARbinAnalysis(){
                         int MaxValue = 0;        // Set LIDAR Max and min values to quatantied that will be exceeded by actual reading
                         int MinValue = 10000;
                         int MaxBin = 0;             // Bin number of Max LIDAR averaging bin
                         int MinBin = 0;             // Bin number of Min LIDAR averaging bin


 
                         // Calculate LIDAR bins
                         int angle = -45;
                         for(int bin = 0; bin < binNumber; bin++){ LIDARbins[bin]= (LIDARdistances[bin]+LIDARdistances[(bin+1)]+LIDARdistances[(bin+2)])/3;  
                                                                   
                                                                   Serial.print ("Bin: "),     Serial.print (bin);
                                                                   Serial.print ("\tAngle: "), Serial.print(angle);
                                                                   Serial.print ("\tValue: "), Serial.println (LIDARbins[bin]);
                                                                  
                                                                   angle += 5; 
                                                                  }
                         Serial.println("");

 
                         // Find minimum bin value and corespondind bin number
                         for(int bin = 0; bin < binNumber; bin++) { if(MinValue > LIDARbins[bin]) {MinValue = LIDARbins[bin]; MinBin = bin;}
                                                                    if(MaxValue < LIDARbins[bin]) {MaxValue = LIDARbins[bin]; MaxBin = bin;}
                                                                   }
 
                         // Assign anblges to the bin places    + Angles are left of center, - Angles are right of center
                         MinBinAngle = LIDARAngles[ MinBin+1 ];
                         MaxBinAngle = LIDARAngles[ MaxBin+1 ];
 
                         // Dispaly values
                         Serial.print("Minimum bin value: "); Serial.print(MinValue); Serial.print("\t Bin#: "); Serial.print (MinBin); Serial.print("\t Angle: "); Serial.println(MinBinAngle);
                         Serial.print("Maximum bin value: "); Serial.print(MaxValue); Serial.print("\t Bin#: "); Serial.print (MaxBin); Serial.print("\t Angle: "); Serial.println(MaxBinAngle);
                         Serial.println(""); // Blank Line
                        }
