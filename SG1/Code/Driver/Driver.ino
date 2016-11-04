/*  Basic driving code for Space Grant Robot 1
    Jacob Anderson 8/31/2015
    
    Notes: -------------------------------------------------------------------
     Servo write from 0 to 180, 0  to  90 is counter clock wise, 90 to 180 is clockwise
     Wheel A  Back  left  wheel:  Forward < 90
     Wheel B  Back  right wheel:  Forward > 90
     Wheel C  Front left  wheel:  Forward < 90
     Wheel D  Front right whell:  Forward > 90
     
     Turning radious: + Radius will turn left, - Radius will turn right
     Lidar angles are: -50 at 50 degrees right of center 0 when centered and 50 at 50 degrees left of center
     
     
*/

// Libraries -------------------------------------------------------------------------------------------
#include <Servo.h>; // Servo library
#include <I2C.h>    // I2C library

// Defined Values -------------------------------------------------------------------------------------
#define    HMC5883L            0x1E          // Compass Address
#define    LIDARLite_ADDRESS   0x62          // Default I2C Address of LIDAR-Lite.
#define    RegisterMeasure     0x00          // Register to write to initiate ranging.
#define    MeasureValue        0x04          // Value to initiate ranging.
#define    RegisterHighLowB    0x8f          // Register to get both High and Low bytes in 1 call.

// Name Servos ----------------------------------------------------------------------------------------
Servo A;     // Wheel A  Back left wheel
Servo B;     // Wheel B  Back right wheel
Servo C;     // Wheel C  Front left wheel
Servo D;     // Wheel D  Front right whell
Servo Lidar; // Servo that positions LIDAR

// Global Variables --------------------------------------------------------------------------------------------
int LIDARmin;           // Minimum reading from LIDAR
int LIDARangle = 0;     // Angle of the minimum value from center the value to the minimum value of the vector

int MaxBinAngle;        // Angle coresponding to Max average bin
int MinBinAngle;        // Angle coresponding to Min average bin

int LSM =  82;   // Median left  side wheel speed. Left  wheels:  Forward < 90
int RSM = 103;   // Median right side wheel speed. Right wheels:  Forward > 90

// Vector variables -------------------------------------------------------------------------------------------
const int LIDARvec  = 21;      // Number of elements in LIDARdistances vector
const int binNumber = 19;      // Number of elements in LIDARbin vector

// Vectors --------------------------------------------------------------------------------------------------
int LIDARdistances[LIDARvec];  // Vector to store LIDAR measurments
int LIDARAngles [ LIDARvec ];  // Vector of Sweep angles coresponding to the distance readings
int LIDARbins[binNumber];      // Vector for LIDAR averaging bins



/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
==== Set up routien ================================================================================================================================================================
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/

void setup(){ Serial.begin(9600);  
              Serial.println("");
              Serial.println("------------ Start Setup -------------------");
  
  // Set up I2C ---------------------------------------------------------------------------------------------------------------
  Serial.println("Start I2C");  
  I2c.begin();      // Opens & joins the irc bus as master
  //I2c.write(HMC5883L,0x02,0x00);  // Configure device for continuous mode
  delay(100);       // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50);  // Sets a timeout to ensure no locking up of sketch if I2C communication fails

  // Pin Modes ----------------------------------------------------------------------------------------------------------------
  Serial.println("Set pin modes");
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);

  // Set up servos ------------------------------------------------------------------------------------------------------------
  Serial.println("Set up servos");
  A. attach(2);      // Attach Back  left  wheel to pin 2
  B. attach(3);      // Attach Back  right wheel to pin 3
  C. attach(4);      // Attach Front left  wheel to pin 4
  D. attach(5);      // Attach Front right wheel to pin 5
  Lidar. attach(6);  // Attach LIDAR Servo to pin 6
  
  // Stop driving motors 
  A. write(90);   
  B. write(90);
  C. write(90);
  D. write(90);
  Lidar.write(85); // Center Lidar Servo, Lidar centers with an input of 85, 90 is a little off center.

  
  // Set up LIDAR vecor -------------------------------------------------------------------------------------------------------
  Serial.print("Set up LIDAR");
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


void loop(){ Serial.println("------------------------------------------------------------------");
   
 // Collect sensor data ------------------------------------------------------------------------------------------------------------------------ 
  LIDARsweep();
  //  **PrintLIDARvector();
  LIDARanalysis();
  //  **LIDARbinAnalysis();
    
  int leftIR   = GetIR(0);  Serial.print("Left IR: "),     Serial.print(leftIR);
  int centerIR = GetIR(1);  Serial.print("\tCenter IR: "), Serial.print(centerIR);
  int rightIR  = GetIR(2);  Serial.print("\tRight IR: "),  Serial.println(rightIR);
  
  Serial.println("");  
  
  
 // Desition making hierarchy -----------------------------------------------------------------------------------------------------------------
 
 int turnAngle = 0;
 // Avoid a hole with the IR sensors
 if ( leftIR < 3 || centerIR < 3 || rightIR < 3 ) { Serial.print("Avoid a Hole");
                                                    Stop(); 
                                                    DriveBackwards();
   
                                                    if ( leftIR < 3 && centerIR < 3 && rightIR < 3 ||centerIR < 3 ) { if ( random(0,10) > 5 ) {turnAngle = 7;}
                                                                                                                      else  {turnAngle = -7;}
                                                                                                                     }
                                                    
                                                    else if (leftIR < 3) { turnAngle = -7;}
                                                    else {turnAngle = 7;}
                                                   }
 
 // Avoid and Obsticle with LIDAR
 else if ( LIDARmin < 10 ) { Serial.println("Avoid an Obsticle");
 
                             if (LIDARangle > 0) { turnAngle = map(LIDARangle, 0,  50, -7, 0); }
                             else                { turnAngle = map(LIDARangle, 0, -50,  7, 0); }
                            
                            }
                                                       
 
 // Return to origonal path if the becaon reading if off of the initial reading by more that 10
 else if (random(0,10) > 7) { Serial.println("return to origonal path"); 
                              turnAngle = 10;
                             
                             }
 
 // just follow compass
 else { FollowCompass();}
 
 Turn(turnAngle);
 
 Serial.println(""); 
 delay (1000);
}






/*----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
======== Subrutiens ================================================================================================================================================================
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/





//---- Sweep LIDAR -------------------------------------------------------------------------------------------------------------------------------------------------
void LIDARsweep(){ /* LIDAR servo is far right at 0 and far left at 180
                      This rutien creats a depth vector at 5 degree incraments starting at 50 degres right of center (-50)
                      with 0 degrees being center and 50 being  50 degrees left of center.
                    */
                   Serial.println("LIDAR is scanning...");
                   int a =0;
                   int angle = -50;
                   for ( int SweepAngle=35; SweepAngle < 140 ; SweepAngle += 5 ) { Lidar. write(SweepAngle); 
                                                                                   delay(50);
    
                                                                                   LIDARdistances[a] = GetDistance();
    
                                                                                   delay (50);
                                                                                /*
                                                                                   Serial.print ("Sweep Angle: "),      Serial.print(SweepAngle);
                                                                                   Serial.print ("   \tLIDAR Angle: "), Serial.print(angle);
                                                                                   Serial.print ("   \tDistance: " ),   Serial.println(LIDARdistances[a]);
                                                                                */
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
                        
  
  
// Driving rutiens -----------------------------------------------------------------------------------------------------------
void DriveForward( int Speed ) {

  int leftWheels  = LSM - Speed;
  int rightWheels = RSM + Speed;
  
  A. write(leftWheels);   // Wheel A  Back  left  wheel:  Forward < 90
  B. write(rightWheels);  // Wheel B  Back  right wheel:  Forward > 90
  C. write(leftWheels);   // Wheel C  Front left  wheel:  Forward < 90
  D. write(rightWheels);  // Wheel D  Front right whell:  Forward > 90
}

void DriveBackwards() {
 
  A. write(RSM);  // Wheel A  Back  left  wheel:  Forward < 90
  B. write(LSM);  // Wheel B  Back  right wheel:  Forward > 90
  C. write(RSM);  // Wheel C  Front left  wheel:  Forward < 90
  D. write(LSM);  // Wheel D  Front right whell:  Forward > 90
}

void Turn( int radius ) {
  
  // Assign wheel speed values
  // Turning radious: + Radius will turn left, - Radius will turn right
  // LSM = 77   RSM = 103
  
  int backLeft  = LSM + radius;
  int backRight = RSM + radius;
  int frontLeft  = LSM + radius;
  int frontRight = RSM + radius;
  
  if (radius > 0) { frontRight = frontRight + 1;} // Increas the speed of the front outer wheel to actuate the robot's pivot 
  else            { frontLeft  = frontLeft - 1; }
                    
                   
  // Drive
  A. write(backLeft);    // Wheel A  Back  left  wheel:  Forward < 90
  B. write(backRight);   // Wheel B  Back  right wheel:  Forward > 90
  C. write(frontLeft);   // Wheel C  Front left  wheel:  Forward < 90
  D. write(frontRight);  // Wheel D  Front right whell:  Forward > 90
  
 
}


void Stop(){
  
  A. write(90);   
  B. write(90);
  C. write(90);
  D. write(90);
  
}                      
 
 
 int GetIR(int pin){ int total = 0;

                    if      (pin == 0) {pin = A0;}
                    else if (pin == 1) {pin = A1;}
                    else               {pin = A2;}
                    
                    
                    for(int i=0; i<10; i++) { int plus = analogRead(pin);
                                              Serial.println(plus); 
                                              total= total + plus;
                                             }
              
                    int average = total/10;
                    Serial.print("Average: "), Serial.println(average);
                    return average;
                   }
 
 void FollowCompass() { Serial.println("Following Compass");}
