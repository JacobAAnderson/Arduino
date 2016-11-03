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

// Define Variables;
int LIDARangle = 35;
const int LIDARvec = 11;
int DISTANCE;


// Vectors
int LIDARdistances[LIDARvec];  // Vector to store LIDAR measurments


// ==== Set up routien =====================================================================================================================
// =========================================================================================================================================
void setup(){
  
  // === Set up LIDAR =================================================================
  Serial.begin(9600); //Opens serial connection at 9600bps.     
  I2c.begin(); // Opens & joins the irc bus as master
  delay(100); // Waits to make sure everything is powered up before sending or receiving data  
  I2c.timeOut(50); // Sets a timeout to ensure no locking up of sketch if I2C communication fails

  Lidar.attach(6); // Attache Lidar Servo to pin 6
  
  Lidar.write(85);  // Centers Lidar detector
  delay (10);
  
  // === Set up LIDAR vecor ==========================================================
  for (int zero = 0; zero < LIDARvec; zero++) {LIDARdistances[zero] = 0;}
  
  Serial.println("Set up done");
  
  PrintLIDARvector(); 
  
  
}
// ==== Main Loop ===========================================================================================================================
// ===========================================================================================================================================
void loop(){ 
  
  Lidar.write(85);
  LIDAR();
  Serial.println(DISTANCE);
  
  LIDARsweep();
  // Serial.println(LIDARdistances);
  
  // Print Distance
  PrintLIDARvector();
 delay (1000);
}



// ===== Subrutiens ==========================================================================================================================
// ===========================================================================================================================================

void LIDARsweep(){ // LIDAR servo is far right at 0 and far left at 180
  
  Serial.println("LIDAR is scanning...");
  int a =0;
  for ( LIDARangle=35; LIDARangle < 145 ; LIDARangle+=10 )
  { Lidar. write(LIDARangle);
    delay(100);
    
    LIDAR();
    LIDARdistances[a] = DISTANCE;
    
    delay (400);
    
    a=a+1;
    // LIDARangle = LIDAR
  }
  Lidar.write(85);
   
}

// ==== Get distance from LIDAR =======================================================
void LIDAR() {
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
  DISTANCE = distance;
}
  
// === Print Lidar Vector ==================================================================

void PrintLIDARvector(){
 
  Serial.print(" LIDAR Vector: [ ");
  for (int i=0;i<(LIDARvec-1);i++)
  { Serial.print(LIDARdistances[i]);
    Serial.print(", ");}
    Serial.print(LIDARdistances[(LIDARvec-1)]);
    Serial.println(" ]");
}


