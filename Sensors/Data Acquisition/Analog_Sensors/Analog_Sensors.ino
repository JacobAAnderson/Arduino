/*  Data Acquisition Scrip
 *  Jacob Anderson
 *  Fort Lewis College
 *  December 26th, 2015
 *  
 *  This scrip colects data from a sensor at a givien time tinterval. 
 *  The data is averaged over 10 readings. 
 *  It then transmits the averaged measurment to another device via I2C comunication.
 *  
 *  This scrip is supporst an analog sensor conected to pin A0
 *  I2C on the arduino Micr: SDA - Pin 2, SCL - pin 3
 */
 
// Libraries --------------------------------------------------------------------------------------------------------------------------
#include <Wire.h>

// Time Keeping -----------------------------------------------------------------------------------------------------------------------
unsigned long time; // Time

// PIN assignments --------------------------------------------------------------------------------------------------------------------
int sensor1PIN = A0;    // Which Sensor is attached to this Pin
int sensor2PIN = A1;    // Which Sensor is attached to this Pin
int sensor3PIN = A2;    // Which Sensor is attached to this Pin
int sensor4PIN = A3;    // Which Sensor is attached to this Pin
int sensor5PIN = A4;    // Which Sensor is attached to this Pin

int LEDPIN = 3;

// Sensor variables -------------------------------------------------------------------------------------------------------------------
int SamplingRate = 2;       // Interval between Sensor Sampling in Seconds
float AverageRange = 10.0;  // Number of sensor readings to average over
int takeSample = false;     // State varible for taking Sensor readings
int SensorOut = false;      // State verialbe for outputing sensor information
int sampleCount = 0;        // Index for the sensor array

// Arrays to store data --------------------------------------------------------------------------------------------------------------
const int ArraySize = 30;    // Size of the sensor array
float SampleTime[ArraySize]; // Array to store the time of each set of measurments
float GPSlat[ArraySize];     // Array to store GPS latitude of each measurment
float GPSlon[ArraySize];     // Array to store GPS longitude of each measurment
float Sensor1[ArraySize];    // Array to store the Averaged sensor inputs from 1st sensor
float Sensor2[ArraySize];    // Array to store the Averaged sensor inputs from 2nd sensor
float Sensor3[ArraySize];    // Array to store the Averaged sensor inputs from 3rd sensor
float Sensor4[ArraySize];    // Array to store the Averaged sensor inputs from 4th sensor
float Sensor5[ArraySize];    // Array to store the Averaged sensor inputs from fth sensor

char Sensor1_type[] = "I'm a Sensor"; // Name of the sensor


/*=====================================================================================================================================
 * ============================================ Set Up ================================================================================
 * ==================================================================================================================================*/
 
void setup() { Serial.begin(9600);

  Serial.print("\n\n--------- Begining Set Up -----------\n");

  Wire.begin(); // Start I2C Bus as Master

  // Set up pin I/O state-----
  pinMode(sensor1PIN, INPUT);
  pinMode(sensor2PIN, INPUT);
  pinMode(sensor3PIN, INPUT);
  pinMode(sensor4PIN, INPUT);
  pinMode(sensor5PIN, INPUT);
  pinMode( LEDPIN, OUTPUT);

  // Turn LED on and of to verify functionality
  digitalWrite(LEDPIN, HIGH);
  delay(1000);
  digitalWrite(LEDPIN, LOW);

  Serial.print("\n----------- Set Up Done -------------\n\n");

}

/*=====================================================================================================================================
 *============================================= Mian Loop ============================================================================= 
 *===================================================================================================================================*/

void loop() {
 
 time = millis()/1000; // Time in seconds

// Take sensor reading if time is a multiple of the sampling rate
 if ((time%SamplingRate) == 0 && takeSample == false) { Serial.print("Time: "), Serial.print(time), Serial.println(" [s]");
                                                        takeSample = true;  
                                                        SensorOut = UpdateSensors(); }
 else if ((time%SamplingRate) == 0 && takeSample == true) ;    
 else takeSample = false ;

 
 // Display the sensor log once the arrays are full
 if (SensorOut == true ) { digitalWrite(3,HIGH);
                           Serial.print(Sensor1_type);
                           Serial.print("\n \t Transmit Data \n--------------------------------------\n");
                              for( int i = 0; i < ArraySize; i++) {
                                                                     Serial.print("\tSensor Data: "), Serial.println(Sensor1[i]);

                                                                     I2C(0, i);
                                                                     I2C(1, SampleTime[i]);
                                                                     I2C(2, GPSlat[i]);
                                                                     I2C(3, GPSlon[i]);
                                                                     I2C(4, Sensor1[i]);
                                                                     I2C(5, Sensor2[i]);
                                                                     I2C(6, Sensor3[i]);
                                                                     I2C(7, Sensor4[i]);
                                                                     I2C(8, Sensor5[i]);
                                                                    
                                                                     }
                           Serial.println(" \tEnd Sensor log \n-------------------------------------");

                           digitalWrite(3,LOW);                                          
    SensorOut = false;
 } 
}


/*=====================================================================================================================================
 *============================================ Subrutiens ============================================================================= 
 *===================================================================================================================================*/


// Process sensor data and transmit via I2C -------------------------------------------------------------------------------------------
//-------------------------------------------------------------------------------------------------------------------------------------
boolean UpdateSensors() { // Reads the sensors, takes an average of 10 reaings and saes the averaged measurement to the sensor array

  SampleTime[sampleCount] = time;
  GPSlat[sampleCount] = random(100.00, 1000.00);
  GPSlon[sampleCount] = random(-100.00, -1000.000);

  // Initalize Sampling variables
  float Sample1 = 0;
  float Sample2 = 0;
  float Sample3 = 0;
  float Sample4 = 0;
  float Sample5 = 0;

  
  
  // Take the average reading from the sensor

  for(int i=0; i<AverageRange; i++) { Sample1 = Sample1+ analogRead(sensor1PIN);
                            Sample2 = Sample2 + analogRead(sensor2PIN);
                            Sample3 = Sample3 + analogRead(sensor3PIN);
                            Sample4 = Sample4 + analogRead(sensor4PIN);
                            Sample5 = Sample5 + analogRead(sensor5PIN);
                          }

  Sample1 = Sample1/AverageRange;
  Sample2 = Sample2/AverageRange;
  Sample3 = Sample3/AverageRange;
  Sample4 = Sample4/AverageRange;
  Sample5 = Sample5/AverageRange;

  /*  // Debuging Print Statements
  Serial.print("\t Raw1: "), Serial.print(analogRead(A0)), Serial.print( "\tSensor1: "), Serial.println (Sample1);
  Serial.print("\t Raw2: "), Serial.print(analogRead(A1)), Serial.print( "\tSensor2: "), Serial.println (Sample2);
  Serial.print("\t Raw3: "), Serial.print(analogRead(A2)), Serial.print( "\tSensor3: "), Serial.println (Sample3);
  Serial.print("\t Raw4: "), Serial.print(analogRead(A3)), Serial.print( "\tSensor4: "), Serial.println (Sample4);
  Serial.print("\t Raw5: "), Serial.print(analogRead(A4)), Serial.print( "\tSensor5: "), Serial.println (Sample5);
  Serial.println("");
  */

  // Store the data to its array
  Sensor1[sampleCount] = Sample1;
  Sensor2[sampleCount] = Sample2;
  Sensor3[sampleCount] = Sample3;
  Sensor4[sampleCount] = Sample4;
  Sensor5[sampleCount] = Sample5;

  /*
  // Transmit value via I2
  Serial.println( "Transmting On I2C");
  I2C( 1,Sample1 );
  I2C( 2,Sample2 );
  I2C( 3,Sample3 );
  I2C( 4,Sample4 );
  I2C( 5,Sample5 );
  */
  // Increass the sensor array index and indicate if the array if full

  if(sampleCount == (ArraySize-1)) {sampleCount = 0;
                                      return true; } // Array is full
  else {sampleCount++;
        return false; } // Array is not full
}


// Transmit data over I2C ------------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------------------------------------
void I2C( int Data, float value){

  // Transmit the data to another device via I2C
  Wire.beginTransmission(8); // transmit to device #8
  if (Data == 0 ) Wire.write("index");
  else if (Data == 1) Wire.write("Time");
  else if (Data == 2) Wire.write("GPSlat");
  else if (Data == 3) Wire.write("GPSlon");
  else if (Data == 4) Wire.write("Sensor1");
  else if (Data == 5) Wire.write("Sensor2");
  else if (Data == 6) Wire.write("Sensor3");
  else if (Data == 7) Wire.write("Sensor4");
  else if (Data == 8) Wire.write("Sensor5");
  else Wire.write("Not a Valid sensor");


  union {
    float number;
    uint8_t bytes[4];
    } m;

  m.number = value;

  for (int i=0; i<4; i++) { Wire.write(m.bytes[i]);
                            // Debuging
                            Serial.print("Bytes: ");
                            Serial.println(m.bytes[i]);
                            
                            }
 
  Wire.endTransmission();    // stop transmitting

  
  
  }
