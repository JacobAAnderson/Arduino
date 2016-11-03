
unsigned long time; // Time

int SamplingRate = 2;       // Interval between Sensor Sampling in Seconds
int takeSample = false;     // State varible for taking Sensor readings
int SensorOut = false;      // State verialbe for outputing sensor information
int sampleCount = 0;        // Index for the sensor array
const int SensorSize = 30;  // Size of the sensor array
int RawData[SensorSize];    // Array to store the raw sensor data
int Sensor1[SensorSize];    // Array to store the Averaged sensor inputs
char Sensor1_type[] = "I'm a Sensor"; // Name of the sensor


void setup() { Serial.begin(9600);
  pinMode(A0, INPUT);

}

void loop() {
 
 time = millis()/1000; // Time in seconds

// Take sensor reading if time is a multiple of the sampling rate
 if ((time%SamplingRate) == 0 && takeSample == false) { Serial.print("Time: "), Serial.print(time), Serial.println(" [s]");
                                                        takeSample = true;  
                                                        SensorOut = UpdateSensors(); }
 else if ((time%SamplingRate) == 0 && takeSample == true) ;    
 else takeSample = false ;

 
 // Display the sensor log once the arrays are full
 if (SensorOut == true ) { Serial.print(Sensor1_type);
                           Serial.print("\n \t Transmit Data \n--------------------------------------\n");
                              for( int i = 0; i < SensorSize; i++) { Serial.print("Raw Data: "), Serial.print(RawData[i]);
                                                                     Serial.print("\tSensor Data: "), Serial.println(Sensor1[i]);}
                           Serial.println(" \tEnd Sensor log \n-------------------------------------");                                          
    SensorOut = false;
 } 
}



boolean UpdateSensors() { // Reads the sensors, takes an average of 10 reaings and saes the averaged measurement to the sensor array
  
  // Save Raw data for comparison
  Serial.print("Raw Data: "), Serial.print(analogRead(A0));
  RawData[sampleCount] = analogRead(A0); 
  
  // Take the average reading from the sensor
  int Sample = 0;
  for(int i=0; i<10; i++) { Sample = Sample + analogRead(A0);}
  Sample = Sample/10;
  Serial.print( "\tAverage Value: "), Serial.println (Sample);
  Serial.println("");

  Sensor1[sampleCount] = Sample;

  // Increass the sensor array index and indicate if the array if full
  if(sampleCount == (SensorSize-1)) {sampleCount = 0;
                                      return true; } // Array is full
  else {sampleCount++;
        return false; } // Array is not full
}

