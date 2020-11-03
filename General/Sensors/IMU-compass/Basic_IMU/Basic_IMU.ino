/* Basic IMU Scrit that displays readings from the Accelerometer and Magnetometer
 * 
 */

// ---- Libraries ------
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

// --- Definitions ------
#define DECLINATION (12.0); // Magnetic Declination in dagrees

// ---- Assign IDs to the IMU sensors
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);
Adafruit_LSM303_Mag_Unified mag     = Adafruit_LSM303_Mag_Unified(12345);

float Role, Pitch, Heading; // Orientation Variables


void setup() {Serial.begin(9600); 
              Serial.println("\n------- Starting Script --------");

// ---- Accelerometer and Magnetometer set up --------------------------------
  Serial.println("Accelerometer and Magnetometer Test\n");
  
  mag.enableAutoRange(true); /* Enable auto-gain */
  
  if(!accel.begin() || !mag.begin()) {Serial.println("Ooops, no LSM303 detected ... Check your wiring!"); while(1); }


  /* Display some basic information on this sensor */
  displaySensorDetails();
}


// ---- Main Loop -------------------------------------------
void loop() {
  
  sensors_event_t event; 
  
  // Get Acceleraometer Data
  accel.getEvent(&event);
  
  float xaccel = (event.acceleration.x);
  float yaccel = (event.acceleration.y);
  float zaccel = (event.acceleration.z);
  
/* Display the Accelerometer results (acceleration is measured in m/s^2)*/
  Serial.print("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
  Serial.print("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");Serial.println("m/s^2 \n");


  // Get Magnetometer Data
  mag.getEvent(&event);
  
  float xmag = event.magnetic.x;
  float ymag = event.magnetic.y;
  float zmag = event.magnetic.z;
  
  /* Display the Magnetometer results (magnetic vector values are in micro-Tesla (uT))*/
  Serial.print("X: "); Serial.print(xmag); Serial.print("  ");
  Serial.print("Y: "); Serial.print(ymag); Serial.print("  ");
  Serial.print("Z: "); Serial.print(zmag); Serial.print("  ");Serial.println("uT \n\n\n"); 

  delay(1000);
}

// ======= Subrutiens ===========================================================================
// ----------------------------------------------------------------------------------------------


// --- Display Sensor data subrutien ------------------------------------------------------------
void displaySensorDetails(void){
  sensor_t sensor;
  
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  
  mag.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" uT");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" uT");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" uT");  
  Serial.println("------------------------------------");
  Serial.println("");
}


