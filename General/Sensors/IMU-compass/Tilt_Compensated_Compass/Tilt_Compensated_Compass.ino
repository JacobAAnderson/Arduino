/*  Tilt Compensated compass
 *   Version 1
 *   Jacob Anderson
 *   March 15th, 2016
 *  
 *  This script uses a magnetometer and accelerometer to calculate the heading of vehicle following the Y axis of the magnetometer.
 *  Changing which axis the heading is read from is easy and there are directions bellow on how to make this change.
 *  
 *  This script is set up for an Adafruit 10DOF INU. 
 *  It can easily be altered to utilize other devices. 
 *  If using other devices, the magnetometer and accelerometer must be oriented in the same direction so that their 
 *  X axes are parallel to each other. Same for their Y and Z axes.
 *  It is indicated bellow where to assign the sensor reading from your magnetometer and accelerometer.
 *  
 */


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>

#define DECLINATION 12 // Local magnetic declination

/* Assign a unique ID to this sensor at the same time */
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);




void setup(void) {
  Serial.begin(9600);
  pinMode(3,OUTPUT);
  digitalWrite(3, LOW);

  // ============================ Set Up For 10DOF IMU ============================================================================================================
  Serial.println("Magnetometer Test"); Serial.println("");
  
  mag.enableAutoRange(true); /* Enable auto-gain */
  
  /* Initialise the sensor */
  if(!mag.begin()) { Serial.println("Ooops, no LSM303 detected ... Check your wiring!");/* There was a problem detecting the LSM303 ... check your connections */
                      while(1);
                      }

  Serial.println("Accelerometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!accel.begin()) { Serial.println("Ooops, no LSM303 detected ... Check your wiring!"); /* There was a problem detecting the ADXL345 ... check your connections */
                        while(1);
                        }
  
  
  displaySensorDetails(); // Display some basic information on this sensor
  // ===============================================================================================================================================================
}

void loop(void) { 
  
  Serial.print("Heading:\t"), Serial.print(Orientation.heading), Serial.println(" [deg]\n\n");
  delay(3000);
  }

 { struct IMU Orientation { float heading, pitch, role; };
  
  // Get sensor data and normalize values --------------------------------------------------------------------------------
  sensors_event_t event; 
  
  // Acceleromitor values. Gravity must be negative downward when the sensor is right side up.
  accel.getEvent(&event);
  double Gx = -(event.acceleration.x);  // Assign X accelerometer value here
  double Gy = -(event.acceleration.y);  // Assign Y accelerometer value here
  double Gz = -(event.acceleration.z);  // Assign Z accelerometer value here

  double Gmag = sqrt( Gx*Gx + Gy*Gy + Gz*Gz );
  Gx = Gx/Gmag;
  Gy = Gy/Gmag;
  Gz = Gz/Gmag;

  // Magnetometer values. The Z compenent of the reading must be negative when the sensor is right side up.
  mag.getEvent(&event);
  double Bx = (event.magnetic.x); // Assign X magnetic value here
  double By = (event.magnetic.y); // Assign Y magnetic value here
  double Bz = (event.magnetic.z); // Assign Z magnetic value here
  
  double Bmag = sqrt( Bx*Bx + By*By + Bz*Bz );
  
  Bx = Bx/Bmag;
  By = By/Bmag;
  Bz = Bz/Bmag;


  // Find world coordinent fraim in terms of the local coordinate frame ==========================================
  // Find East, cross product of the gravity vector and magnetic vector
  double Ex = Gy*Bz - Gz*By;
  double Ey = -(Gx*Bz - Gz*Bx);
  double Ez = Gx*By - Gy*Bx;

  double Emag = sqrt( Ex*Ex + Ey*Ey +Ez*Ez );
  Ex = Ex/Emag;
  Ey = Ey/Emag;
  Ez = Ez/Emag;

  // Find North, cross product of the east vector and gravity vector
  double Nx = Ey*Gz - Ez*Gy;
  double Ny = -(Ex*Gz - Ez*Gx);
  double Nz = Ex*Gy - Ey*Gx;

  double Nmag = sqrt( Nx*Nx + Ny*Ny +Nz*Nz );
  Nx = Nx/Nmag;
  Ny = Ny/Nmag;
  Nz = Nz/Nmag;


  // Calculate vector for the direction of travel ============================================================================
  /* =========================================================================================================================
   * =========================================================================================================================
   * Here is where you can choose which axes on the IMU that you wish to follow.
   * This code is set up to follow the Y axes.
   * 
   * The following calculation is a projection of a vector of magnatude 1 along the axes of travel into the North - East plain
   * 
   * To change axis,
   * replace the subscripts of the terms multipying the North and East vectors with that of the desired axis.
   * 
   * i.e.: following the X axes would look like :
   * 
   * Yx = Nx*Nx + Ex*Ex;
   * Yy = Nx*Ny + Ex*Ey;
   * Yz = Nx*Nz + Ex*Ez;
   * 
   * ========================================================================================================================
   * ======================================================================================================================*/

  // Project direction of travel vector into the Nort - East plain
  double Yx = Ny*Nx + Ey*Ex;
  double Yy = Ny*Ny + Ey*Ey;
  double Yz = Ny*Nz + Ey*Ez;
  double Ymag = sqrt(Yx*Yx + Yy*Yy + Yz*Yz);

  Yx /= Ymag;
  Yy /= Ymag;
  Yz /= Ymag;

  // Calculate the components of the Y vector along the North and East vectors via dot products
  double  X = Yx*Nx + Yy*Ny + Yz*Nz;
  double  Y = Yx*Ex + Yy*Ey + Yz*Ez;


  // Calculate heading =====================================================================================================
  float heading = atan2(Y, X);

  heading = heading* 180.0/PI;
  
  if ( heading < 0 ) heading = 360 + heading;
 
  // Compensate for declination
  heading = heading + DECLINATION; 
  if ( heading > 360 ) heading = heading - 360;

  Orientation.heading = heading

  /* Debugging Statments
  Serial.print("Gx: "); Serial.print(Gx,4); Serial.print("  ");
  Serial.print("Gy: "); Serial.print(Gy,4); Serial.print("  ");
  Serial.print("Gz: "); Serial.print(Gz,4); Serial.print("  ");Serial.println(" [m/s^2]");

  Serial.print("Bx: "); Serial.print(Bx,4); Serial.print("  ");
  Serial.print("By: "); Serial.print(By,4); Serial.print("  ");
  Serial.print("Bz: "); Serial.print(Bz,4); Serial.print("  ");Serial.println(" [uT] \n");

  Serial.print("Ex: "); Serial.print(Ex,4); Serial.print("  ");
  Serial.print("Ey: "); Serial.print(Ey,4); Serial.print("  ");
  Serial.print("Ez: "); Serial.print(Ez,4); Serial.print("\n");

  Serial.print("Nx: "); Serial.print(Nx,4); Serial.print("  ");
  Serial.print("Ny: "); Serial.print(Ny,4); Serial.print("  ");
  Serial.print("Nz: "); Serial.print(Nz,4); Serial.print("\n");

  Serial.print("Yx: "); Serial.print(Yx,4); Serial.print("  ");
  Serial.print("Yy: "); Serial.print(Yy,4); Serial.print("  ");
  Serial.print("Yz: "); Serial.print(Yz,4); Serial.print("\n\n");

  Serial.print("X: "); Serial.print(X,4); Serial.print("  ");
  Serial.print("Y: "); Serial.print(Y,4); Serial.print("\n\n");
   */
 
  return Orientation;
}



void displaySensorDetails(void) {
  sensor_t sensor;
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
  delay(500);

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
  delay(500);
}

