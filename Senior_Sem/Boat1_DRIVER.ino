/*  Boat Driver Code - Boat 3 - version 12
 *  Jacob Anderson
 *  Fort Lewis College
 *  June 20th, 2016
 *  
 *  This script drives a boat with differential thruster steering via GPS waypoint navigation with a geo-fence.
 *  It takes water quality data and send it to an electron via I2C.
 *  
 *  * Sept 1st, 2016 - Thruster calibration routien implemented
 *  
 *  * June 20th, 2016 - GPS Navigation upgraded
 *  
 *  * June 2nd, 2016 - I2C communication changed to String content
 *  
 *  * May 27th, 2016 - Watch Dog Timer added
 *  
 *  * May 26th, 2016 - Data logging to an SD card
 *  
 *  * March 29th, 2016 - This Code Includes GPS Nav with Goe-Fencing  
 *  
 *  * March 24th, 2016 - The trusters are controlled via I2C which provides information on power consumption and RPMs
 *                       
 *  * March 15th, 2015 - Tilt compensated compass routine using a magnetometer and accelerometer.                    
 *  
 *  * Dec 26th, 2015 - Collect data from six external sensor and 7 internal sensors at a given time interval. 
 *                     The time interval is listed in the preamble below as "SamplingRate".
 *    
 *                     The external sensor data ( Water Quality ) is averaged over 10 readings. It is then converted into physical data based on the sensor's transfer function. 
 *                     The internal sensors gather data on power usage, condition inside the electronics housing, speed, heading and GPS position.
 *    
 *                     This data is stored in arrays. Once the arrays are full it is transmits to a wireless communication device via I2C communication.
 *    
 *    This Code reads Vernier analog Sensors. 
 *    Sensors: Sensor 1 - Water Conductivity / Salinity Probe
 *             Sensor 2 - Water Temperature Probe
 *             Sensor 3 - Water pH Probe
 *             Sensor 4 - empty
 *             Sensor 5 - empty
 *             Sensor 6 - empty
 *                    
 *===============================================================================================================================================================                    
 * Pin out 
 *            Arduino Pin - Input / Output                                     - Vernier Sensor's Pin
 *           -----------------------------------------------------------------------------------------------------------
 *                  5Vcc  - All Sensors                                        - Pin 5 Power
 *                  GND   - All Sensors                                        - Pin 2 GND
 *                  
 *                  A0    - Water Quality: Salinity                            - Pin 6 0-5V Sensor Output                   
 *                  A1    - Water Quality: Thermometer                         - Pin 6 0-5V Sensor Output
 *                  A2    - Water Quality: pH                                  - Pin 6 0-5V Sensor Output 
 *                  A3    - Water Quality: empty
 *                  A4    - Water Quality: empty
 *                  A5    - Water Quality: empty
 *                  
 *                  
 *                  0     - Rx
 *                  1     - Tx
 *                  2     - Thruster Power ( To SSR)
 *                  3     - Thruster I2C enable
 *                  4     - SPI bus: Chip Select
 *                  5     - Electron Power
 *                  6     - -------------------------------> This pin is buggy with the SD card shield on the Arduino
 *                  7     - Sensor Power
 *                  
 *                  8     - Port Thruster
 *                  9     - -------------------------------> (Boat 1, No PWM on this pin)
 *                  10    - StarBoard Thruster
 *                  11    - 
 *                  12    - 
 *                  13    - 
 *                  14    - Power Off  ( To power relay )
 *                  
 *                  18    - Tx1 to GPS Rx
 *                  19    - Rx1 to GPS Tx
 *                  20    - SDA
 *                  21    - SDL                   
 *                  
 *==============================================================================================================================================================                  
 *  Blue ESC Addresses
 *  
 *  Number:     0       1       2       3       4       5       6       7       8       9      10
 *  Address:  0x29    0x2A    0x2B    0x2C    0x2D    0x2E    0x2F    0x30    0x31    0x32    0x33 
 */
 
// Libraries =============================================================================================================================
#include <Adafruit_GPS.h>       // GPS
#include <Adafruit_HTU21DF.h>   // Temperature Humidity Sensor
#include <Adafruit_LSM303_U.h>  // IMU
#include <Adafruit_Sensor.h>    // Sensor library for IMU
#include "Arduino_I2C_ESC.h"    // Blue ESC 
#include <avr/wdt.h>            // Watch dog timer library
#include <math.h>               // Extra math functions
#include <SD.h>                 // SD card read/write library
#include <SPI.h>                // SPI communication
#include <Wire.h>               // I2C communication


// Macros ===================================================================================================================================================
// PIN assignments -------------------------------------------------------
#define SENSOR_1PIN A0    // Salinity
#define SENSOR_2PIN A1    // Temperature
#define SENSOR_3PIN A2    // pH
#define SENSOR_4PIN A3    // Empty
#define SENSOR_5PIN A4    // Empty
#define SENSOR_6PIN A5    // Empty


#define CHIP_SELECT_PIN 4         // SPI Chip Select Pin
#define ELECTRON_POWER_PIN 5      // Power to the SSR that supplies the Electron
#define POWER_PIN 14              // Boat automatic power off
#define SENSOR_POWER_PIN 7        // Power to the SSR that supplies the water quality sensors

#define THRUSTER_I2C_PIN 3        // Connects Thruster I2C
#define THRUSTER_POWER_PIN 2      // Power to the SSR that supplies the thrusters
#define THRUSTER_PORT_PWM_PIN 8   // PWM to Port thruster
#define THRUSTER_STAR_PWM_PIN 10  // PWM to Starboard thruster


// I2C Addresses ------------------------------------------------------------------------------
#define ELECTRON_ADDRESS 8      // I2C address for the Electron
#define ESC_ADDRESS_PORT 0x29   // Port Thruster ID 0
#define ESC_ADDRESS_STAR 0X2C   // Starboard Thruster ID 3

// SD Card Data Log File Size ------------------------------------------------------------------
#define SD_FILE "Boat1.txt"

// Constants -----------------------------------------------------------------------------------
#define ANGLE 30                  // Angle for pivoting towards waypoint
#define AVE_THRUST 60             // Average thruster input as a percent of the full range
#define RAMP_STEP 5000            // Velocity Ramp -> step interval
#define TURN 40                   // Thruster input for pivoting as a percent of the full range
#define MAX_THRUSTER_INPUT 16000  // Maximum accepted thruster input
#define MIN_THRUSTER_INPUT -16000 // Minimum accepted thruster input

#define DECLINATION 12.0          // Local magnetic declination in degrees
#define SAMPLING_RATE 10          // Sensor Sampling frequency in seconds
#define SUNRISE 7                 // Hour of Sun Rise --> 7am
#define SUNSET 19                 // Hour of sun Set ---> 7pm
#define TIMEZONE 6                // Time Zone Difference from Grenage Mean Time
#define WAYPOINT_RADIUS 5        // Waypoint Radius in Meters

/* Lat, Long to meters in Durango*/
#define LON_TO_METER 88672.45     // Convert Degrees of Longitude to meters at 37.29 deg Lat
#define LAT_TO_METER 110983.06    // Convert Degrees of Latitude to meters at 37.29 deg Lat

/* Lat, Lon to meters in LA
#define LON_TO_METER  92266.59   // Convert Degrees of Longitude to meters at 34.109 deg Lat
#define LAT_TO_METER 110924.33   // Convert Degrees of Latitude to meters at 34.109 deg Lat
*/

// Functions------------------------------------------------------------------------------
#define VOLTAGE(V) ( V * 5.0 / 1025.0 )  // Function that converts an analog reading to its corresponding voltage
//===============================================================================================================================================================


// Name New Ports, Sensors and Motors ============================================================================================================================
// Ports ---------------------------------------------------------------------------------
HardwareSerial mySerial = Serial1;  //Initialize new software serial port for GPS communication

// Motors --------------------------------------------------------------------------------
Arduino_I2C_ESC motor_PORT(ESC_ADDRESS_PORT);   // Name thrusters
Arduino_I2C_ESC motor_STAR(ESC_ADDRESS_STAR);

// Sensors ------------------------------------------------------------------------------------
Adafruit_GPS GPS(&Serial1);                                                     // GPS 
Adafruit_HTU21DF htu = Adafruit_HTU21DF();                                      // Internal temperature and humidity sensor
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(54321);     // IMU - accelerometer
Adafruit_LSM303_Mag_Unified mag = Adafruit_LSM303_Mag_Unified(12345);           // IMU - magnetometer

//===============================================================================================================================================================


// Time Keeping -----------------------------------------------------------------------------------------------------------------------
unsigned long time; // Time in seconds
String Time;        // String to hold GPS time --> Hr:min:sec

// Global State Variables ===================================================================================================================================
bool Battery = HIGH;   // State variable for battery voltage
bool Cell = true;      // State variable to indicate if the electron is On or Off
bool DayTime = true;   // State variable to identify day time or night time
bool InFence = true;   // State Variable to indicate that boat is inside the Geo Fence
bool Thrusters = true; // State variable to indicate if the trusters are on or off


// Calibration Values ========================================================================================================================================= 
// Motors -----------------------------------------------------
int portMax, portMin; // Max and Min signal values for the Port Thruster
int starMax, starMin; // Max and Min signal values for the Starboard Thruster

// GPS Nave Variables ==========================================================================================================================================
//GPS parsing------------------------------------------------------------------------------
float bearing;     // Direction of travel calculated from GPS. Set to 0 "N" if GPS never gets a fix
float distance;     // Radius from waypoint
byte gpsindex = 0; // Waypoint index
byte pindex;       // Holds Waypoint index when executing Geo - Fence


// Navigation Waypoints -->  The last waypoint is in the middle of the Geo Fence ----------------------------------------------------------------------------------------------------------------------------------
/* West Side of Rogers Reservoir
float desLat[] = {   37.2918732,    37.2920454,    37.2911140,    37.2916318 };   // Latitudinal waypoints.  Last point is center of polygon for geo fence
float desLon[] = { -107.8468074,  -107.8451296,  -107.8461901,  -107.8459003 };   // Longitudinal waypoints. Last point is center of polygon for Geo- Fencing
*/

/* Center of Rogers Reservoir */
float desLat[] = {   37.290866,   37.290768,   37.290287,   37.290328,   37.290968 };   // Latitudinal waypoints.  Last point is center of polygon for geo fence
float desLon[] = { -107.846407, -107.845710, -107.845788, -107.846376, -107.845742};   // Longitudinal waypoints. Last point is center of polygon for Geo-Fencing

/* Harvey Mud College
float desLat[] = {   34.109171,   34.109090,   34.108993,   34.109150 };   // Latitudinal waypoints.  Last point is center of polygon for geo fence
float desLon[] = { -117.712713, -117.712488, -117.712515, -117.712591 };   // Longitudinal waypoints. Last point is center of polygon for Geo-Fencing
*/

/* Geo-Fencing ===================================================================================================================================================
 * =============================================================================================================================================================== 
 * 
 * Uncomment the area you are working in
 */

/* Police Pond ---------------------------------------------------------------------------------------------------------------------------------------------------
float polyX[] = { -107.8646571,  -107.8648315,  -107.8650710,  -107.8649944,  -107.8647844,  -107.8645760,  -107.8646571 };  // Longitudinal coordinates of geo-fence
float polyY[] = {   37.2807077,    37.2806380,    37.2802310,    37.2800889,    37.2800872,    37.2805308,    37.2807077 };  // Latitudinal coordinates of geo-fence
*/

/* West side of Roger's Reservoir -------------------------------------------------------------------------------------------------------------------------------
float polyX[] = { -107.8466066,  -107.8449061,  -107.8446771,  -107.8449667,  -107.8473404,  -107.8468529,  -107.8466066 };  // Longitudinal coordinates of geo-fence
float polyY[] = {   37.2907932,    37.2911090,    37.2921141,    37.2924604,    37.2920273,    37.2915075,    37.2907932 };  // Latitudinal coordinates of geo-fence
*/

/* Center of Rogers Reservoir ---------------------------------------------------------------------------------------------------------------------------------- */
float polyX[] = { -107.846868,  -107.845248,  -107.844876,  -107.845492,  -107.846354,  -107.846868 };  // Longitudinal coordinates of geo-fence
float polyY[] = {   37.291170,    37.291698,    37.290559,    37.290129,    37.290026,    37.291170 };  // Latitudinal coordinates of geo-fence

/* Harvey Mud College pond -------------------------------------------------------------------------------------------------------------------------------------
float polyX[] = { -117.712842, -117.712765, -117.712507, -117.712383, -117.712521, -117.712708,  -117.712842 };   // Longitudinal coordinates of geo-fence
float polyY[] = {   34.109080,   34.109249,   34.109189,   34.108992,   34.108930,   34.109068,    34.109080, };  // Latitudinal coordinates of geo-fence
*/



/*=====================================================================================================================================
 * ============================================ Set Up ===============================================================================
 * ==================================================================================================================================*/
 
void setup() { //Serial.begin(115200);
  
// Pin Set Up =========================================================================== 
  // Set up pin I/O state--------------------------------
  pinMode(SENSOR_1PIN, INPUT);
  pinMode(SENSOR_2PIN, INPUT);
  pinMode(SENSOR_3PIN, INPUT);
  pinMode(SENSOR_4PIN, INPUT);
  pinMode(SENSOR_5PIN, INPUT);
  pinMode(SENSOR_6PIN, INPUT);

  pinMode( ELECTRON_POWER_PIN, OUTPUT);
  pinMode( SENSOR_POWER_PIN, OUTPUT);
  pinMode( POWER_PIN, OUTPUT);
  pinMode( THRUSTER_I2C_PIN, OUTPUT );
  pinMode( THRUSTER_POWER_PIN,OUTPUT);
  pinMode( THRUSTER_PORT_PWM_PIN, OUTPUT);
  pinMode( THRUSTER_STAR_PWM_PIN, OUTPUT);
  

  // Set pins to there start up state ----------------------
  digitalWrite( ELECTRON_POWER_PIN, HIGH);
  digitalWrite( THRUSTER_I2C_PIN, HIGH );
  digitalWrite( THRUSTER_POWER_PIN, HIGH);

  digitalWrite( SENSOR_POWER_PIN, LOW );
  digitalWrite( POWER_PIN, LOW );
  digitalWrite( THRUSTER_PORT_PWM_PIN, LOW );
  digitalWrite( THRUSTER_STAR_PWM_PIN, LOW);


  delay(10); // Delay for stability
  
  
  bool FAIL = LOW;  // Local State variable to turn off boat if sensors, battery voltage or SD card do not check out


// Initialize the SD card read/write =======================================================================================================
  // see if the card is present and can be initialized:
  if (!SD.begin(CHIP_SELECT_PIN)) FAIL = HIGH;                     

  delay(10); // Delay for stability
  
  // I2C set up ================================================================================================================================
  Wire.begin(); // Start I2C Bus as Master
  
   // Slow down I2C clock to 12.5 kHz (Best used for transmitting I2C over long wires )-------
   TWBR = 158;  
   TWSR |= bit (TWPS0);
   //----------------------------------------------------------------------------------------

  delay(10); // Delay for stability
  
// Sensor Set up ==================================================================================================================================================
  // Temperature and Humidity Sensor -----------------------------------------------------
   if (!htu.begin()) { File dataFile = SD.open(SD_FILE, FILE_WRITE);
                       dataFile.println("Temperature and Humidity Sensor Failed to Initialize");
                       dataFile.close(); 
                       
                       FAIL = HIGH;
                      }
  // IMU ------------------------------------------------------------------------------------------------------
  mag.enableAutoRange(true); /* Enable auto-gain */
  
  if(!mag.begin()){   File dataFile = SD.open(SD_FILE, FILE_WRITE);
                      dataFile.println("IMU Failed to Initialize");
                      dataFile.close(); 
                       
                      FAIL = HIGH;
                    }

  
  if(!accel.begin()) {  File dataFile = SD.open(SD_FILE, FILE_WRITE);
                        dataFile.println("IMU Failed to Initialize");
                        dataFile.close(); 
                        
                        FAIL = HIGH;
                      }

  
  delay(10); // Delay for stability
  
  // GPS Set up =======================================================================================================================
  // Check that the X and Y arrays of the Geo-Fence are the same size --------------------------------------------
  if (sizeof(polyX) != sizeof(polyY)) { File dataFile = SD.open(SD_FILE, FILE_WRITE);
                                        dataFile.println("Geo-Fence array sizes do not match");
                                        dataFile.close(); 
                       
                                        FAIL = HIGH;
                                        }
    
  // Check that the X and Y arrays of the Geo-Fence are the same size --------------------------------------------
  if (sizeof(desLat) != sizeof(desLon)) { File dataFile = SD.open(SD_FILE, FILE_WRITE);
                                          dataFile.println("GPS array sizes do not match");
                                          dataFile.close(); 
                                          
                                          FAIL = HIGH;
                                         }                                      

  // Initialize the GPS unit -------------------------------------------------------------------------------------
  GPS.begin(9600);                              //turn on GPS at 9600
  GPS.sendCommand("$PGCND,33,0*6D");            //turn off antenna update data
  GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA); //request rmc and gga sentences only
  GPS.sendCommand(PMTK_SET_NMEA_UPDATE_10HZ);   //set update rate 10 1 Hz
  // Interrupt for updated GPS info ------------------------------------------------------------------------------
  OCR0A = 0xAF;
  TIMSK0 |= _BV(OCIE0A);

  delay(10); // Delay for stability
  
  motor_PORT.update();
  motor_STAR.update();
  
  // Wait for motors to start up --------------------------------------------------------------------
  while ( (motor_PORT.voltage() == 0.0) || (motor_STAR.voltage() == 0.0)){ motor_PORT.update();
                                                                           motor_STAR.update();
                                                                          }


  for ( int i = 0; i <= 10; i++ ) { // Set thruster signal to stop for activation
                                    int signal1 = 0;
                                    
                                    // Send Signal to the thrusters
                                    motor_PORT.set( signal1 );
                                    motor_STAR.set( signal1 );

                                    delay(100);
                                    }


  // Check Battery Voltage to see that Voltage is sufficient for operation
  if( (motor_PORT.voltage() <= 10.5) || (motor_STAR.voltage() <= 10.5) ) { File dataFile = SD.open(SD_FILE, FILE_WRITE);
                                                                           dataFile.println("Low Battery Voltage at start up");
                                                                           dataFile.close(); 
                                                                           
                                                                           FAIL = HIGH;
                                                                           }

  
  // Turn off boat if sensors, battery voltage or SD card do not check out-----------------------------------------------------
  digitalWrite( POWER_PIN, FAIL );


  delay(10); // Delay for stability


  // Do Thruster Calibration --------------------------------------------------------------------------------------------------
  ThrusterCalibrate(); 


  // Wait for GPS to get a fix -----------------------------------------------------------------------------------------------
  while(!GPS.fix) GPS.parse(GPS.lastNMEA());        

  TimeKeeping();
  
  //Set up a header ----------------------------------------------------------------------------------------------------------
  String dataString = "Start Up: ";
  dataString += Time;
  
  File dataFile = SD.open(SD_FILE, FILE_WRITE);
  dataFile.println(dataString);
  dataFile.close();

  delay(10); // Delay for stability

// Set up Watch dog timer ========================================================================================================
  cli();        // disable all interrupts 
  wdt_reset();  // reset the WDT timer 

  WDTCSR |= B00011000;  // Enter Watchdog Configuration mode: 

  /* Set Watchdog settings:
   * Set timer to restart the Arduino after  ~8 second
   * Enable the use of an interrupt --> Will log reset to the SD card
   */
  WDTCSR = B01101001;

  sei(); // Enable all interrupts  


//=============================== End Setup =====================================================================================================================================================
}




/********************** Timers ******************************************************************************************************/
SIGNAL(TIMER0_COMPA_vect) { char c = GPS.read();} // Interrupt is called once a millisecond, looks for any new GPS data, and stores it
/************************************************************************************************************************************/


/*=====================================================================================================================================
 *============================================= Mian Loop ============================================================================= 
 *===================================================================================================================================*/

void loop() {

  wdt_reset();     // Reset Watch dog timer
  TimeKeeping();   // Update Time

  GPS.parse(GPS.lastNMEA());  // Update GPS data
  GetBearing();               // Up date bearing and distance
  GPSindex();                 // Check for waypoint radius achievment

  Drive();    


  /* Take sensor reading if time is a multiple of the sampling rate  ===========================================================================================
   * The DayTime State variable is managed as a subset of this section
   */
  
  static bool takeSample;
  if ((( time + 1 ) % SAMPLING_RATE ) == 0 ) digitalWrite(SENSOR_POWER_PIN, HIGH); // Turn Sensor power on
  
  if (( time % SAMPLING_RATE ) == 0 && takeSample == true ) { takeSample = false;  
                                                              UpdateSensors();   //Take Sensor readings
                                                              BatteryCheck();    // Check the battery voltages
                                                              // ThrusterCheck();   // Check thrusters for drop outs
                                                              // ThrusterData();
                                                           }
  else if ( (time % SAMPLING_RATE ) == 0 && takeSample == false ) ;    
  else takeSample = true ;

  
// Regulate Electron power based on Battery voltage =============================================================================================================
  if ( Battery == HIGH && !Cell ) ElectronOn();
  if ( Battery == LOW  &&  Cell ) ElectronOff();


  MonitorGPS(); // Monitor GPS for drop outs
}


/*==============================================================================================================================================================================
**                                                                                                                                                                            **
**                                                                 Subroutines                                                                                                 **
**                                                                                                                                                                            **
*=============================================================================================================================================================================*/


void BatteryCheck(void) { // Check the battery voltage to see if it is at an acceptable level ---> Called by UpdataSensors()

  if (!Thrusters) return;  // Thrusters are off, can not read battery voltage
   
  motor_PORT.update();
  motor_STAR.update();

  if((motor_PORT.voltage() == 0) || (motor_STAR.voltage() == 0 )) return;                                              // Thruster drop out ---> Exit function  
  else if((motor_PORT.voltage() < 10.5) || (motor_STAR.voltage() < 10.5)) {  String error = Time;                      // Batteries are past the safe discharge level --> Turn boat off
                                                                             error += "Battery voltage too low!!";                                                                             error += "\t";
                                                                             error += "Star-Bat: ";
                                                                             error += String(motor_STAR.voltage());
                                                                             error += "\t";
                                                                             error += "Port-Bat: ";
                                                                             error += String(motor_PORT.voltage());

                                                                             File dataFile = SD.open(SD_FILE, FILE_WRITE);
                                                                             dataFile.println(error);
                                                                             dataFile.close();

                                                                             digitalWrite( POWER_PIN, HIGH );
                                                                      }
  else if((motor_PORT.voltage() < 11) || (motor_STAR.voltage() < 11)) Battery = LOW;                          // Batteries are low ---> Switch state to low
  else if(Battery == LOW && ((motor_PORT.voltage() > 12) || (motor_STAR.voltage() > 12 ))) Battery = HIGH;        // Batteries are full --> Switch state to high
  else;

}

void Drive(void) { // Vehicle control and thruster commands  ---> Gets called when the boat is driving during the day time or at night when the boat drifts out of the geo-fence
  
  static int PortSignal;     // Port Thruster Signals
  static int portThrust;
  static int StarSignal;     // Starbord Thruster Signals
  static int starThrust;
  
  static float t_now;  // Current time is seconds
  static float t_last; // Previous time in seconds

  t_now = millis()/1000.0;
  if (( t_last - t_now ) > 4,000,000) t_last = 0; // Set previous time to 0 if millis() overflows

// Update navigation at 4Hz =================================================================================================================================    
  static bool upDateNave;
  if ((millis()/10) % 25 == 0 && upDateNave == true) { 

      upDateNave = false;

      static int throttleDiff;
      static float deltaC;
      static float heading;


      // If there is a GPS fix Calculate bearing based on GPS coordinates, otherwise, stop the boat -------------------------------------------------------------------------------------
      if (GPS.fix ) { if (!pointInPolygon(GPS.longitudeDegrees,GPS.latitudeDegrees)) GeoFence(); //If lat and long leave the fenced polygon,  preform subroutine to get robot back into fence
                      else GetBearing(); // Perform waypoint Navigation
                       
                      heading = GetHeading();

                      // Determine Angle between the boat's heading and the desired direction of travel and update thrusters =======================================================
                      if      (bearing < 180 && (bearing+180) < heading )    { deltaC = 360 - heading + bearing;}
                      else if (180 < bearing && heading < (bearing - 180))   { deltaC = bearing - 360 - heading;}
                      else                                                   { deltaC = bearing - heading;}


                       // Determine Signal to the thrusters -----------------------------------------------------------------------------------------------------------
                       // Pivot Clockwise
                       if ( deltaC >= ANGLE ) { PortSignal =  TURN;
                                                StarSignal = -TURN;
                                               }


                       // Pivot Counter Clockwise
                       else if ( deltaC <= -ANGLE ) { PortSignal = -TURN;
                                                      StarSignal =  TURN;
                                                     }


                       // Make and arcing turn towards the bearing
                       else if ( abs( deltaC ) >= 2 ) { throttleDiff = map( deltaC, -ANGLE, ANGLE, -TURN, TURN);
                                                      PortSignal = ( AVE_THRUST + throttleDiff );
                                                      StarSignal = ( AVE_THRUST - throttleDiff );
                                                     }

                       else { PortSignal = AVE_THRUST;    // Go straight
                              StarSignal = AVE_THRUST;
                              }
                     }
      else { PortSignal = 0;
             StarSignal = 0;
            }

      // Project thruster commands onto the calibrated range of values for each thruster
      if ( PortSignal >= 0 ) PortSignal *= ( portMax / 100.0 );
      else                   PortSignal *= ( portMin / 100.0 );

      if ( StarSignal >= 0 ) StarSignal *= ( starMax / 100.0 );
      else                   StarSignal *= ( starMin / 100.0 );

    }
  else if ((millis()/10) % 100 == 0 && upDateNave == false);
  else upDateNave = true;

  // Ramp the truster inputs-------------------------------------------------
  portThrust = Vel_Ramp( portThrust, PortSignal, t_now, t_last );
  starThrust = Vel_Ramp( starThrust, StarSignal, t_now, t_last ); 

  // Constrain the thrusters signal rang to the band of accepted values ------------
  constrain( portThrust, MIN_THRUSTER_INPUT, MAX_THRUSTER_INPUT );
  constrain( starThrust, MIN_THRUSTER_INPUT, MAX_THRUSTER_INPUT );

  // Send Signal to the thrusters -----------------------------------------
  motor_PORT.set( portThrust );
  motor_STAR.set( starThrust );

  t_last = t_now; // Update previous time
  
}

void ElectronOff(void) { // Turn the Particle electron Off ---> Called in main loop

  digitalWrite( ELECTRON_POWER_PIN, LOW);
  Cell = false;

  String data = Time;
  data += "   Electron Off";

  File dataFile = SD.open(SD_FILE, FILE_WRITE);
  dataFile.println(data);
  dataFile.close();
}

void ElectronOn(void) { // Turn the Particle Electron On  ---> Called in main loop

  digitalWrite( ELECTRON_POWER_PIN, HIGH);
  Cell = true;

  String data = Time;
  data += "   Electron On";

  File dataFile = SD.open(SD_FILE, FILE_WRITE);
  dataFile.println(data);
  dataFile.close();
}

void GeoFence(void){ // Manage waypoint index  ---> Called by Drive()

  pindex = gpsindex;
    
  gpsindex = sizeof(desLat)/sizeof(float)-1; // Change GPS index to the last Waypoint in the array which should be in the center of the Geo-fence
  
  GetBearing();
  
  gpsindex = pindex;
}

void GetBearing(void){  // Calculate bearing and distance from current location to the next waypoint  ---> Called by Drive()
  
  float deltaLon = desLon[gpsindex] - GPS.longitudeDegrees; //calculate a difference in lat and long for angle
  float deltaLat = desLat[gpsindex] - GPS.latitudeDegrees;

 
  bearing = atan2(deltaLon, deltaLat);   //Calculate desired angle in quadrant 1 or 4
  bearing *= ( 180.0 / PI );        //covert to degrees


  deltaLon *= LON_TO_METER;   // Convert distances to Meters
  deltaLat *= LAT_TO_METER;

  distance = sqrt(deltaLon * deltaLon + deltaLat * deltaLat); //Distance for waypoint switching within radius
}

float GetHeading(void) { // Read tilt compensated compass heading of GPS angle  ---> Called in Drive() and UpdataSensors()

  if (GPS.speed >= 0.5 )  return GPS.angle;
  
   // Get sensor data and normalize values --------------------------------------------------------------------------------
  sensors_event_t event; 
  
  // Accelerometer values. Gravity must be negative downward when the sensor is right side up.
  accel.getEvent(&event);
  double Gx = -(event.acceleration.x);  // Assign X accelerometer value here
  double Gy = -(event.acceleration.y);  // Assign Y accelerometer value here
  double Gz = -(event.acceleration.z);  // Assign Z accelerometer value here

  double Gmag = sqrt( Gx*Gx + Gy*Gy + Gz*Gz );
  Gx /= Gmag;
  Gy /= Gmag;
  Gz /= Gmag;


  // Magnetometer values. The Z component of the reading must be negative when the sensor is right side up.
  mag.getEvent(&event);
  double Bx = (event.magnetic.x); // Assign X magnetic value here
  double By = (event.magnetic.y); // Assign Y magnetic value here
  double Bz = (event.magnetic.z); // Assign Z magnetic value here
  
  double Bmag = sqrt( Bx*Bx + By*By + Bz*Bz );
  
  Bx /= Bmag;
  By /= Bmag;
  Bz /= Bmag;


  // Find world coordinate frame in terms of the local coordinate frame -----------------------------------------
  // Find East, cross product of the gravity vector and magnetic vector
  double Ex = Gy*Bz - Gz*By;
  double Ey = -(Gx*Bz - Gz*Bx);
  double Ez = Gx*By - Gy*Bx;

  double Emag = sqrt( Ex*Ex + Ey*Ey +Ez*Ez );
  Ex /= Emag;
  Ey /= Emag;
  Ez /= Emag;

  // Find North, cross product of the east vector and gravity vector
  double Nx = Ey*Gz - Ez*Gy;
  double Ny = -(Ex*Gz - Ez*Gx);
  double Nz = Ex*Gy - Ey*Gx;

  double Nmag = sqrt( Nx*Nx + Ny*Ny +Nz*Nz );
  Nx /= Nmag;
  Ny /= Nmag;
  Nz /= Nmag;


  // Calculate vector for the direction of travel ----------------------------------------------------------------
  // Project direction of travel vector into the North - East plain
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


  // Calculate heading -------------------------------------------------------------------------------------------
  float heading = atan2(Y, X);

  heading = heading * 180.0/PI;
  
  if ( heading < 0 ) heading += 360;
 
  // Compensate for declination
  heading += DECLINATION; 
  if ( heading > 360 ) heading -= 360;

  return heading;
}

void GPSindex(void) { // Change waypoints once the current waypoint is reached ---> Called in main loop

  if(distance < WAYPOINT_RADIUS ) gpsindex ++; //next waypoint

  if (gpsindex >= (sizeof(desLat)/sizeof(float) -1) ) gpsindex = 0; //loop waypoints when final waypoint reached. The very last waypoint in the array is in the center of the Geo-fence,This will skip that waypoint
}

int Hour(void) { // Convert GPS hour into local time and manage DayTime state Variable ---> Called durning Start up and by UpdateSensors()

  int hour;

  if (GPS.hour >= TIMEZONE) hour = GPS.hour - TIMEZONE;
  else hour = 24 - TIMEZONE + GPS.hour;

  if( (hour <= SUNRISE) || (hour >= SUNSET) ) DayTime = false;
  else DayTime = true;

  return hour;
}

void MonitorGPS(void) { // Monitor GPS for drop-outs --> Called in main loop
  
  static bool GPSFIX;

  if (GPSFIX && !GPS.fixquality) { String data = Time;
                                   data += "   GPS fix lost";

                                   File dataFile = SD.open(SD_FILE, FILE_WRITE);
                                   dataFile.println(data);
                                   dataFile.close();
                                  }
                            
   else if (!GPSFIX && GPS.fixquality > 0) { String data = Time;
                                             data += "   GPS fix acquired";

                                             File dataFile = SD.open(SD_FILE, FILE_WRITE);
                                             dataFile.println(data);
                                             dataFile.close();
                                            }
  else;

  if(GPS.fixquality == 0) GPSFIX = false;
  else GPSFIX = true;

}

bool pointInPolygon( float x, float y ){ // Geo - Fence  ---> Called in main loop and Drive()
  
  int i, j = sizeof(polyX)/sizeof(float) - 1; //set index for i=0 and j
  bool oddNodes = false; //variable for false return
 
  for ( i = 0; i < (sizeof(polyX)/sizeof(float)); i++ ) { if ( (polyY[i] < y && polyY[j] >= y || polyY[j] < y && polyY[i] >= y) //statement for points in side slanted sides of polygon
                                                                &&  (polyX[i] <= x || polyX[j] <= x) ){ //makes sure polygon is within x boundaries
                                                                                                       oddNodes ^= ( polyX[i] + (y - polyY[i]) / (polyY[j] - polyY[i]) * (polyX[j] - polyX[i]) < x );  //point in polygon method, determine odd nodes
                                                                                                       } //if the number of nodes crossed is odd, point is in polygon
                                                         j = i; //reset index
                                                         }
  
  // Set the state variable for the Geo-Fence ------------------------------------------------------------                                                      
  if (!oddNodes ) InFence = false;                      // If the boat leaves the geo-fence set state to false
  else InFence = true;      

  return oddNodes; //return value of odd nodes crossed in binary, 1 true 0 false
}

void Stop(void) { // Stops the Vehicle ---> Gets Called by ThrustersOff()

  int i=0;
  int signal1;
  int signal2;

  while ( i <= 1000 ) {  GPS.parse(GPS.lastNMEA());

                         if ( GPS.speed > 1 ) {  signal1 = -AVE_THRUST; // Set thrusters to run backwards
                                                 signal2 = -AVE_THRUST;
                                                }

                         else { signal1 = 0; // Set thrusters to stop
                                signal2 = 0;
                                }                    

                         signal1 *= ( portMin / 100.0 );
                         signal2 *= ( starMin / 100.0 );

                         // Send Signal to the thrusters -----------------------------------------
                         motor_PORT.set( signal1 );
                         motor_STAR.set( signal2 );

                         i++;
                         }
}

float Thermistor(int Raw){ //This function calculates temperature from ADC count ---> Called  bu UpdataSensors()
  long Resistance; 
  float Resistor = 14890;  //Fixed resistor for Thermistor
  float Temp;  // Dual-Purpose variable to save space.
  Resistance=( Resistor*Raw /(1024-Raw)); 
  Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later
  Temp = 1 / (0.00102119 + (0.000222468 * Temp) + (0.000000133342 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  // Convert Kelvin to Celsius                      
  return Temp;                                      // Return the Temperature
}

void ThrusterCalibrate(void) { // Calibrates the thrusters at startup --> Called by setup()

/*  Calibrates thrusters by finding the rang of RPM for both thrusters and confining each thrusters input range to mach each other
 * 
 */
 
  byte i = 0; 
  
  int sign = 0;
  int portThrust = 0; // Port Thruster Signals
  int starThrust = 0; // Starboard Thruster Signals
  
  float portAVE = 0;
  float starAVE = 0;
  float signalAVE = 0;
  
  float PortRPM[10];
  float StarRPM[10];
  float Signal[10];

  
  for ( int x = MIN_THRUSTER_INPUT; x <= MAX_THRUSTER_INPUT; x += (MAX_THRUSTER_INPUT/4) ) {  // Find RPMs for corresponding thruster inputs over the range of acceptable thruster inputs
                                                   
                                                   float t_now;  // Current time is seconds
                                                   float t_last; // Previous time in seconds

                                                   t_now = millis()/1000.0; // Set times
                                                   t_last = t_now;

                                                   while ( (portThrust != x) && (starThrust != x) ) { // Ramp the truster inputs-------------------------------------------------

                                                                                                                        t_now = millis()/1000.0;
                                                                                                                        
                                                                                                                        portThrust = Vel_Ramp( portThrust, x, t_now, t_last );
                                                                                                                        starThrust = Vel_Ramp( starThrust, x, t_now, t_last ); 

                                                                                                                        // Constrain the thrusters signal rang to the band of accepted values ------------
                                                                                                                        constrain( portThrust, MIN_THRUSTER_INPUT, MAX_THRUSTER_INPUT );
                                                                                                                        constrain( starThrust, MIN_THRUSTER_INPUT, MAX_THRUSTER_INPUT );

                                                                                                                        // Send Signal to the thrusters -----------------------------------------
                                                                                                                        motor_PORT.set( portThrust );
                                                                                                                        motor_STAR.set( starThrust );

                                                                                                                        t_last = t_now;

                                                                                                                        delay (100);
                                                                                                                      }
                                                   
                                                   int t = 0;
                                                   while ( t <= 5 ) { motor_PORT.set( portThrust );
                                                                      motor_STAR.set( starThrust );
                                                                      delay(1000);
                                                                      t++;
                                                                      }

                                                   // Record RPMs and corresponding signal ------------------------------------------------------------------------------------
                                                   
                                                  
                                                   motor_PORT.update();
                                                   motor_STAR.update();

                                                   if ( x < 0 ) sign = -1;
                                                   else sign = 1;

                                                   PortRPM[i] = sign * motor_PORT.rpm();
                                                   portAVE += PortRPM[i];
                                                   
                                                   StarRPM[i] = sign * motor_STAR.rpm();
                                                   starAVE +=  StarRPM[i];
                                                   
                                                   Signal[i] = x;
                                                   signalAVE += x;

                                                   /* Debugging Statements
                                                   Serial.print("Index: "), Serial.print(i);
                                                   Serial.print("\tSignal: "), Serial.print(x);
                                                   Serial.print("\tSign: "), Serial.print(sign);
                                                   Serial.print("\tPort RPM: "), Serial.print(PortRPM[i]);
                                                   Serial.print("\tStar RPM: "), Serial.println(StarRPM[i]);
                                                   */

                                                   i++;

                                                   delay(500);
                                                  }

  // Stop Thrusters
  byte s = 0;
  motor_PORT.set( s );
  motor_STAR.set( s );

  // Finish averaging calculations
  portAVE /= i;
  starAVE /= i;
  signalAVE /= i;

  // Find the Highest Low-rang RPM
  int minRPM;
  if ( PortRPM[0] < StarRPM[0] ) minRPM = StarRPM[0];
  else minRPM = PortRPM[0];

  // Find the Lowest High-range RPM
  int maxRPM;
  if ( PortRPM[i-1] < StarRPM[i-1] ) maxRPM = PortRPM[i-1];
  else maxRPM = StarRPM[i-1];


  // Do linear regression on RPMs vs Signal to find the low-range and high-range values that correspond to the limiting values =========================================
  
  float portSlope = 0;
  float portDenom = 0;
  float portInt = 0;
  
  float starSlope = 0;
  float starDenom = 0;
  float starInt = 0;

  // Calculate Slopes ---------------------------------------------------------------------------------------------------------------------------------------------
  for (int j = 0; j < i; j++) {  portSlope += ( PortRPM[j] - portAVE ) * ( Signal[j] - signalAVE );
                                 portDenom += ( PortRPM[j] - portAVE ) * ( PortRPM[j] - portAVE );

                                 starSlope += ( StarRPM[j] - starAVE ) * ( Signal[j] - signalAVE );
                                 starDenom += ( StarRPM[j] - starAVE ) * ( StarRPM[j] - starAVE );
                                }

  portSlope /= portDenom;
  starSlope /= starDenom;  

  
  // Calculate Intercepts -------------------------------------------------------------------------------------------------------------------------------------------
  portInt = signalAVE - portSlope * portAVE;
  starInt = signalAVE - starSlope * starAVE;


  // Calculate Maximum and Minimum thruster inputs ------------------------------------------------------------------------------------------------------------------
  portMax = portSlope * maxRPM + portInt;
  portMin = abs( portSlope * minRPM + portInt );

  starMax = starSlope * maxRPM + starInt;
  starMin = abs( starSlope * minRPM + starInt );


/*Debugging Statements -----------------------------------------------------------------------------
  Serial.print("\nSignal Ave: "), Serial.print(signalAVE);
  Serial.print("\tPort Ave: "), Serial.print(portAVE);
  Serial.print("\tStar Ave: "), Serial.println(starAVE);

  Serial.print("\nMax RPM: "), Serial.println(maxRPM);
  Serial.print("Min RPM: "), Serial.println(minRPM);
  
  Serial.print("\nPort Slope: "), Serial.print(portSlope);
  Serial.print("\tInt: "), Serial.print(portInt);

  Serial.print("\nStar Slope: "), Serial.print(starSlope);
  Serial.print("\tInt: "), Serial.println(starInt);

  Serial.println("\n-----------------  Calibration Values  ----------------------");
  Serial.print("PortMax / Min: "), Serial.print(portMax),Serial.print(", "), Serial.println(portMin);
  Serial.print("StarMax / Min: "), Serial.print(starMax),Serial.print(", "), Serial.println(starMin);
*/

}

/* Thruster Check
void ThrusterCheck(void) { // Check for thruster drop out ---> Called in main loop

  if (!Thrusters) return; // Exit this function if the trusters are turned off

  motor_PORT.update();
  motor_STAR.update();
  
  int signal = 0;
  
  // Check for thruster drop out --> Stop thruster if detected
  if((motor_PORT.voltage() == 0) || (motor_STAR.voltage() == 0 ))  { motor_PORT.set( signal );
                                                                     motor_STAR.set( signal );

                                                                     String data = Time;
                                                                     data += "   Thruster Drop out detected";
                                                                     data += "\tPort: ";
                                                                     data += String( motor_PORT.voltage() );
                                                                     data += "\tStar: ";
                                                                     data += String( motor_STAR.voltage() ); 

                                                                     File dataFile = SD.open(SD_FILE, FILE_WRITE);
                                                                     dataFile.println(data);
                                                                     dataFile.close();
                                                                    
                                                                     while ((motor_PORT.voltage() == 0) || (motor_STAR.voltage() == 0 )) { motor_PORT.update();
                                                                                                                                          motor_STAR.update();
                                                                                                                                         }

                                                                     TimeKeeping();
                                                                     data = Time;
                                                                     data += "   Thruster Drop out cleared";

                                                                     dataFile.println(data);
                                                                     dataFile.close();
                                                                    }
  else return;

}
*/

/* ThrusterData --> Called in main loop
  void ThrusterData(void) { // Reads data coming off the thruster via I2C --> Called in main loop

  motor_PORT.update();
  motor_STAR.update();

  Serial.print("\n\n\nESC PORT: ");
  if(motor_PORT.isAlive()) Serial.print("OK\t\t"); 
  else Serial.print("NA\t\t");
  Serial.print(motor_PORT.rpm());Serial.print(" RPM\t\t");
  Serial.print(motor_PORT.voltage());Serial.print(" V\t\t");
  Serial.print(motor_PORT.current());Serial.print(" A\t\t");
  Serial.print(motor_PORT.temperature());Serial.print(" `C");
  Serial.println();

  Serial.print("\nESC STAR: ");
  if(motor_STAR.isAlive()) Serial.print("OK\t\t"); 
  else Serial.print("NA\t\t");
  Serial.print(motor_STAR.rpm());Serial.print(" RPM\t\t");
  Serial.print(motor_STAR.voltage());Serial.print(" V\t\t");
  Serial.print(motor_STAR.current());Serial.print(" A\t\t");
  Serial.print(motor_STAR.temperature());Serial.print(" `C");
  Serial.println();
  
}
*/

void ThrustersOff(void) { // Turn off Thruster SSRs and Manage Thruster State variable ---> Called in main loop

  Stop();

  // Set thruster signal to stop for dry testing ------
  int signal1 = 0;
  int signal2 = 0;

  // Send Signal to the thrusters -----------------------------------------
  motor_PORT.set( signal1 );
  motor_STAR.set( signal2 );

  delay(10);

  // Turn off the SSRs for thruster power and I2C communication
  digitalWrite( THRUSTER_I2C_PIN, LOW );
  digitalWrite( THRUSTER_POWER_PIN, LOW);

  Thrusters = false; // Manage state variable

  String data = Time;
  data += "   Thrusters Off";

  File dataFile = SD.open(SD_FILE, FILE_WRITE);
  dataFile.println(data);
  dataFile.close();
  
  }

void ThrustersOn(void) {  // Turn on Thruster SSRs and Manage Thruster State variable  ---> Called in main loop
  
  // Turn on the SSRs for thruster power and I2C communication
  digitalWrite( THRUSTER_POWER_PIN, HIGH);
  digitalWrite( THRUSTER_I2C_PIN, HIGH ); 

  Thrusters = true;  // Manage state variable
  
  delay(10);

  // Wait for thrusters to become responsive 
  while ( (motor_PORT.voltage() == 0.0) || (motor_STAR.voltage() == 0.0)){ motor_PORT.update();
                                                                           motor_STAR.update();
                                                                           
                                                                           wdt_reset();     // Reset Watch dog timer
                                                                           }

  for ( int i = 0; i <= 10; i++ ) { // Set thruster signal to stop for reactivation
                                    int signal1 = 0;
                                    
                                    // Send Signal to the thrusters
                                    motor_PORT.set( signal1 );
                                    motor_STAR.set( signal1 );

                                    wdt_reset();     // Reset Watch dog timer

                                    delay(100);
                                    }

  String data = Time;
  data += "   Thrusters On";

  File dataFile = SD.open(SD_FILE, FILE_WRITE);
  dataFile.println(data);
  dataFile.close();

  }

void TimeKeeping(void) { // Time keeping, updates 'time' and 'Time' --> Called in main loop, UpdateSensors() and error messages
                         
                         time = millis()/1000; // Time in seconds
                         
                         Time = String(GPS.month);
                         Time += "/";
                         Time += String(GPS.day);
                         Time += "/";
                         Time += String(GPS.year);
                         Time += "   ";
                         Time += String(Hour());
                         Time += ":";
                         Time += String(GPS.minute);
                         Time += ":";
                         Time += String(GPS.seconds);
                         
                        }
        
void UpdateSensors(void) { // Reads the sensors, takes an average of 10 readings from analog sensors, writes the values to the SD card and transmits them to the electron vis I2C ---> Called in main loop


  // Internal Measurements ---------------------------------------------------------------------------- 

  if ( htu.readTemperature() > 50 ) { File dataFile = SD.open(SD_FILE, FILE_WRITE);
                                      dataFile.println("Temperature too high");
                                      dataFile.close();

                                      digitalWrite(POWER_PIN, HIGH);
                                    }
                                          
  if ( htu.readHumidity() > 75 ) { File dataFile = SD.open(SD_FILE, FILE_WRITE);
                                   dataFile.println("Humidity too high");
                                   dataFile.close();

                                   digitalWrite(POWER_PIN, HIGH);
                                  }
  

  // Water Quality Measurements ----------------------------------------------------------------------
  const float AverageRange = 10.0;  // Number of sensor readings to average over

  
  // Initialize Sampling variables
  static float Sample1 = 0;
  static float Sample2 = 0;
  static float Sample3 = 0;
  static float Sample4 = 0;
  static float Sample5 = 0;
  static float Sample6 = 0;

  // Take the average reading from the sensor

  analogReference(EXTERNAL);

  analogRead(SENSOR_1PIN);
  delay(100);
  for(byte i=0; i < AverageRange; i++) { Sample1 += analogRead(SENSOR_1PIN);
                                         delay(10);                                  // Delay for Stability
                                          }

  analogRead(SENSOR_2PIN);
  delay(100);
  for(byte i=0; i < AverageRange; i++) { Sample2 += analogRead(SENSOR_2PIN);
                                         delay(10);
                                         }

  analogRead(SENSOR_3PIN);
  delay(100);
  for(byte i=0; i < AverageRange; i++) { Sample3 += analogRead(SENSOR_3PIN);
                                         delay(10);
                                         }


  analogReference(DEFAULT);
  digitalWrite( SENSOR_POWER_PIN, LOW);

  Sample1 /= AverageRange;
  Sample2 /= AverageRange;
  Sample3 /= AverageRange;

  // Convert the sensor data into physical Data based on the sensor's transfer functions=============================
  
  // Sensor calibration Values -----------------------------------
  const float Sal_slope = 16.3;
  const float Sal_int = 0.0;
  const float pH_slope = -10.0;
  const float pH_int = 22.5;

  // Convert sensor 1 into salinity data
  static float Salinity = VOLTAGE( Sample1 ) * Sal_slope + Sal_int;

  // Convert sensor 2 into Temperature
  static float Temp = Thermistor( Sample2 ); 
  
  // Convert sensor 3 into pH
  static float pH =  VOLTAGE( Sample3 ) * pH_slope + pH_int;

  
  // Assembling sensor data as a string for transmission ----------------------------------------------------------------------------
  String dataString = "Data: ";

  dataString += Time;
  dataString += "\t";

  if( GPS.fix ) { dataString += "GPS: ";
                  dataString += String(GPS.fixquality);
                  dataString += ",";
                  dataString += String((int)GPS.satellites);
                  dataString += ",";
                  dataString += String(GPS.latitudeDegrees,6);
                  dataString += ",";
                  dataString += String(GPS.longitudeDegrees,6);
                  dataString += "\t";
  
                  if(InFence) dataString += "In";
                  else  dataString += "Out";
                  }
  else dataString += "No GPS Fix";

  dataString += "\t";

  if(DayTime) dataString += "Day";
  else  dataString += "Night";

  dataString += "\tWP: ";
  dataString += String(distance);
  dataString += ",";
  dataString += String(GetHeading());  
  dataString += ",";
  dataString += String(GPS.speed);
  dataString += "\t";  

   if(Thrusters){ dataString += "Thrusters ON: ";
  
                  motor_PORT.update();
                  motor_STAR.update();
;
                  dataString += String(motor_PORT.current());
                  dataString += ",";
                  dataString += String(motor_PORT.voltage());
                  dataString += ",";
                  dataString += String(motor_STAR.current());
                  dataString += ",";
                  dataString += String(motor_STAR.voltage());
                  }
  else  dataString += "Thrusters OFF";

  dataString += "\t";
  dataString += String(htu.readTemperature());
  dataString += ",";
  dataString += String(htu.readHumidity());

  dataString += "\t";
  dataString += String(Temp);
  dataString += ", ";
  dataString += String(Salinity);
  dataString += ", ";
  dataString += String(pH);

  delay(10); // Delay for stability


// Transmit data to the electron ---------------------------------------------------------------------------

  // Convert the data String into a character array for I2C transmission
  char transmit[ (dataString.length()+1) ];
  dataString.toCharArray(transmit, (dataString.length()+1) );

  wdt_reset();  // Reset Watch dog timer in case I2C takes too long

  // Transmit the character array via I2C
  byte i = 0;
  Wire.beginTransmission(ELECTRON_ADDRESS);
  while ( i <= dataString.length() ) {  if (i % 30 == 0) {  Wire.endTransmission();     // I2C buffer can hold a little over 30 characters. This ends and restarts the I2C every 30 characters as to not exceed the buffer
                                                            delay(10);
                                                            Wire.beginTransmission(ELECTRON_ADDRESS);
                                                            }
                                                    
                                       Wire.write(transmit[i]);
                                       i++;
                                     }
                                                          
  Wire.endTransmission();
  
}

int Vel_Ramp( int velocity, int target, float t_now,  float t_last ) { // Velocity Ramp function --> Called in Drive()

  int sign = 0; 
  
  float Step = RAMP_STEP * ( t_now - t_last );     // Calculate step size
  int error = velocity - target;                  // Calculate the difference between the target velocity and the current velocity

/*  Serial.print("\nVelocity: "), Serial.print(velocity);
  Serial.print("\tTarget: "), Serial.println(target);
  Serial.print("t_now: "), Serial.print(t_now,6);
  Serial.print("\tt_last: "), Serial.println(t_last,6);
  Serial.print("Step: "), Serial.print(Step);
  Serial.print("\tError: "), Serial.println(error);
*/

  if (abs(Step) > abs(error)) return target;      // If the distance to he target velocity is less than the step size return the target velocity
  
  if (velocity < target) sign = 1;                // Calculate the direction of the velocity step
  else sign = -1; 

  return velocity + sign * Step;
  
}


// INTERUPTS ********************************************************************************************************************************

ISR(WDT_vect) { // Watchdog timer interrupt.  

  // Serial.println("Woof Woof!!");
  
  File dataFile = SD.open(SD_FILE, FILE_WRITE);
  dataFile.println("\n\nWoof Woof!!");
  dataFile.close();
  
}

