/*
VernierThermistor (v 2014.09)
Reads the temperature from a Vernier Stainless Steel Temperature Probe (TMP-BTA)
or Surface Temperature Sensor (STS-BTA) connected to the BTA 1 connector. 
As written, the readings will be displayed every half second. Change the variable 
TimeBetweenReadings to change the rate.

We use the Steinhart-Hart equation (in the function Thermistor) to determine temperature 
from the raw A/D converter reading. Because of the use of log functions, in the Steinhart-Hart 
equation, this sketch requires the math.h library. 

See www.vernier.com/engineering/stem/sensors/temperature-sensor/
for more information on how thermistors are read.

See www.vernier.com/arduino for more information.
 */
#include <math.h>
int ThermistorPIN =A1;// Analog Pin 0
int TimeBetweenReadings = 500; // in ms
int ReadingNumber=0;

void setup() 
{
  pinMode(3, OUTPUT);
  digitalWrite(3,HIGH);
  
  Serial.begin(9600);
  Serial.println("Vernier Format 2");
  Serial.println("Temperature Readings taken using Ardunio");
  Serial.println("Data Set");
  Serial.print("Time");//long name
  Serial.print("\t"); //tab character
  Serial.println ("Temperature"); 
  Serial.print("t");
  Serial.print("\t"); //tab character
  Serial.println ("Temp"); //short name
  Serial.print("seconds");
  Serial.print("\t"); // tab character
  Serial.println ("degrees C"); 
}
void loop() 
{
  float Time;
  int Count; //reading from the A/D converter (10-bit)
  float Temp; //the print below does the division first to avoid overflows
  
  Serial.print(ReadingNumber/1000.0*TimeBetweenReadings); 
  
  Count=analogRead(ThermistorPIN);       // read count from the A/D converter 
  // Serial.print("A0:"); 
  // Serial.print(Count);
  
  Temp=Thermistor(Count);       // and  convert it to CelsiusSerial.print(Time/1000); //display in seconds, not milliseconds                       
  
  Serial.print("\t"); //tab character
  Serial.println(Temp,1);   // display temperature to one digit                                
  
  delay(TimeBetweenReadings); // Delay a bit... 
  
  ReadingNumber++;  
}



float Thermistor(int Raw) //This function calculates temperature from ADC count
{
 /* Inputs ADC count from Thermistor and outputs Temperature in Celsius
 *  requires: include <math.h>
 * There is a huge amount of information on the web about using thermistors with the Arduino.
 * Here we are concerned about using the Vernier Stainless Steel Temperature Probe TMP-BTA and the 
 * Vernier Surface Temperature Probe STS-BTA, but the general principles are easy to extend to other
 * thermistors.
 * This version utilizes the Steinhart-Hart Thermistor Equation:
 *    Temperature in Kelvin = 1 / {A + B[ln(R)] + C[ln(R)]^3}
 *   for the themistor in the Vernier TMP-BTA probe:
 *    A =0.00102119 , B = 0.000222468 and C = 1.33342E-7
 *    Using these values should get agreement within 1 degree C to the same probe used with one
 *    of the Vernier interfaces
 * 
 * Schematic:
 *   [Ground] -- [thermistor] -------- | -- [15,000 ohm resistor] --[Vcc (5v)]
 *                                     |
 *                                Analog Pin 0
 For the circuit above:
 * Resistance = ( Count*RawADC /(1024-Count))
 */
 long Resistance; 
 float Resistor = 15000; //fixed resistor
// the measured resistance of your particular fixed resistor in
// the Vernier BTA-ELV and in the SparkFun Vernier Adapter Shield 
// is a precision 15K resisitor 
  float Temp;  // Dual-Purpose variable to save space.
  Resistance=( Resistor*Raw /(1024-Raw)); 
  Temp = log(Resistance); // Saving the Log(resistance) so not to calculate  it 4 times later
  Temp = 1 / (0.00102119 + (0.000222468 * Temp) + (0.000000133342 * Temp * Temp * Temp));
  Temp = Temp - 273.15;  // Convert Kelvin to Celsius                      
  return Temp;                                      // Return the Temperature
}




