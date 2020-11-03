// This skrip recordes values for a sensor and averages them to return a smootherer reading

const int numberReadings = 20;
int readings[numberReadings];
int index = 0;
int total = 0;
int average = 0;

int inputPin = A0;

void setup () { Serial.begin(9600);
  // initialize all readings to 0
  for (int thisReading = 0; thisReading < numberReadings; thisReading++)
    readings[thisReading] = 0;
}

void loop(){
  sensor();
  delay(100);
}




void sensor(){
    total = total - readings[index];        // Subtract the last reading from the array
   readings[index] = analogRead(inputPin);  // Read from the sensors
   total = total + readings[index];         // Add the reading from to the total.
   index = index + 1;                       // Advance to the next position in the array
   
   if (index >= numberReadings) index = 0;  // Wrap around to the begining o fthe aray once your at the end
   
   average = total / numberReadings;        // Calculate average sensor value
   Serial.println(average);                   // Display average
}
