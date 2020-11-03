/* This is a general perpose script that findes the minimum value of
   the eliments in a vector and its location in the vector
   Jacob Anderson
   March 31, 2015
   For: Space Grant Robot 1 "SG1"
   */

// Define Vector and number of bins
const int VectorNumber = 11;                                    // Number of bins in the vector
int Vector[VectorNumber] = {-10,12,12,3,54,15,86,88,33,39,10};  // Vector with values

// Variable used in the vector manipulations
int place;              // Location of the minimum value elimant in the vectoer. Eliment 1 of the vector is 0.
int angle = 0;          // Angle of the minimum value from center
int VectorMin = 1000;   // Minimum value of the vector. Set very high initialy so that logic statments will reduce
                           //the value to the minimum value of the vector

// ==== Set up =================================================================================================
void setup (){Serial.begin(9600);}

// ==== Main loop ==============================================================================================
void loop(){
  PrintVector(); // Display the vector
   
  
  // Find the minimum Value in vector
  for(int b =0; b < VectorNumber; b++){  
    if( Vector[b] < VectorMin){ VectorMin = Vector[b]; place = b;}
   } 
  
  // Calculate angle in degrees from center. + angle is to the left, - angle is to the right
  if      ( place <= 4) { angle = -(place-5)*10;}
  else if ( place ==  5) { angle = 0;}
  else if ( place == 12) { Serial.print("Something wrong!!!!");}
  else                  { angle = (5-place)*10;}
  
   // Display the calculated values
   Serial.print("Vector Min: ");
   Serial.print(VectorMin);
   Serial.print("\t Place: ");
   Serial.print(place);
   Serial.print("\t Angle: ");
   Serial.println(angle);
   
   delay (1000);
}

// ==== Subrutiens ==============================================================================================
// ==============================================================================================================
void PrintVector(){  // Prints the vector
  
  Serial.print("Vector: [ ");
  for (int i=0;i<(VectorNumber-1);i++)
  { Serial.print(Vector[i]);
    Serial.print(", ");}
    Serial.print(Vector[(VectorNumber-1)]);
    Serial.println(" ]");

}

