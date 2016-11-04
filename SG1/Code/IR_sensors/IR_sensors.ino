/* This script is to read the three IR sensors on the Space Grant 1 "SG1" robot
   and smooth out the readings through averaging.
   Jaob Anderson
   March 31, 2015
   */

// Define Vectors
const int IR_vec_number = 10;    // Number of elements in the vectors
int LeftIRvec[IR_vec_number];    // Left IR vector
int CenterIRvec[IR_vec_number];  // Center IR vector
int RightIRvec[IR_vec_number];   // Right IR vector

// Define Vector Variables
int LeftIRtotal = 0;    // Sum of each IR Vector
int CenterIRtotal = 0;
int RightIRtotal = 0;

int index = 0;

// Average value of each IR vector
int LeftIRave = 0;     // Average on flat ground 280
int CenterIRave = 0;   // Average on flat ground 316
int RightIRave = 0;    // Average on flat ground 278 - See more values at end of script

// Define Inputs

int LeftIR   = A0;  // Left IR sensor assigned to pin A0
int CenterIR = A1;  // Center IR sensor assigned to pin A1
int RightIR  = A2;  // Right IR sensor assigned to pin A2


// === Set up rutien ==================================================================================================
// ====================================================================================================================
void setup () { Serial.begin(9600);

// -- Set up inputs ----------------------------------------------
  pinMode(LeftIR,   INPUT);
  pinMode(CenterIR, INPUT);
  pinMode(RightIR,  INPUT);
  
// --- initialize all readings to 0 ------------------------------
  for (int Zero = 0; Zero < IR_vec_number; Zero++) {
   LeftIRvec[Zero]   = 0;
   CenterIRvec[Zero] = 0;
   RightIRvec[Zero]  = 0; }
   
   Serial.print("Set up done");
}

// === Main loop ======================================================================================================
// ====================================================================================================================
void loop(){
  ReadIR();
  PrintIRvectors();
  delay(1000);
}


// ==== Subrutiens ====================================================================================================
// ====================================================================================================================

// --- Read IR sensors ---------------------------------------------------------------------------------------
void ReadIR(){
  
    LeftIRtotal   = LeftIRtotal   - LeftIRvec[index];      // Subtract the last reading from the array
    CenterIRtotal = CenterIRtotal - CenterIRvec[index];
    RightIRtotal  = RightIRtotal  - RightIRvec[index];
    
   LeftIRvec[index] = analogRead(LeftIR);      // Read from the sensors
   CenterIRvec[index] = analogRead(CenterIR);
   RightIRvec[index] = analogRead(RightIR);
   
   LeftIRtotal   = LeftIRtotal   + LeftIRvec[index];      // Subtract the last reading from the array
   CenterIRtotal = CenterIRtotal + CenterIRvec[index];
   RightIRtotal  = RightIRtotal  + RightIRvec[index];
    
   index = index + 1;                       // Advance to the next position in the array
   
   if (index >= IR_vec_number) index = 0;  // Wrap around to the begining o fthe aray once your at the end
   
   LeftIRave   = LeftIRtotal   / IR_vec_number;        // Calculate average sensor values
   CenterIRave = CenterIRtotal / IR_vec_number;
   RightIRave  = RightIRtotal  / IR_vec_number;
   
   /*
   Serial.print("Left IR ave: ");         // Display averages
   Serial.print(LeftIRave);
   Serial.print("\t Center IR ave: ");
   Serial.print(CenterIRave);
   Serial.print("\t Right IR ave: ");
   Serial.println(RightIRave);
   */
}

// --- Display IR Vectors ----------------------------------------------------------------------------------------
void PrintIRvectors(){
 
  Serial.print("Left IR Vector: [ ");
  for (int i=0;i<(IR_vec_number-1);i++)
  { Serial.print(LeftIRvec[i]);
    Serial.print(", ");}
    Serial.print(LeftIRvec[(IR_vec_number-1)]);
    Serial.print(" ]");
    Serial.print("\t Left IR ave: ");         // Display averages
    Serial.println(LeftIRave);
    
    Serial.print("Center IR Vector: [ ");
  for (int i=0;i<(IR_vec_number-1);i++)
  { Serial.print(CenterIRvec[i]);
    Serial.print(", ");}
    Serial.print(CenterIRvec[(IR_vec_number-1)]);
    Serial.print(" ]");
    Serial.print("\t Center IR ave: ");
    Serial.println(CenterIRave);
    
    Serial.print("Right IR Vector: [ ");
  for (int i=0;i<(IR_vec_number-1);i++)
  { Serial.print(RightIRvec[i]);
    Serial.print(", ");}
    Serial.print(RightIRvec[(IR_vec_number-1)]);
    Serial.print(" ]");
    Serial.print("\t Right IR ave: ");
    Serial.println(RightIRave);
    
    Serial.println("");
}
/*    Average IR Values

Out Side:

Flat ground  : Concteet in shade  Concreet in sun    Dirt in shade   Grass in shade    No thing              Sun
-------------:-----------------------------------------------------------------------------------------
Left         :  322                  302                  293            273                2        Varys significantly
Center       :  326                  319                  312            322                5        Varys significantly
Right        :  300                  303                  300            278                5        Varys significantly


*/

