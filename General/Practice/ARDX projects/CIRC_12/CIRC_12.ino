/*  This is a practice script
    This scrip controls a four lead RGB LED
*/

int LEDpins [] = {9, 10, 11}; // LED output pins: pin 9 - red, pin 10 - green, pin 11 - blue

const boolean ON  = LOW;    // The RGB LED shares a comon anode, thus a low pin will allow current flow.
const boolean OFF = HIGH;

// Predefined colors
const boolean RED[]     = { ON,  OFF, OFF };  
const boolean GREEN[]   = { OFF, ON,  OFF };
const boolean BLUE[]    = { OFF, ON,  OFF };
const boolean YELLOW[]  = { ON,  ON,  OFF };
const boolean CYAN[]    = { OFF, ON,  ON  };
const boolean MAGENTA[] = { ON,  OFF, ON  };
const boolean WHITE[]   = { ON,  ON,  ON  };
const boolean BLACK[]   = { OFF, OFF, OFF };

// Array to hold the predefined colors
const boolean* COLORS[] = { RED, GREEN, BLUE, YELLOW, CYAN, MAGENTA, WHITE, BLACK }; 


void setup() { Serial.begin(9600);

    for( int i=0; i<3; i++) { pinMode( LEDpins[i], OUTPUT ); } // Set up LED pins as outputs
 
Serial.println(" --------- Setup Done ---------------");
}


void loop() {
  
  setColor( LEDpins, CYAN); // Set the color of the LED
  
  randomColor();
  
}

/* -------------------------------------------------------------------------------------------
============== Subrutiens ====================================================================
--------------------------------------------------------------------------------------------*/

void setColor(int* led, boolean* color) {
    
    for (int i=0; i<3; i++) { digitalWrite (led[i], color[i]);}
   
}


void randomColor() { 
  
  int rand = random(0, sizeof(COLORS) /2); // get random number for a random color
  
  setColor( LEDpins, COLORS[rand]);  // Set Color
  
  delay(1000);





