

void setup() { Serial.begin(9600);

union {
  float number;
  uint8_t bytes[4];
  } m;

m.number = 231.456; // Assign a number to the float

for (int i=0; i<4; i++){ Serial.print(m.bytes[i], DEC); // Print the hex representation of the float
                         Serial.print(' ');
                        }

Serial.println("\n");
  
 union {
    uint8_t input[4];
    float Value; 
    } u;
  
    u.input[0] = m.bytes[0];
    u.input[1] = m.bytes[1];
    u.input[2] = m.bytes[2];
    u.input[3] = m.bytes[3];

// float val = u.Value;

    Serial.print(u.Value);
    


}

void loop() {
  // put your main code here, to run repeatedly:

}
