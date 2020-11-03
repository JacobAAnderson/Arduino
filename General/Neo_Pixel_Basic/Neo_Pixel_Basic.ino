#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
  #include <avr/power.h>
#endif

#define PIN 6

Adafruit_NeoPixel strip = Adafruit_NeoPixel(60, PIN, NEO_GRB + NEO_KHZ800);

// Color order: red, green, blue
uint32_t magenta = strip.Color(255, 0, 255);
uint32_t red = strip.Color(255, 0, 0);

void setup() { Serial.begin(9600);

  strip.begin();
  strip.show(); // Initialize all pixels to 'off'

  Serial.println("Begin!!");
}

void loop() {
  
int j = 0;

Serial.println ("Random");
while ( j<100 ) {
                    for ( int i =0; i < strip.numPixels(); i++) {
                                                                   int red = random(0,255);
                                                                   int green = random(0,255);
                                                                   int blue = random(0,255);

                                                                   strip.setPixelColor( i, red, green, blue);
                                                                   strip.show();
                                                                   }

                    j++;
                  }

Serial.println("Magenta");
for ( int i =0; i < strip.numPixels(); i++) { strip.setPixelColor( i, magenta);
                                               strip.show();
                                              }

delay(2000);

Serial.println("Red");
for ( int i =0; i < strip.numPixels(); i++) { strip.setPixelColor( i, red);
                                               strip.show();
                                              }

delay(2000);

}

