/*  This is a general perpose script for passing a vector into a subfunction
 *   and returning a vector
 */


int Array1[3];
int Array2[3];

void setup() {Serial.begin(9600);

Serial.print("Size of Array1: ");
Serial.println(sizeof(Array1)/sizeof(int));

for ( int i = 0; i< (sizeof(Array1)/sizeof(int)); i++) { Array1[i] = random(0,10);
                                                         Serial.println(Array1[i]);
                                                         }


Serial.print("\n\nSize of Array2: ");
Serial.println(sizeof(Array2)/sizeof(int));
                                                         
for ( int i = 0; i< (sizeof(Array2)/sizeof(int)); i++) { Array2[i] = random(0,10);
                                                          Serial.println(Array2[i]);
                                                        }

Serial.println("\n\n");

int* A = Add(Array1, Array2);
Serial.println("Sum of the Vectos is:");

Serial.print("Size of product array: ");
Serial.println(sizeof(A)/sizeof(int));

for ( int i = 0; i< (sizeof(A)/sizeof(int)); i++) Serial.println(A[i]);



}

void loop() { }

int* Add(int V1[], int V2[]) {

Serial.println("V1:");
for ( int i = 0; i< (sizeof(V1)/sizeof(int)); i++) Serial.println(V1[i]);

Serial.println("V2:");
for ( int i = 0; i< (sizeof(V2)/sizeof(int)); i++) Serial.println(V2[i]);  
  
 if ( sizeof(V1)/sizeof(int) != sizeof(V2)/sizeof(int) ) { Serial.println("**Array Sizes do not match**");
                                                           return 0;
                                                           }
Serial.println("\nSum:"); 
int Sum[sizeof(V1)/sizeof(int)];

for (int i=0; i < (sizeof(V1)/sizeof(int)); i++) { Sum[i] = V1[i]+V2[i];
                                                   Serial.println(Sum[i]);
                                                   }
Serial.println();
return Sum;
}

