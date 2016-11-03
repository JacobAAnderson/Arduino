const int VectorNumber = 12;
int Vector[VectorNumber] = {10,13,52,63,4,85,86,887,8,6559,10,11};

int V1 = Vector[0];
void setup (){Serial.begin(9600);}
void loop(){
  PrintVector();
}

void PrintVector(){
  int i;
  
  Serial.print("Vector: [ ");
  for (i=0;i<(VectorNumber-1);i++)
  { Serial.print(Vector[i]);
    Serial.print(", ");}
    Serial.print(Vector[(VectorNumber-1)]);
    Serial.println(" ]");
  
  
  /*
  Serial.println( );
  Serial.print("Vector: ");
  Serial.print(Vector[0]);
  Serial.print(", ");
  Serial.print(Vector[1]);
  Serial.print(", ");
  Serial.print(Vector[2]);
  Serial.print(", ");
  Serial.print(Vector[3]);
  Serial.print(", ");
  Serial.print(Vector[4]);
  Serial.print(", ");
  Serial.print(Vector[5]);
  Serial.print(", ");
  Serial.print(Vector[6]);
  Serial.print(", ");
  Serial.print(Vector[7]);
  Serial.print(", ");
  Serial.print(Vector[8]);
  Serial.print(", ");
  Serial.print(Vector[9]);
  Serial.print(", ");
  Serial.print(Vector[10]);
  Serial.print(", ");
  Serial.println(Vector[11]);
  Serial.println();
  */
}

