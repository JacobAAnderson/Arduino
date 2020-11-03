
char Force[] = "Force Sensor";
char ph[] = "ph sensor";
int vec[] = {1, 5, 6, 9, 0};

void setup() { Serial.begin(9600);
 
 
}

void loop() {
  static int a;
  Serial.println(Force);
  Serial.println(ph);
  Serial.println(vec[a]);
  Serial.println(ph[a]);
  
  if (a > 4) { a = 0; }
  else { a++; }
  
  Serial.println(a);
  
  delay(1000);
  
}
