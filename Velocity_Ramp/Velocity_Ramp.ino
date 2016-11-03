
# define RampStep 3000

float time = 0.0;
int velocity = 0;

void setup() { Serial.begin(9600);

Serial.println(" Starting Velocity Ramper");
 

}

void loop() {

  static int target;
  static float t_last;

  
  time = millis() / 1000.0;
  if (( t_last - time ) > 4,000,000) t_last = 0;

  if( Serial.available()) target = Serial.parseInt();

  velocity = Vel_Ramp( velocity, target, time, t_last );

   
  t_last = time;

  Serial.print("Time: "), Serial.print(time);
  Serial.print("\tTarget: "), Serial.print(target);
  Serial.print("\tVelocity: "), Serial.println(velocity);

  delay(10);
  
}

int Vel_Ramp( int velocity, int target, float t_now,  float t_last ) {

  int sign = 0;
  
  float Step = RampStep * ( t_now - t_last );
  int error = velocity - target;

  Serial.println();
  Serial.print("Step: "), Serial.print(Step);
  Serial.print("\terror: "), Serial.println(error);
  

  if (Step > abs(error)) return target;
  
  if (velocity < target) sign = 1;
  else sign = -1; 

  return velocity + sign * Step;
  
}

