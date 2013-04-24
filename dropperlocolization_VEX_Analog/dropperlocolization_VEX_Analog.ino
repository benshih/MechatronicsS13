// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.
// DC Motor Constants
#define drop_pos_L A0           // Dropper position detector left side
#define drop_pos_M A1           // Dropper position detector left side
#define drop_pos_R A2           // Dropper position detector left side
#define D1_DIR_ONE 4            // Chain Motor Board L2
#define D1_DIR_TWO 5            // Chain Motor Board L1
#define D1_ENABLE 6             // Chain Motor Board Enable
#define OFF 0                   // Defines Chain Motor Off
#define LEFT  1                 // Dropper pos Left
#define MID   2                 // Dropper pos Mid
#define RIGHT 3                 // Dropper pos Right

// the setup routine runs once when you press reset:
void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  DPCHAIN_INIT();
} 
 
void loop() 
{  
  DP_pos(LEFT);
} 



/**
 * @brief DC Motor Init
 *
 * @param void
 * @return void
 */
void DPCHAIN_INIT()
{
  // Init for Chain Motor 
  pinMode(D1_DIR_ONE, OUTPUT);   // init L2
  pinMode(D1_DIR_TWO, OUTPUT);   // init L1
  pinMode(D1_ENABLE, OUTPUT);    // init enable
  
  analogWrite(D1_ENABLE,OFF);    // turn DC motor off
  digitalWrite(D1_DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(D1_DIR_TWO, LOW);
  
}

/**
 * @brief Stops DC Motors by braking
 *
 * @param void
 * @return void
 */
void DPCHAIN_BRAKE()
{
  analogWrite(D1_ENABLE,OFF);      // turn DC motor off
  digitalWrite(D1_DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(D1_DIR_TWO, LOW);
}

/**
 * @brief Makes DC Motors move dir at speed rpm
 *
 * @param desired_speed desired speed
 * @param dir moving left or right
 * @return void
 */
void DPCHAIN_MOVE(int desired_speed, int dir)
{
  if(dir == LEFT)
  {
    digitalWrite(D1_DIR_ONE, HIGH);
    digitalWrite(D1_DIR_TWO, LOW);

  }
  if (dir == RIGHT)
  {
    digitalWrite(D1_DIR_ONE, LOW);
    digitalWrite(D1_DIR_TWO, HIGH);
  }  
  analogWrite(D1_ENABLE, desired_speed);
}
/**
 * @brief move the dropper to Left, Right or Mid 
 *
 * @param 
 * @param t
 * @return void
 */
void DP_pos(int pos)
{
  int dir=0;
  // read the input on analog pin 0:
  int sensorValueL = analogRead(A0);
  // read the input on analog pin 1:
  int sensorValueM = analogRead(A1);
  // read the input on analog pin 2:
  int sensorValueR = analogRead(A2);
  Serial.print("sensor L = " );                       
  Serial.print(sensorValueL); 
  Serial.print("\t sensor M = " );                       
  Serial.print(sensorValueM);   
  Serial.print("\t senso R = ");      
  Serial.print(sensorValueR);
  switch (pos)
  {
  case LEFT:
    if (sensorValueL>100)
    {
      DPCHAIN_BRAKE();
      return;
    }
    else dir= LEFT;
  break;
  
  case MID:
  if (sensorValueM<10)
    {
      DPCHAIN_BRAKE();
      return;
    }
    else 
    {
      if (sensorValueL>100)
      dir= RIGHT;
      if (sensorValueR>100)
      dir= LEFT;
    }
  break;
  
  case RIGHT:
    if (sensorValueR > 100)
    {
      DPCHAIN_BRAKE();
      return;
    }
    else dir= RIGHT;
  break;
  }
  DPCHAIN_MOVE(75,dir);
  
  Serial.print("\t dir = " );                       
  Serial.println(dir);  
}
