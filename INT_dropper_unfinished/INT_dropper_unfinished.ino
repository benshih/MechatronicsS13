
// DC Motor Constants
#define drop_pos_L 2           // Dropper position detector left side INT1
#define drop_pos_M A1           // Dropper position detector left side
#define drop_pos_R 3           // Dropper position detector left side INT0
#define D1_DIR_ONE 4             // Chain Motor Board L2
#define D1_DIR_TWO 5            // Chain Motor Board L1
#define D1_ENABLE 6             // Chain Motor Board Enable
#define OFF 0                   // Defines Chain Motor Off
#define LEFT  2                 // Dropper pos Left
#define MID   1                 // Dropper pos Mid
#define RIGHT 3                 // Dropper pos Right
int DPpose=0;
int dir=0;

// the setup routine runs once when you press reset:
void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  DPCHAIN_INIT();
  attachInterrupt(0,posR , RISING);
  attachInterrupt(1,posL , RISING);  
} 
 
void loop() 
{  
 DP_pos(RIGHT);
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

void posR ()
{
  DPpose= RIGHT;
  Serial.print("DPpose = RIGHT" );                       
  Serial.println(DPpose);
}

void posL ()
{
  DPpose = LEFT;
  Serial.print("DPpose = RIGHT" );                       
  Serial.println(DPpose);
}

void DP_pos(int pos)
{
  switch (pos)
  {
  case LEFT:
    if (DPpose == pos)
    {
      DPCHAIN_BRAKE();
      return;
    }
    else dir= LEFT;
  break;
  
  case MID:
  if (DPpose == pos)
    {
      DPCHAIN_BRAKE();
      return;
    }
    else 
    {
      if (pos == LEFT)
      dir= RIGHT;
      if (pos == RIGHT)
      dir= LEFT;
    }
  break;
  
  case RIGHT:
    if (DPpose == pos)
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

