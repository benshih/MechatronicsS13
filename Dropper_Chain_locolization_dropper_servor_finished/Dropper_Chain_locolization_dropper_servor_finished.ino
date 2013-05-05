
// DC Motor Constants
#define drop_pos_L 2            // Dropper position detector left side INT0
#define drop_pos_M A2           // Dropper position detector left side A2
#define drop_pos_R 3            // Dropper position detector left side INT1
#define D1_DIR_ONE 10            // Chain Motor Board L2
#define D1_DIR_TWO 11            // Chain Motor Board L1
#define D1_ENABLE 12             // Chain Motor Board Enable
#define OFF 0                   // Defines Chain Motor Off
#define LEFT  2                 // Dropper pos Left
#define MID   1                 // Dropper pos Mid
#define RIGHT 3                 // Dropper pos Right
#include <Servo.h> 
 
Servo servoL;  // create servo object to control servo left
Servo servoR;  // create servo object to control servo Right
int DPpos = LEFT;                 // Initial dropperchain pos is N/A 

// the setup routine runs once when you press reset:
void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  // initialize Dropchain
  DPCHAIN_INIT();
  // initialize interrupt for Dropchain lozolization position left and right
  pinMode(drop_pos_L,INPUT);
  pinMode(drop_pos_R,INPUT);  
  attachInterrupt(0, posL , RISING);
  attachInterrupt(1, posR , RISING);
  servoR.attach(14);  // attaches the servo on pin 14 to the servoR
  servoL.attach(15);  // attaches the servo on pin 15 to the servoL
  servoL.write(70);
  servoR.write(180); 
} 

void loop() 
{  
  DP_pos(MID);
  delay (10000);
  DP_pos(LEFT);
  DP_drop();
  delay (5000);
  DP_pos(MID);
  delay (10000);
  DP_pos(RIGHT);
  DP_drop();
  delay (5000);
  while(1);
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
//interupt:Right position arrived renew Dropper position = RIGHT  
void posR ()
{
  DPpos= RIGHT;
//  Serial.print("DPpos = RIGHT" );                       
//  Serial.println(DPpos);
}
//interupt:Left position arrived renew Dropper position = LEFT 
void posL ()
{
  DPpos = LEFT;
//  Serial.print("DPpos = LEFT" );                       
//  Serial.println(DPpos);
}

/**
 * @brief Dropper actuator servo
 *
 * @param void
 * @return void
 */
/**
 * @brief Dropper actuator servo
 *
 * @param void
 * @return void
 */
void DP_drop()
{
  servoL.write(180);
  servoR.write(60);
  delay(800);
  servoL.write(70);
  servoR.write(170);
}

/**
 * @brief Dropperchain locolization
 *
 * @param int pos
 * @return void
 */
void DP_pos(int pos)
{
  int dir = 4;
  while (dir != OFF) 
  {
    // read the input on analog pin 0:
    int sensorValueM = analogRead(drop_pos_M);
//    Serial.print("\t sensor M = " );                       
//    Serial.println(sensorValueM);
    if(sensorValueM < 60)
    DPpos = MID;
    if (pos==DPpos)//the actual pose DPpos reaches the goal pose pos
    {
      DPCHAIN_BRAKE();//stop the chain
      dir=OFF;//stop chain
    }
    else
    {
      switch (pos)
      {
      case LEFT:
        dir= LEFT;
        break;

      case MID:
        {
          if (DPpos == LEFT)
            dir= RIGHT;
          if (DPpos == RIGHT)
            dir= LEFT;
        }
        break;

      case RIGHT:
        dir= RIGHT;
        break;
      }
      DPCHAIN_MOVE(150,dir);  
    } 
  Serial.print(" DPpos = " );                       
  Serial.print(DPpos);
  Serial.print("\t pos = " );                       
  Serial.println(pos);
  Serial.print("\t dir = " );                       
  Serial.println(dir);
  }  
}


