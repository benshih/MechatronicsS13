
// DC Motor Constants
#define drop_pos_L 2            // Dropper position detector left side INT0
#define drop_pos_M A0           // Dropper position detector left side
#define drop_pos_R 3            // Dropper position detector left side INT1
#define D1_DIR_ONE 4            // Chain Motor Board L2
#define D1_DIR_TWO 5            // Chain Motor Board L1
#define D1_ENABLE 6             // Chain Motor Board Enable
#define OFF 0                   // Defines Chain Motor Off
#define LEFT  2                 // Dropper pos Left
#define MID   1                 // Dropper pos Mid
#define RIGHT 3                 // Dropper pos Right
#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
int DPpos = MID;                 // Initial dropperchain pos is N/A 

// the setup routine runs once when you press reset:
void setup()
{
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  // initialize Dropchain
  DPCHAIN_INIT();
  // initialize interrupt for Dropchain lozolization position left and right
  pinMode(drop_pos_L,INPUT);
  pinMode(drop_pos_R,INPUT);  
  attachInterrupt(0, posL , RISING);
  attachInterrupt(1, posR , RISING);  
} 

void loop() 
{  
  DP_pos(LEFT);
  DP_drop();
  delay (5000);
  DP_pos(RIGHT);
  DP_drop();
  delay (5000);
  DP_pos(LEFT);
  DP_drop();
  delay (5000);
  DP_pos(MID);
  DP_drop();
  delay (5000);
  DP_pos(RIGHT);
  DP_drop();
  delay (5000);
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
void DP_drop()
{
  myservo.write(90);
  delay(1000);
  myservo.write(0);
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
    int sensorValueM = analogRead(A0);
//    Serial.print("\t sensor M = " );                       
//    Serial.println(sensorValueM);
    if(sensorValueM < 20)
    DPpos = MID;
    if (pos==DPpos)//the actual pose DPpos reaches the goal pose pos
    {
      DPCHAIN_BRAKE();//stop the chain
      dir=OFF;//stop chain
      myservo.write(0);//
      delay(1000);
      myservo.write(90);
      delay(1000);
      myservo.write(0);
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
      DPCHAIN_MOVE(100,dir);  
    } 
  Serial.print(" DPpos = " );                       
  Serial.print(DPpos);
  Serial.print("\t pos = " );                       
  Serial.println(pos);
  Serial.print("\t dir = " );                       
  Serial.println(dir);
  }  
}


