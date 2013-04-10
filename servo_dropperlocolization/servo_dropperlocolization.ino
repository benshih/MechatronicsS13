// Sweep
// by BARRAGAN <http://barraganstudio.com> 
// This example code is in the public domain.
// DC Motor Constants
#define M1_DIR_ONE 4            // Motor Board L2
#define M1_DIR_TWO 5            // Motor Board L1
#define M1_ENABLE 6             // Motor Board Enable
#define M2_DIR_ONE 7            // Motor Constants for Motor 2
#define M2_DIR_TWO 8
#define M2_ENABLE 9
#define OFF 0                   // Defines DC Motor Off
#define RIGHT 1                 // Rotate Right
#define LEFT 2                  // Rotate Left
#define FORWARD 1               // Move Forward
#define BACK 2                  // Move Backward

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 

void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
  DCM_INIT();
} 
 
 
void loop() 
{ 
  myservo.write(0);
  delay(400);
  myservo.write(90);
  delay(400); 
  DCM_MOVE(255,FORWARD);
} 



/**
 * @brief DC Motor Init
 *
 * @param void
 * @return void
 */
void DCM_INIT()
{
  // Init for Motor 1
  pinMode(M1_DIR_ONE, OUTPUT);   // init L2
  pinMode(M1_DIR_TWO, OUTPUT);   // init L1
  pinMode(M1_ENABLE, OUTPUT);    // init enable
  
  analogWrite(M1_ENABLE,OFF);    // turn DC motor off
  digitalWrite(M1_DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(M1_DIR_TWO, LOW);
  
  // Init for Motor 2
  pinMode(M2_DIR_ONE, OUTPUT);   // init L2
  pinMode(M2_DIR_TWO, OUTPUT);   // init L1
  pinMode(M2_ENABLE, OUTPUT);    // init enable
  
  analogWrite(M2_ENABLE,OFF);    // turn DC motor off
  digitalWrite(M2_DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(M2_DIR_TWO, LOW);
}

/**
 * @brief Stops DC Motors by braking
 *
 * @param void
 * @return void
 */
void DCM_BRAKE()
{
  analogWrite(M1_ENABLE,OFF);      // turn DC motor off
  digitalWrite(M1_DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(M1_DIR_TWO, LOW);
 
}

/**
 * @brief Makes DC Motors move dir at speed rpm
 *
 * @param desired_speed desired speed of rotation
 * @param dir rotate right or left
 * @return void
 */
void DCM_MOVE(int desired_speed, int dir)
{
  if(dir == FORWARD)
  {
    digitalWrite(M1_DIR_ONE, HIGH);
    digitalWrite(M1_DIR_TWO, LOW);
    
    digitalWrite(M2_DIR_ONE, HIGH);
    digitalWrite(M2_DIR_TWO, LOW);
  }
  
  else
  {
    digitalWrite(M1_DIR_ONE, LOW);
    digitalWrite(M1_DIR_TWO, HIGH);
    
    digitalWrite(M2_DIR_ONE, LOW);
    digitalWrite(M2_DIR_TWO, HIGH);
  }
  
  analogWrite(M1_ENABLE, desired_speed);
  analogWrite(M2_ENABLE, desired_speed);
}
