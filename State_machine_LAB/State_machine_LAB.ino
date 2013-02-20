/**
 * @file TASK_5.ino
 * @brief contains all state machine code
 *
 * @author Mark Erazo (merazo)
 * @author Ram Muthiah (rmuthiah)
 * @author Ben Shih (bshih1)
 * @author Hao Wang (haow1)
 */

// Servo Constants and Library
#include <Servo.h> 
#define SRV_ONE 9            // Servos for Dropper
#define SRV_TWO 10
#define SRV_DEL 400          // Delay for Servos to Stabilize
Servo servo_one;             // create servo objects to control a servos
Servo servo_two;

// DC Motor Constants
#define DIR_ONE 4            // Motor Board L2
#define DIR_TWO 5            // Motor Board L1
#define ENABLE 6             // Motor Board Enable
#define OFF 0                // Defines DC Motor Off
#define ON 200               // Defines DC Motor On

// Possible States on FSM
#define START 1
#define FOLLOW_ROW 2
#define CHG_DIR 3
#define DROP 4
#define NEW_ROW 5

// Switch Debounce
#define SWT_PIN       11
#define DEBOUNCE_TIME 50

// Sensor Constants
#define FRC_PIN A0             // input from Force sensor
#define IR_PIN  A1             // input from IR sensor
#define FRC_UNP 700            // Force Sensor Unpressed Value
#define IR_MIN 40              // IR Sensor Cap Value for Position Reached

// Current State of FSM
int cur_state;

// Shingle State Variables and Constants
#define ODD 3                  // Odd Row Shingle Count
#define EVEN 4                 // Even Row Shingle Count
int cur_row;                   // Odd or Even Row
int shingle_count;             // Shingle Count on Current Row

/**
 * @brief initializes FSM constant, Serial Communication, DCM, and Servo constants
 *
 * @param void
 * @return void
 */
void setup() 
{
  SWITCH_INIT();
  SERIAL_INIT();
  SERVO_INIT();
  DCM_INIT();
  
  cur_state = START;
  cur_row = ODD;
  shingle_count = 0;
  Serial.println("The current state is START");
}

/**
 * @brief runs Serial Communication, DCM, and Servo constants with switch debounce
 *
 * @param void
 * @return void
 */
void loop() 
{
  // the FSM in its entirety
  switch (cur_state) 
  {
    case START:
      SWITCH_DEBOUNCE();
      break;
      
    case FOLLOW_ROW:
      DCM_MOVE_FORWARD();
      // if the force sensor does "detect" roof edge, back up
      if(analogRead(FRC_PIN) < FRC_UNP)
      {
        DCM_MOVE_REVERSE();
        cur_state = CHG_DIR;
        Serial.println("The current state is CHG_DIR");
        break;
      }
      
      // if IR Sensor detects "line" transition to drop state
      if(analogRead(IR_PIN) < IR_MIN)
      {
        cur_state = DROP;
        Serial.println("The current state is DROP");
      }
      break;
      
    case CHG_DIR:
      // Back up until Roof Edge is clear
      // if the force sensor does "detect" roof edge, change back to moving
      if(analogRead(FRC_PIN) >= FRC_UNP)
      {
        cur_state = FOLLOW_ROW;
        Serial.println("The current state is FOLLOW_ROW");
      }
      break;
      
    case DROP:
      DCM_BRAKE();
      // if IR Reading has steadied itself, drop shingle
      // otherwise stay in DROP state until this occurs
      if(analogRead(IR_PIN) >= IR_MIN)
      {
        SHINGLE_DROP();
        CHECK_SHINGLE_COUNT();
      }
      break;
      
    case NEW_ROW:
      // Back up to beginning of next row
      DCM_MOVE_REVERSE();
      // Takes approx. 2 seconds (note this measure is complete BS)
      delay(3000);
      cur_state = FOLLOW_ROW;
      Serial.println("The current state is FOLLOW_ROW");
      break;
      
    default: 
      break;
  }
}

/**
 * @brief Switch Init
 *
 * @param void
 * @return void
 */
void SWITCH_INIT()
{
  pinMode(SWT_PIN, INPUT);          // Switch for current motor selection
}

/**
 * @brief Serial Init
 *
 * @param void
 * @return void
 */
void SERIAL_INIT()
{
  Serial.begin(9600);           // Serial initialization
  Serial.flush();               // Flush of serial buffer
}

/**
 * @brief Servo Init
 *
 * @param void
 * @return void
 */
void SERVO_INIT()
{
  servo_one.attach(SRV_ONE);      // attaches the servo on pin 9 and 10
  servo_two.attach(SRV_TWO);      // to the servo objects
}

/**
 * @brief DC Motor Init
 *
 * @param void
 * @return void
 */
void DCM_INIT()
{
  pinMode(DIR_ONE, OUTPUT);   // init L2
  pinMode(DIR_TWO, OUTPUT);   // init L1
  pinMode(ENABLE, OUTPUT);    // init enable
  
  analogWrite(ENABLE,0);      // turn DC motor off
  digitalWrite(DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(DIR_TWO, LOW);
}

/**
 * @brief Enacapsulates Switch Debounce Code for Start Condition
 *
 * @param void
 * @return void
 */
void SWITCH_DEBOUNCE()
{
  if(digitalRead(SWT_PIN) == HIGH)
  {
    delay(DEBOUNCE_TIME);
    if(digitalRead(SWT_PIN) == HIGH)
    {
      cur_state = FOLLOW_ROW;
      Serial.println("The current state is FOLLOW_ROW");
    }
  }
}

/**
 * @brief Makes DC Motor move "Forward" at half speed
 *
 * @param void
 * @return void
 */
void DCM_MOVE_FORWARD()
{
  digitalWrite(DIR_ONE, LOW);
  digitalWrite(DIR_TWO, HIGH);
  analogWrite(ENABLE, ON);
}

/**
 * @brief Makes DC Motor move "Reverse" at half speed
 *
 * @param void
 * @return void
 */
void DCM_MOVE_REVERSE()
{
  digitalWrite(DIR_ONE, HIGH);
  digitalWrite(DIR_TWO, LOW);
  analogWrite(ENABLE, ON);
}

/**
 * @brief Stops DC Motor by braking
 *
 * @param void
 * @return void
 */
void DCM_BRAKE()
{
  analogWrite(ENABLE,0);      // turn DC motor off
  digitalWrite(DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(DIR_TWO, LOW);
}

/**
 * @brief Shingle Drop encapsulation
 *
 * @param void
 * @return void
 */
void SHINGLE_DROP()
{
  shingle_count++;
  servo_one.write(90);
  servo_two.write(90);
  delay(SRV_DEL);
  servo_one.write(0);
  servo_two.write(0);
  delay(SRV_DEL);
}

/**
 * @brief Evaluate Row Conditions
 *
 * @param void
 * @return void
 */
void CHECK_SHINGLE_COUNT()
{
  // Return if shingles are not completed for current row
  if(shingle_count != cur_row)
  {
    cur_state = FOLLOW_ROW;
    Serial.println("The current state is FOLLOW_ROW");
    return;
  }
  
  // Reset shingle counts otherwise
  shingle_count = 0;
  // Switch Next Row
  if(cur_row == ODD)
  {
    cur_row = EVEN;
  }
  else
  {
    cur_row = ODD;
  }
  
  // Start next row
  cur_state = NEW_ROW;
  Serial.println("The current state is NEW_ROW");
}
