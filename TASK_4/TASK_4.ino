/**
 * @file TASK_4.ino
 * @brief contains all sensor code
 *
 * @author Ram Muthiah (rmuthiah)
 * @author Ben Shih (bshih1)
 * @author Mark Erazo (merazo)
 * @author Hao Wang (haow1)
 */

// Servo Control library
#include <Servo.h> 

// Defines DC Motor Off
#define OFF 0

// Definition for all Motor types
#define SRV 0
#define DCM 1
#define STP 2           

// Serial Temp
int readin;

// Stepper Constants
#define motorStep 7          // A4988 Stepper Motor Driver Carrier step pin
#define motorDir 8           // A4988 Stepper Motor Driver Carrier direction pin
#define STEP_PER_DEG 5.75/10

int numSteps;                 // holder for number of microsteps required
int numDegrees = 0;           // number of degrees to be moved by stepper specified by user
int dir = 0;                  // Stepper Motor Driver Direction Reference
int curDegrees = 0;

// DC Motor Constants
#define encoderPin1 2        // Encoder Pin A
#define encoderPin2 3        // Encoder Pin B
#define dir_1 4              // Motor Board L2
#define dir_2 5              // Motor Board L1
#define enable 6             // Motor Board Enable
#define KP 2                 // Proportionality Constant for Control
#define KI 0.04              // Integration Constant for Control 

volatile long encoderValue = 0;  // Current Encoder Value from DCM
volatile long desired_loc = 0;   // User Defined Desired location of DCM
volatile float error;            // Difference between current location and user
                                 // desired location of motor used for PID control

// Servo Constants
#define IR_PIN A1             // input from IR sensor
#define SRV_SCALE 1.8         // Scaling Factor for IR values
#define SERVODELAY 200        // ms delay for servo
Servo myservo;                // create servo object to control a servo 
                              // a maximum of eight servo objects can be created 
int pos = 0;                  // variable to store the servo position 

/**
 * @brief initializes Serial Communication, DCM, Stepper, and Servo constants
 *
 * @param void
 * @return void
 */
void setup()
{
  // Serial init
  Serial.begin(9600);           // Serial initialization
  Serial.flush();               // Flush of serial buffer
  
  // Stepper init
  pinMode(motorStep, OUTPUT);   // Set up motor step pin as output on the A4988 Stepper Motor Driver Carrier.
  pinMode(motorDir, OUTPUT);    // Set up motor direction pin as output on the A4988 Stepper Motor Driver Carrier.
  digitalWrite(motorDir, LOW);  // Initialize Stepper Directions
  digitalWrite(motorStep, HIGH);// Turn off stepper (active low)
  
  // Servo Init
  myservo.attach(9);            // attaches the servo on pin 9 to the servo object
  
  //DC Motor init
  pinMode(encoderPin1, INPUT);  // init Encoders A and B 
  pinMode(encoderPin2, INPUT);

  digitalWrite(encoderPin1, HIGH); // turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); // turn pullup resistor on

  attachInterrupt(0, updateEncoder, CHANGE);  // triggers update encoder on every Encoder A change
  
  pinMode(dir_1, OUTPUT); // init L2
  pinMode(dir_2, OUTPUT); // init L1
  pinMode(enable, OUTPUT);// init enable
  
  analogWrite(enable,0);  // turn DC motor off
  digitalWrite(dir_1, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(dir_2, LOW);
}

/**
 * @brief runs Serial Communication, DCM, Stepper, and Servo constants with switch debounce
 *
 * @param void
 * @return void
 */
void loop() 
{
  numDegrees = (analogRead(A5) * 3 / 4) - curDegrees;
  pos = analogRead(IR_PIN) * SRV_SCALE;
  desired_loc = (((analogRead(A0) - 639) * -1 / 2) + 192) * 9 / 10;
  
  Serial.println(numDegrees);
  
  // if serial is available, assume it is a number
  if(Serial.available())
  {
    // parse the number and place under correct value
    readin = Serial.parseInt();
    // if readin is leet this resets all the values to a clear slate
    // and turns off all the motors and prints directions
    if(readin == 1337)
    {
      digitalWrite(motorDir, LOW);
      digitalWrite(motorStep, HIGH);
      analogWrite(enable,OFF);
      pos = 0;
      desired_loc = 0;
      numDegrees = 0;
      Serial.println("Servo is controlled by IR Sensor and moves between 0 and 180 deg");
      Serial.println("based on proximity of interfering object (0 if close, 180 otherwise)");
      Serial.println();
      Serial.println("DC Motor is controlled by pressure sensor");
      Serial.println("Moves between 0 and 360 deg based on the amount of pressure applied");
      Serial.println();
      Serial.println("Stepper is controlled by potentiometer");
      Serial.println("Based on turning moves between -9 deg and 9 deg per iteration");
    }
  }
  
  // Servo Control
  myservo.write(pos); // Set to variable servo.
  delay(SERVODELAY);
    
  // DC motor control
  error = desired_loc - encoderValue;
  // threshold so that if error is within 5 degrees of desired location
  // halt DC motor until desired location changes
  if(abs(error) < 10)
  {
    error = 0;
  }
  error = (KP * error); //PI control equation
  // set direction of DC motor based on sign of PI output
  if(error > 0)
  {
    digitalWrite(dir_1, LOW);
    digitalWrite(dir_2, HIGH);
  }
  else if(error < 0)
  {
    digitalWrite(dir_1, HIGH);
    digitalWrite(dir_2, LOW);
    error = -error;
  }
  
  // if dead on do nothing
  if(error == 0){}
  // high threshold for PI output of 255
  else if(error > 255)
  {
    error = 255;
  }
  // low threshold for PI output of 70
  else if(error < 70)
  {
    error = 70;
  }
  
  // write PI output to PWM to enable pin
  analogWrite(enable,error);
  
  // Stepper Control
  // calculate desired number of steps on stepper
  numSteps = numDegrees * STEP_PER_DEG;
  // reverse direction of stepper if direction is negative
  if(numSteps < 0)
  {
    numSteps = -numSteps;
    dir = 1;
  }
  else
  {
    dir = 0;
  }
  
  // set direction of Stepper
  digitalWrite(motorDir, dir);

  // write numSteps pulses to stepper to enable numDegrees motion
  for(int i = 0; i < numSteps; i++)
  {
    // Move the motor a single step with a duty cycle of 25%.
    digitalWrite(motorStep, LOW);
    delay(2);
    
    digitalWrite(motorStep, HIGH);
    delay(2);
  }
  
  curDegrees = numDegrees + curDegrees;
}

/**
 * @brief controls encoder count by detecting orientation of motor based on
 *        A and B pin orientations
 *
 * @param void
 * @return void
 */
void updateEncoder()
{
  int encA = digitalRead(encoderPin1); //MSB = most significant bit
  int encB = digitalRead(encoderPin2); //LSB = least significant bit

  // CW direction
  if(encA == encB)
  {
    encoderValue--;
  }
  // CCW direction
  else
  {
    encoderValue++;
  } 
}
