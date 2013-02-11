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

// Switch defining motor selection
#define SWT 11               

// Stepper Constants
#define motorStep 7          // A4988 Stepper Motor Driver Carrier step pin
#define motorDir 8           // A4988 Stepper Motor Driver Carrier direction pin
#define STEP_PER_DEG 5.75/10
int numSteps;                 // holder for number of microsteps required
int numDegrees = 0;           // number of degrees to be moved by stepper specified by user
int dir = 0;                  // Stepper Motor Driver Direction Reference

// DC Motor Constants
#define encoderPin1 2        // Encoder Pin A
#define encoderPin2 3        // Encoder Pin B
#define dir_1 4              // Motor Board L2
#define dir_2 5              // Motor Board L1
#define enable 6             // Motor Board Enable
#define KP 1                 // Proportionality Constant for Control
#define KI 0.04              // Integration Constant for Control 

volatile long encoderValue = 0;  // Current Encoder Value from DCM
volatile long desired_loc = 0;   // User Defined Desired location of DCM
volatile float error;            // Difference between current location and user
                                 // desired location of motor used for PID control
volatile float acc_err = 0;      // Integration of Error over time used for PID control

int readin;                   // Value holder for current input
int cur_motor = SRV;          // Current Motor Reference

int buttonState;              // the current reading from the input pin
int lastButtonState = LOW;    // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;    // the last time the output pin was toggled
long debounceDelay = 50;      // the debounce time; increase if the output flickers

// Servo Constants
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
  // switch init
  pinMode(SWT, INPUT);          // Switch for current motor selection
  
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
  // read the state of the switch into a local variable:
  int reading = digitalRead(SWT);
  desired_loc = (desired_loc + analogRead(A5))/2;
  // check to see if you just pressed the button 
  // (i.e. the input went from LOW to HIGH),  and you've waited 
  // long enough since the last press to ignore any noise:  

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) 
  {
    // reset the debouncing timer
    lastDebounceTime = millis();
  } 
  
  if ((millis() - lastDebounceTime) > debounceDelay) 
  {
    // whatever the reading is at, it's been there for longer
    // than the debounce delay, so take it as the actual current state:
    if(buttonState != reading)
    {
      // if reading is different from current button state
      // switch has been pressed and next motor is set
      cur_motor = (cur_motor + 1) % 3;
      
      // set current button state
      buttonState = reading;
      
      // To provide user directions for specific motor
      Serial.print("Current motor is ");
      if(cur_motor == SRV)
        Serial.println("Servo. Input limited to 0 to 180");
      else if (cur_motor == DCM)
        Serial.println("DC. Input limited to -1000 to 1000");
      else
        Serial.println("Stepper. Input limited to -1000 to 1000");
    }
  }
  
  // if serial is available, assume it is a number
  if(Serial.available())
  {
    // parse the number and place under correct value
    readin = Serial.parseInt();
    // if readin is leet this resets all the values to a clear slate
    // and turns off all the motors
    if(readin == 1337)
    {
      digitalWrite(motorDir, LOW);
      digitalWrite(motorStep, HIGH);
      analogWrite(enable,OFF);
      pos = 0;
      desired_loc = 0;
      numDegrees = 0;
    }
    
    // otherwise takes current input value and assigns it to variable holding
    // user specified input for the currently selected motor
    else if(cur_motor == SRV)
    {
      pos = readin;
    }
    else if (cur_motor == DCM)
    {
      desired_loc = readin;
    }    
    else
    {
      numDegrees = readin;
    }
  }
  
  // if current motor is servo
  if(cur_motor == SRV)
  {
    myservo.write(0); // Set to one extreme.
    delay(800);
    myservo.write(180); // Set to other extreme.
    delay(800);
    myservo.write(pos); // Set to variable servo.
    delay(800);
  }
  // if current motor is a DC motor
  else if(cur_motor == DCM)
  {
    error = desired_loc - encoderValue;
    Serial.print(desired_loc);
    Serial.print(" ");
    Serial.print(error);
    // threshold so that if error is within 5 degrees of desired location
    // halt DC motor until desired location changes
    if(abs(error) < 5)
    {
      acc_err = 0;
      error = 0;
    }
    acc_err = acc_err + error; //integrator for error term for PI Control
    error = (KP * error) + (KI * acc_err); //PI control equation
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
    Serial.print(" ");
    Serial.println(error);
    
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
  }
  // or it is a Stepper
  else
  {
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
    
    // reset numDegrees so stepper does not continue motion
    numDegrees = 0;
  }
  
  // set last button read for debounce
  lastButtonState = reading;
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
