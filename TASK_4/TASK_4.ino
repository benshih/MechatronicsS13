//Ram Muthiah
//Hao Wang
//Ben Shih
//Mark Erazo

#include <Servo.h> 

#define OFF 0

//Definition for all Motor types
#define SRV 0
#define DCM 1
#define STP 2

#define SWT 11               //switch defining motor selection

#define motorStep 7          // A4988 Stepper Motor Driver Carrier step pin
#define motorDir 8           // A4988 Stepper Motor Driver Carrier direction pin
#define DEGREES_PER_STEP 1.8

#define encoderPin1 2
#define encoderPin2 3
#define dir_1 4
#define dir_2 5
#define enable 6

volatile long encoderValue = 0;
volatile long cur_dir = 0;
volatile long desired_loc = 0;
volatile float error;
volatile float motor_in;
volatile float acc_err = 0;
float KP = 1;
float KI = 0.04;

int readin;                   // Value holder for current input
int cur_motor = SRV;          // Current Motor

int buttonState;              // the current reading from the input pin
int lastButtonState = LOW;    // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;    // the last time the output pin was toggled
long debounceDelay = 50;      // the debounce time; increase if the output flickers

int numSteps;                 // holder for number of microsteps required
int numDegrees = 0;         // number of degrees to be moved by stepper
int dir = 0;                

Servo myservo;                // create servo object to control a servo 
                              // a maximum of eight servo objects can be created 
int pos = 0;                 // variable to store the servo position 

void setup()
{
  pinMode(SWT, INPUT);        // Switch for motor control
  pinMode(motorStep, OUTPUT); // Set up motor step pin as output on the A4988 Stepper Motor Driver Carrier.
  pinMode(motorDir, OUTPUT);  // Set up motor direction pin as output on the A4988 Stepper Motor Driver Carrier.
  Serial.begin(9600);         // Serial initialization
  Serial.flush();             // Flush of serial buffer
  
  myservo.attach(9);          // attaches the servo on pin 9 to the servo object
  
  digitalWrite(motorDir, LOW);
  digitalWrite(motorStep, HIGH);
  
  pinMode(encoderPin1, INPUT); 
  pinMode(encoderPin2, INPUT);

  digitalWrite(encoderPin1, HIGH); //turn pullup resistor on
  digitalWrite(encoderPin2, HIGH); //turn pullup resistor on

  attachInterrupt(0, updateEncoder, CHANGE); 
  
  pinMode(dir_1, OUTPUT); 
  pinMode(dir_2, OUTPUT);
  pinMode(enable, OUTPUT);
  analogWrite(enable,0);
  digitalWrite(dir_1, LOW);
  digitalWrite(dir_2, LOW);
}

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
    if(readin == 1337)
    {
      digitalWrite(motorDir, LOW);
      digitalWrite(motorStep, HIGH);
      analogWrite(enable,OFF);
      pos = 0;
      desired_loc = 0;
      numDegrees = 0;
    }
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
    if(abs(error) < 5)
    {
      acc_err = 0;
      error = 0;
    }
    acc_err = acc_err + error;
    error = (KP * error) + (KI * acc_err);
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
    if(error == 0){}
    else if(error > 255)
    {
      error = 255;
    }
    else if(error < 70)
    {
      error = 70;
    }
    
    analogWrite(enable,error);
  }
  // or it is a Stepper
  else
  {
    numSteps = numDegrees * 5.75 / 10;
    if(numSteps < 0)
    {
      numSteps = -numSteps;
      dir = 1;
    }
    else
    {
      dir = 0;
    }
    
    digitalWrite(motorDir, dir);
  
    for(int i = 0; i < numSteps; i++)
    {
      // Move the motor a single step with a duty cycle of 25%.
      digitalWrite(motorStep, LOW);
      delay(2);
      
      digitalWrite(motorStep, HIGH);
      delay(2);
    }
    
    numDegrees = 0;
  }
  
  // set last button read for debounce
  lastButtonState = reading;
}

void updateEncoder()
{
  int MSB = digitalRead(encoderPin1); //MSB = most significant bit
  int LSB = digitalRead(encoderPin2); //LSB = least significant bit

  if(MSB == LSB)
  {
    encoderValue--;
  }
  else
  {
    encoderValue++;
  } 
}
