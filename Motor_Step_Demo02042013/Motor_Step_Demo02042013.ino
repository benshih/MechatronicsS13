//Ram Muthiah
//Hao Wang
//Ben Shih
//Mark Erazo

#include <Servo.h> 

//Definition for all Motor types
#define SRV 0
#define DCM 1
#define STP 2

#define SWT 3               //switch defining motor selection

#define motorStep 7          // A4988 Stepper Motor Driver Carrier step pin
#define motorDir 8           // A4988 Stepper Motor Driver Carrier direction pin
#define DEGREES_PER_STEP 1.8

int readin;                   // Value holder for current input
int cur_motor = SRV;          // Current Motor

int buttonState;              // the current reading from the input pin
int lastButtonState = LOW;    // the previous reading from the input pin

// the following variables are long's because the time, measured in miliseconds,
// will quickly become a bigger number than can be stored in an int.
long lastDebounceTime = 0;    // the last time the output pin was toggled
long debounceDelay = 50;      // the debounce time; increase if the output flickers

int numSteps;                 // holder for number of microsteps required
int numDegrees = 180;         // number of degrees to be moved by stepper
int dir = 0;                

Servo myservo;                // create servo object to control a servo 
                              // a maximum of eight servo objects can be created 
int pos = 90;                 // variable to store the servo position 

void setup()
{
  pinMode(SWT, INPUT);        // Switch for motor control
  pinMode(motorStep, OUTPUT); // Set up motor step pin as output on the A4988 Stepper Motor Driver Carrier.
  pinMode(motorDir, OUTPUT);  // Set up motor direction pin as output on the A4988 Stepper Motor Driver Carrier.
  Serial.begin(9600);         // Serial initialization
  Serial.flush();             // Flush of serial buffer
}

void loop() 
{
  // read the state of the switch into a local variable:
  int reading = digitalRead(SWT);

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
    if(cur_motor == SRV)
    {
      pos = readin;
    }
    else if (cur_motor == DCM)
    {
      
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
    delay(1000);
    myservo.write(180); // Set to other extreme.
    delay(1000);
    myservo.write(pos); // Set to variable servo.
    delay(1000);
  }
  // if current motor is a DC motor
  else if(cur_motor == DCM)
  {
    
  }
  // or it is a Stepper
  else
  {
    numSteps = numDegrees / DEGREES_PER_STEP;
    if(numSteps < 0)
    {
      dir = 1;
      numSteps = -numSteps;
    }
    else
    {
      dir = 0;
    }
    digitalWrite(motorDir, dir);
    
    // Move the motor a single step with a duty cycle of 50% for the number of steps as determined by the motor's physical properties of degrees-per-step.
    for(int i = 0; i < numSteps; i++)
    {
      digitalWrite(motorStep, LOW);
      delay(2);
      
      digitalWrite(motorStep, HIGH);
      delay(2);
    } 
  }
  
  // set last button read for debounce
  lastButtonState = reading;
}
