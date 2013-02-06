// Stepper only. 

//http://www.tigoe.net/pcomp/code/circuits/motors/stepper-motors/
//http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Robotics/42BYGHM809.PDF

/*
 Stepper Motor Controller
 language: Wiring/Arduino

 This program drives a unipolar or bipolar stepper motor.
 The motor is attached to digital pins 8 and 9 of the Arduino.

 The motor moves 100 steps in one direction, then 100 in the other.
 
 To connect the phase coils in parallel, connect stepper leads A and C’ to board output 1A, stepper leads A’ and C to board output 1B, stepper leads B and D’ to board output 2A, and stepper leads B’ and D to board output 2B.
http://dlnmh9ip6v2uc.cloudfront.net/datasheets/Robotics/42BYGHM809.PDF



*/

// define the pins that the motor is attached to. You can use
// any digital I/O pins.

// User selectable (input from hyperterminal gui).
#define numDegrees 180 // In degrees.
#define degPerStep 0.9
#define pulsePer90 20

#define dir 0
#define motorStep 7
#define motorDir 8

//int duty;
int numSteps;

void setup() {

  // Initialize the Serial port:
  Serial.begin(9600);

  // Set up the step and dir lines as digital outputs. Each pulse to step corresponds to one [micro]step of the stepper motor in the direction selected by the DIR pin.
  pinMode(motorStep, OUTPUT);
  pinMode(motorDir, OUTPUT);
//  duty = dutyCyc/100*255;
  numSteps = numDegrees * 5.75 / 10; // [degrees / (degrees/step)]
  
}

void loop() {
  Serial.println(dir);
  digitalWrite(motorDir, dir);
  
  for(int i = 0; i < numSteps; i++)
  {
    // Move the motor a single step with a duty cycle of 25%.
    digitalWrite(motorStep, LOW);
    //Serial.println("motor rising edge");
    delay(2);
    
    digitalWrite(motorStep, HIGH);
    //Serial.println("motor falling edge");
    delay(2);
    
    //Serial.println(i);
    //Serial.println();
  }

  Serial.println("pause\n");
  delay(2000);
}

