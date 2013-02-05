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

#define motorSteps 200     // change this depending on the number of steps
                           // per revolution of your motor
#define dir 0
#define dutyCyc 50
#define motorStep 7
#define motorDir 8

void setup() {

  // Initialize the Serial port:
  Serial.begin(9600);

  // Set up the step and dir lines as digital outputs. Each pulse to step corresponds to one [micro]step of the stepper motor in the direction selected by the DIR pin.
  pinMode(motorStep, OUTPUT);
  pinMode(motorDir, OUTPUT);
  int duty = dutyCyc/100*255;
}

void loop() {
  Serial.println(dir);
  digitalWrite(motorDir, dir);
  
  analogWrite(motorStep, duty);
}

