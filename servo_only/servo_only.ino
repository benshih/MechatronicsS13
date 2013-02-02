// RC Servo only.

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object 
} 
 
 
void loop() 
{
  myservo.write(0); // Set to one extreme.
  delay(1000);
  myservo.write(180); // Set to other extreme.
  delay(1000);
  myservo.write(90); // Set to 
  delay(1000);
}
