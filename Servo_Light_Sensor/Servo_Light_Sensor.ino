// RC Servo plus light sensor control

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // a maximum of eight servo objects can be created 
 
int sensorPin = A1; //selected A1 analog input to control servo using light sensor
int sensorDelay = 2000; //time to wait before next step of code (1000) was old time
int sensorValue = 0; // variable to store the value coming from the sensor
 
void setup() 
{ 
  myservo.attach(9);  // attaches the servo on pin 9 to the servo object
  Serial.begin(9600); 
} 
 
 
void loop() 
{
  // read the value from the sensor:
  sensorValue = analogRead(sensorPin);
  delay(sensorDelay);
  Serial.println(sensorValue); 
  delay(sensorDelay);
}
