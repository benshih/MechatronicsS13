/*
  AnalogReadSerial
  Reads an analog input on pin 0&2, prints the result .
  Sensor 0 is attached to pin A0, Sensor 1 is attached to pin A1 and the outside pins to +5V and ground.
  when out of roof the read is 190
  when within roof the read is 190-210(1); 80-90(2)
  when detect the single the read is 
 */

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial.begin(9600);
}

// the loop routine runs over and over again forever:
void loop() {
  // read the input on analog pin 0:
  int sensorValue0 = analogRead(A0);
  // read the input on analog pin 1:
  int sensorValue1 = analogRead(A1);
  // print out the value you read:
  Serial.print("sensor L = " );                       
  Serial.print(sensorValue0);      
  Serial.print("\t sensor R = ");      
  Serial.println(sensorValue1);
  delay(10);        // delay in between reads for stability
}
