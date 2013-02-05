#define encoderPin1 2
#define encoderPin2 3
#define dir_1 4
#define dir_2 5
#define enable 6

volatile long encoderValue = 0;
volatile long cur_dir = 0;
volatile long desired_loc = 300;
volatile float error;
volatile float motor_in;
volatile float acc_err = 0;
float KP = 1;
float KI = 0.000;

void setup()
{
  Serial.begin (9600);

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
  delay(100); //just here to slow down the output, and show it will work  even during a delay
  Serial.print(encoderValue);
  Serial.print(" ");
  Serial.println(error);
  error = desired_loc - encoderValue;
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
