// linreg.h and linreg.cpp can be found at
// https://github.com/benshih/MechatronicsS13/tree/master/linreg

#include <linreg.h>
#include <CMUcam4.h>
#include <CMUcom4.h>
//state mechine

// Camera Tracking Parameters
#define RED_MIN 110
#define RED_MAX 196
#define GREEN_MIN 40
#define GREEN_MAX 75
#define BLUE_MIN 0
#define BLUE_MAX 48

// Camera LED and Init Constants
#define LED_BLINK 5 // Hz
#define WAIT_TIME 5000 // 5 seconds

//Edge Dection
#define IR_edge0air 200
#define IR_edge1air 270
#define IR_edge0shingle 180
#define IR_edge1shingle 260

// Noise Parameter
#define NOISE_FILTER 6

// Serial Rate
#define BAUD_RATE 9600

// DC Motor Constants
#define M1_DIR_ONE 4            // Motor Board L2
#define M1_DIR_TWO 5            // Motor Board L1
#define M1_ENABLE 6             // Motor Board Enable
#define M2_DIR_ONE 7            // Motor Constants for Motor 2
#define M2_DIR_TWO 8
#define M2_ENABLE 9
#define OFF 0                   // Defines DC Motor Off
#define RIGHT 1                 // Rotate Right
#define LEFT 2                  // Rotate Left

CMUcam4 cam(CMUCOM4_SERIAL1);
int error;
double est_err;
double cur_angle;
double A,B;
int state=0;/*
0:stop
1:moving
2:swiching line
3:retracking line

*/
void setup()
{
  Serial.begin(BAUD_RATE);
//  CAMERA_INIT();
//  DCM_INIT();
}

void loop()
{
  //edge detection
    switch (EDGE_DET()) 
    {
    case 0:
      //edge_sensor0_roof_out stop
      state = 0;
      break;
    case 1:
      //edge_sensor0_shingle
      //Serial.print("shingle detected");
      state = 1;
      break;
//    case 2:
//      //edge_sensor0_roof_in
//      break;
    case 3:
      //edge_sensor1_roof_out switch line
      state = 2;
      break;
    case 4:
      //edge_sensor1_shingle
      state = 1;
      break;
//    case 5:
//      //edge_sensor1_roof_in
//      break;
    default: 
      break;
  }
  
  //state 
  switch (state) 
    {
    case 0:
      //stop
//      DCM_BRAKE();
      Serial.println("stop");
      break;
    case 1:
      //moving
      Serial.println("moving");
      break;
    case 2:
      //switching line
      Serial.println("switching");
      break;
//    case 3:
//      //tracking line.
//      state = 2;
      break;
    default: 
      break;
  }
}    
/**
 * @brief Edge Detection
 * sensor0 is attached to Pin A0
 * sensor1 is attached to Pin A1
 * @param void
 * @return 0:roof_out,roof_in,shinge_det 
 *
 */
int EDGE_DET()
{
  
  // read the input on analog pin 0:
  int sensorValue0 = analogRead(A0);
  // read the input on analog pin 1:
  int sensorValue1 = analogRead(A1);
  // print out the value you read:
  Serial.print("sensor 0 = " );                       
  Serial.print(sensorValue0);      
  Serial.print("\t sensor 1 = ");      
  Serial.println(sensorValue1);
  delay(1);        // delay in between reads for stability
  //sensor0
  if (sensorValue0 > IR_edge0air) 
  {
    return 0; //if root_out return 0
  }
  else if(sensorValue0 < IR_edge0shingle)
  {
    sensorValue0 = 1;//if detect shingle return 1
  }
//  else return 2;//if roof_in return 2
  
 //sensor1
  if (sensorValue1 > IR_edge1air) 
  {
    return 3; //if root_out return 3
  }
  else if(sensorValue1 < IR_edge1shingle)
  {
    sensorValue1 = 1;//if detect shingle return 4
  }
  //else if return 5;//if roof_in return 5
  if (sensorValue0 = sensorValue1)
  return 1;
}
