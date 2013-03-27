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
#define IR_edge0air 190
#define IR_edge1air 310
#define IR_edge0shingle 170
#define IR_edge1shingle 300

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
#define FORWARD 1               // Move Forward
#define BACK 2                  // Move Backward

CMUcam4 cam(CMUCOM4_SERIAL1);
int error;
double est_err;
double cur_angle;
double A,B;
int state;

int sensorval0[10] = {0,0,0,0,0,0,0,0,0,0};
int sensorval1[10] = {0,0,0,0,0,0,0,0,0,0};
int cur_idx = 0;
/*
0:stop
1:moving forward
2:swiching line
3:moving backward
*/
int current_dir=0;

void setup()
{
  Serial.begin(BAUD_RATE);
  //CAMERA_INIT();
  DCM_INIT();
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
      if (current_dir==0)
        state = 1;
      else 
        state = 3;
      break;
//    case 2:
//      //edge_sensor0_roof_in
//      break;
    case 3:
      //edge_sensor1_roof_out switch line
        state = 2;

      break;
    default: 
      break;
  }
  
  //state 
  switch (state) 
    {
    case 0:
      //stop
      DCM_BRAKE();
      Serial.println("stop");
      break;
    case 1:
      //moving
      Serial.println("moving backward");
      DCM_MOVE(255,BACK);
      break;
    case 2:
      //switching line
      Serial.println("switching");
      for(int i = 0; i < 10; i++)
      {
        sensorval0[i] = 0;
        sensorval1[i] = 0;
      }
      DCM_MOVE(255,FORWARD);
      delay(800);
      DCM_ROTATE(255,RIGHT);
      delay(2000);
      DCM_MOVE(255,FORWARD);
      delay(5000);
      DCM_ROTATE(255,LEFT);
      delay(2000);
      current_dir = 1;
      break;
     case 3:
     //backwards
     Serial.println("moving forward");
     DCM_MOVE(255,FORWARD);
     break;
    default: 
      break;
  }
}

 
/**
 * @brief Camera Init
 *
 * @param void
 * @return void
 */
void CAMERA_INIT()
{
  error = cam.begin();

  if(error < CMUCAM4_RETURN_SUCCESS)
  {
    Serial.print("CMUCam Fail with error ");
    Serial.println(error);
    while(1) {}
  }

  // Wait for auto gain and auto white balance to run.
  cam.LEDOn(LED_BLINK);
  delay(WAIT_TIME);

  // Turn on Poll Mode and Line Mode
  cam.pollMode(true);
  cam.lineMode(true); 

  // Turn on Noise Filter
  cam.noiseFilter(NOISE_FILTER);

  // Turn auto gain and auto white balance off.
  cam.autoGainControl(false);
  cam.autoWhiteBalance(false);
  
  // Set Tracking Parameters
  cam.setTrackingParameters(RED_MIN, RED_MAX, GREEN_MIN, GREEN_MAX, BLUE_MIN, BLUE_MAX);
  
  // Indicate all initialization is finished
  cam.LEDOn(CMUCAM4_LED_ON);
}

/**
 * @brief Takes linear regrassion of current bitmap within thresholds set above
 *        and sets angle determined by linear regression
 *
 * @param void
 * @return void
 */
void track_line()
{
  LinearRegression lr;

  // Contains Centroid coordinates and bounding box coordinates
  CMUcam4_tracking_data_t packetT;

  // Contains 60 x 80 binary image that CMUCam currently sees
  CMUcam4_image_data_t packetF;
  int numPixels = 0;
  
  cam.trackColor(); // Initialize color parameters
  cam.getTypeTDataPacket(&packetT); // Tracking Data
  cam.getTypeFDataPacket(&packetF); // Image Data
  
  // Linear Regression
  if(packetT.pixels)
  {
    for(int y = 0; y < CMUCAM4_BINARY_V_RES; y++)
    {
      for(int x = 0; x < CMUCAM4_BINARY_H_RES; x++)
      {
        if(cam.getPixel(&packetF, y, x))
        {
          // Normalized to Cartesian Coordinates (0 - 159, 0 - 119) -> (-80 - 79, 60 - -59)
          lr.addXY(x - 80, 59 - y);
          numPixels++;
        }
      }
    }
    
    Serial.print("Number of pixels tracked is ");
    Serial.println(numPixels);
    
    if(lr.haveData())
    {   
      A = lr.getA();
      B = lr.getB();
      est_err = lr.getStdErrorEst();
      cur_angle = atan2(B,1)*180/PI;
      
      if(est_err > 12)
      {
        cur_angle = 90;
      }
      
      Serial.print(A);
      Serial.print(" + ");
      Serial.print(B);
      Serial.println("x");
      
      Serial.print("Margin of Error is ");
      Serial.println(est_err);
      
      Serial.print("The angle of this line is ");
      Serial.println(cur_angle);
    }
    
    Serial.println();
  }
}


/**
 * @brief DC Motor Init
 *
 * @param void
 * @return void
 */
void DCM_INIT()
{
  // Init for Motor 1
  pinMode(M1_DIR_ONE, OUTPUT);   // init L2
  pinMode(M1_DIR_TWO, OUTPUT);   // init L1
  pinMode(M1_ENABLE, OUTPUT);    // init enable
  
  analogWrite(M1_ENABLE,OFF);    // turn DC motor off
  digitalWrite(M1_DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(M1_DIR_TWO, LOW);
  
  // Init for Motor 2
  pinMode(M2_DIR_ONE, OUTPUT);   // init L2
  pinMode(M2_DIR_TWO, OUTPUT);   // init L1
  pinMode(M2_ENABLE, OUTPUT);    // init enable
  
  analogWrite(M2_ENABLE,OFF);    // turn DC motor off
  digitalWrite(M2_DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(M2_DIR_TWO, LOW);
}

/**
 * @brief Stops DC Motors by braking
 *
 * @param void
 * @return void
 */
void DCM_BRAKE()
{
  analogWrite(M1_ENABLE,OFF);      // turn DC motor off
  digitalWrite(M1_DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(M1_DIR_TWO, LOW);
  
  analogWrite(M2_ENABLE,OFF);      // turn DC motor off
  digitalWrite(M2_DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(M2_DIR_TWO, LOW);
}

/**
 * @brief Makes DC Motors move dir at speed rpm
 *
 * @param desired_speed desired speed of rotation
 * @param dir rotate right or left
 * @return void
 */
void DCM_MOVE(int desired_speed, int dir)
{
  if(dir == FORWARD)
  {
    digitalWrite(M1_DIR_ONE, HIGH);
    digitalWrite(M1_DIR_TWO, LOW);
    
    digitalWrite(M2_DIR_ONE, HIGH);
    digitalWrite(M2_DIR_TWO, LOW);
  }
  
  else
  {
    digitalWrite(M1_DIR_ONE, LOW);
    digitalWrite(M1_DIR_TWO, HIGH);
    
    digitalWrite(M2_DIR_ONE, LOW);
    digitalWrite(M2_DIR_TWO, HIGH);
  }
  
  analogWrite(M1_ENABLE, desired_speed);
  analogWrite(M2_ENABLE, desired_speed);
}

/**
 * @brief Makes DC Motors rotate dir at speed rpm
 *
 * @param desired_speed desired speed of rotation
 * @param dir rotate right or left
 * @return void
 */
void DCM_ROTATE(int desired_speed, int dir)
{
  if(dir == LEFT)
  {
    digitalWrite(M1_DIR_ONE, LOW);
    digitalWrite(M1_DIR_TWO, HIGH);
    
    digitalWrite(M2_DIR_ONE, HIGH);
    digitalWrite(M2_DIR_TWO, LOW);
  }
  
  else
  {
    digitalWrite(M1_DIR_ONE, HIGH);
    digitalWrite(M1_DIR_TWO, LOW);
    
    digitalWrite(M2_DIR_ONE, LOW);
    digitalWrite(M2_DIR_TWO, HIGH);
  }
  
  analogWrite(M1_ENABLE, desired_speed);
  analogWrite(M2_ENABLE, desired_speed);
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
  
  sensorval0[cur_idx] = sensorValue0;
  sensorval1[cur_idx] = sensorValue1;
  cur_idx = (cur_idx + 1) % 10;
  
  sensorValue0 = sensorval0[0] + sensorval0[1] + sensorval0[2] + sensorval0[3] + sensorval0[4];
  sensorValue0 = sensorValue0 + sensorval0[5] + sensorval0[6] + sensorval0[7] + sensorval0[8] + sensorval0[9];
  sensorValue0 = sensorValue0 / 10;
  
  sensorValue1 = sensorval1[0] + sensorval1[1] + sensorval1[2] + sensorval1[3] + sensorval1[4];
  sensorValue1 = sensorValue1 + sensorval1[5] + sensorval1[6] + sensorval1[7] + sensorval1[8] + sensorval1[9];
  sensorValue1 = sensorValue1 / 10;
  
  // print out the value you read:
  Serial.print("sensor 0 = " );                       
  Serial.print(sensorValue0);      
  Serial.print("\t sensor 1 = ");      
  Serial.println(sensorValue1);
  
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

  return 1;
}
