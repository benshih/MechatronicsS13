/** @file CMUCAM_MOTOR_IR_integrated.ino
 *  @brief This file contains the line following and row transition code for the shingler
 *
 *  @author Ram Muthiah (rmuthiah)
 *  @bug Line Lost Case is not accounted for
 *       Must add servos and motors
 *       Must add edge detection
 *       Add Row Transition
 */

// linreg.h and linreg.cpp can be found at
// https://github.com/benshih/MechatronicsS13/tree/master/linreg

#include <linreg.h>
#include <CMUcam4.h>
#include <CMUcom4.h>

// Camera Tracking Parameters
#define RED_MIN 120
#define RED_MAX 200
#define GREEN_MIN 100
#define GREEN_MAX 180
#define BLUE_MIN 60
#define BLUE_MAX 150
#define NUM_PIXELS_NOT_NOISE 50
#define NO_IMAGE_FOUND 181

// Camera LED and Init Constants
#define LED_BLINK 5 // Hz
#define WAIT_TIME 5000 // 5 seconds

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

CMUcam4 cam(CMUCOM4_SERIAL1);   // Serial Port CMUCam is attached to
int error;                      // CMUCAM error detection on startup
double cur_angle;               // Current angle of tracked line, insignificant if
                                // numPixels is 0
int numPixels = -1;             // Number of pixels tracked in most recent bitmap
double A,B;                     // A + Bx,slope and intercept of tracked line
                                // insignificant if numPixels is 0

// Edge Detection Constants
#define IR_edge0air 500
#define IR_edge1air 500
//init for IR averaging 
int sensorval0[10] = {770,770,770,770,770,770,770,770,770,770};
int sensorval1[10] = {770,770,770,770,770,770,770,770,770,770};
int cur_idx = 0;                //curent operated IR sensorvalX[]

void setup()
{
  Serial.begin(BAUD_RATE);
  CAMERA_INIT();
  DCM_INIT();
}

//*************************state machine_test
switch ()

//*************************
void loop()
{
  track_line();
  
  if(cur_angle == NO_IMAGE_FOUND || numPixels < NUM_PIXELS_NOT_NOISE)
  {
    // FIND LINE FUNCTION SHOULD BE CALLED HERE
    DCM_BRAKE();
    Serial.print("numPixels is ");
    Serial.print(numPixels);
    Serial.print(" and cur_angle is ");
    Serial.print(cur_angle);
  }
  else
  {
    Serial.print("The current speed of rotation is ");
    if(cur_angle > 2)
    {
      DCM_ROTATE(50 + int(cur_angle), RIGHT);
      Serial.println(50 + int(cur_angle));
    }
    
    else if(cur_angle < -2)
    {
      DCM_ROTATE(50 - int(cur_angle), LEFT);
      Serial.println(-50 + int(cur_angle));
    }
    
    else
    {
      DCM_MOVE(255,BACK);
      Serial.println(0);
    }
  }
  
  Serial.println();
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
  double est_err;
  
  // Contains Centroid coordinates and bounding box coordinates
  CMUcam4_tracking_data_t packetT;

  // Contains 60 x 80 binary image that CMUCam currently sees
  CMUcam4_image_data_t packetF;
  
  cam.trackColor(); // Initialize color parameters
  cam.getTypeTDataPacket(&packetT); // Tracking Data
  cam.getTypeFDataPacket(&packetF); // Image Data
  
  numPixels = 0;
  cur_angle = NO_IMAGE_FOUND;
  
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

    if(lr.haveData())
    {   
      A = lr.getA();
      B = lr.getB();
      est_err = lr.getStdErrorEst();
      cur_angle = atan2(B,1)*180/PI;
      
      //CHECK THIS TO ENSURE CORRECTNESS
      if(est_err > 12)
      {
        cur_angle = 90;
      }
      
      Serial.print(A);
      Serial.print(" + ");
      Serial.print(B);
      Serial.println("x");
      
      Serial.print("The angle of this line is ");
      Serial.println(cur_angle);
    }
    
    Serial.println();
  }
}

/**
 * @brief Edge Sensor Init
 *
 * @param void
 * @return void
 */
void EDGE_INIT()
{
  // Init for Power for IR1 and IR2
  pinMode(M1_DIR_ONE, OUTPUT);   // init L2
  pinMode(M1_DIR_TWO, OUTPUT);   // init L1
  
  digitalWrite(M1_DIR_ONE, HIGH); // Both Direction pins low means the motor has braked
  digitalWrite(M1_DIR_TWO, HIGH);
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
  if(dir == RIGHT)
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
  
  //Do averge on 10 :
  sensorval0[cur_idx] = sensorValue0;
  sensorval1[cur_idx] = sensorValue1;
  cur_idx = (cur_idx + 1) % 10;
  
  sensorValue0 = sensorval0[0] + sensorval0[1] + sensorval0[2] + sensorval0[3] + sensorval0[4];
  sensorValue0 = sensorValue0 + sensorval0[5] + sensorval0[6] + sensorval0[7] + sensorval0[8] + sensorval0[9];
  sensorValue0 = sensorValue0 / 10;
  
  sensorValue1 = sensorval1[0] + sensorval1[1] + sensorval1[2] + sensorval1[3] + sensorval1[4];
  sensorValue1 = sensorValue1 + sensorval1[5] + sensorval1[6] + sensorval1[7] + sensorval1[8] + sensorval1[9];
  sensorValue1 = sensorValue1 / 10;
  
  // print out the value it read after averaging
  Serial.print("sensor 0 = " );                       
  Serial.print(sensorValue0);      
  Serial.print("\t sensor 1 = ");      
  Serial.println(sensorValue1);

//sensor0(left)  
  if (sensorValue0 < IR_edge0air) 
  {
    return 0; //if left_roofout return 0
  }
  
 //sensor1(right)
  if (sensorValue1 < IR_edge1air) 
  {
    return 2; //if right_rootout return 2
  }

  return 1;//both roofon return 1
}
