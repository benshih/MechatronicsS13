/** @file CMUCAM_MOTOR_IR_integrated.ino
 *  @brief This file contains the line following and row transition code for the shingler
 *
 *  @author Ram Muthiah (rmuthiah)
 *  @bug Line Lost Case is not accounted for
 *       Must add servos and motors
 *       Add Row Transition
 */

// linreg.h and linreg.cpp can be found at
// https://github.com/benshih/MechatronicsS13/tree/master/linreg

#include <linreg.h>
#include <CMUcam4.h>
#include <CMUcom4.h>

// Camera Tracking Parameters
#define RED_MIN 110
#define RED_MAX 220
#define GREEN_MIN 90
#define GREEN_MAX 200
#define BLUE_MIN 50
#define BLUE_MAX 170
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
#define IR_edge0air 450         // Threshold for detection of edge with IR sensor
#define IR_edge1air 450
int IR0_EDGE = 0;               // Sensor Status with respect to the edge
int IR1_EDGE = 0;

void setup()
{
  Serial.begin(BAUD_RATE);
  CAMERA_INIT();
  DCM_INIT();
}

void loop()
{
  FOLLOW_LINE(BACK);
  ROW_TRANSITION();
  FOLLOW_LINE(FORWARD);
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
 * @brief Straightens robot to line
 *
 * @param void
 * @return void
 */
void STRAIGHTEN()
{
  while(1)
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
        DCM_ROTATE(60 + int(cur_angle), LEFT);
        Serial.println(60 + int(cur_angle));
      }
      
      else if(cur_angle < -2)
      {
        DCM_ROTATE(60 - int(cur_angle), RIGHT);
        Serial.println(-60 + int(cur_angle));
      }
      
      else
      {
        return;
      }
    }
  }
}

/**
 * @brief Edge Detection
 *        sensor0 is attached to Pin A0
 *        sensor1 is attached to Pin A1
 *        Sets status of edge variables
 *
 * @param void
 * @return void
 */
void EDGE_DET()
{
  // read the input on analog pin 0 and 1 and save status
  if(analogRead(A0) < IR_edge0air)
  {
    IR0_EDGE = 1;
  }
  else
  {
    IR0_EDGE = 0;
  }
  
  if(analogRead(A1) < IR_edge1air)
  {
    IR1_EDGE = 1;
  }
  else
  {
    IR1_EDGE = 0;
  }

  Serial.print("sensor 0 = " );                       
  Serial.print(IR0_EDGE);      
  Serial.print("\t sensor 1 = ");      
  Serial.println(IR1_EDGE);
}

/**
 * @brief Follow Line in direction specified by input
 *
 * @param driection to move in along line
 * @return void
 */
void FOLLOW_LINE(int dir)
{
  while(1)
  {
    EDGE_DET();
    if(dir == FORWARD)
    {
      Serial.println("Going Forward");
      if(IR0_EDGE)
      {
        DCM_BRAKE();
        return;
      }
    }
    else
    {
      Serial.println("Going Backward");
      if(IR1_EDGE)
      {
        DCM_BRAKE();
        return;
      }
    }
    
    STRAIGHTEN();
    
    if(dir == FORWARD)
    {
      DCM_MOVE(255,FORWARD);
    }
    else
    {
      DCM_MOVE(255,BACK);
    }
  }
}

/**
 * @brief Move up row from one edge
 *        Initial assumption that robot is stopped
 *
 * @param original direction of movement
 * @return void
 */
void ROW_TRANSITION()
{
  DCM_ROTATE(255,RIGHT);
  delay(1000);
  DCM_MOVE(255,FORWARD);
  delay(1500);
  while(1)
  {
    track_line();
    if(numPixels > NUM_PIXELS_NOT_NOISE)
    {
      if(A > 15 && A < 75)
      {
        STRAIGHTEN();
        return;
      }
    }
  }
}
