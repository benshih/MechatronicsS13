/** @file CMUCAM_MOTOR_IR_integrated.ino
 *  @brief This file contains the line following and row transition code for the shingler
 *
 *  @author Ram Muthiah (rmuthiah)
 *          Hao Wang (haow1)
 *
 *  @bug Line Lost Case is not accounted for
 *       Must add servos and motors
 *       Remove Magic Numbers
 *       Translational Error Correcting
 */

// linreg.h and linreg.cpp can be found at
// https://github.com/benshih/MechatronicsS13/tree/master/linreg

#include <linreg.h>
#include <CMUcam4.h>
#include <CMUcom4.h>
#include <Servo.h> 

// Camera Tracking Parameters
#define RED_MIN 145
#define RED_MAX 195
#define GREEN_MIN 140
#define GREEN_MAX 170
#define BLUE_MIN 105
#define BLUE_MAX 150
#define NUM_PIXELS_NOT_NOISE 25
#define NO_IMAGE_FOUND 181
#define LOWER_CENTER 28         // Limits of A in A + Bx
#define UPPER_CENTER 33

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

// Direction Constants
#define RIGHT 3                 // Rotate Right
#define LEFT 2                  // Rotate Left
#define FORWARD 1               // Move Forward
#define BACK 2                  // Move Backward
#define Edge_L A0               // Edge senxor left
#define Edge_L A1               // Edge senxor right

CMUcam4 cam(CMUCOM4_SERIAL1);   // Serial Port CMUCam is attached to
int error;                      // CMUCAM error detection on startup
double cur_angle;               // Current angle of tracked line, insignificant if
                                // numPixels is 0
int numPixels = -1;             // Number of pixels tracked in most recent bitmap
double A,B;                     // A + Bx,slope and intercept of tracked line
                                // insignificant if numPixels is 0

// Edge Detection Constants
#define IR_edge0air 400         // Threshold for detection of edge with IR sensor
#define IR_edge1air 400
int IR0_EDGE = 0;               // Sensor Status with respect to the edge (0:on roof;1 off roof)
int IR1_EDGE = 0;

// Dropper Constants
#define drop_pos_L 2            // Dropper position detector left side INT0
#define drop_pos_M A2           // Dropper position detector mid side A2
#define drop_pos_R 3            // Dropper position detector right side INT1
#define D1_DIR_ONE 10           // Chain Motor Board L2
#define D1_DIR_TWO 11           // Chain Motor Board L1
#define D1_ENABLE 12            // Chain Motor Board Enable
// #define OFF 0                // Defines Chain Motor Off
// #define LEFT  2              // Dropper pos Left
#define MID   1                 // Dropper pos Mid
// #define RIGHT 3              // Dropper pos Right

// Feeder Constants
#define FD_DIR_ONE 38
#define FD_DIR_TWO 39
#define FD_ENABLE 40
 
Servo servoL;  // create servo object to control servo left
Servo servoR;  // create servo object to control servo Right
int DPpos = RIGHT;                 // Initial dropperchain pos is N/A 

void setup()
{
  Serial.begin(BAUD_RATE);
  // CAMERA_INIT();
  DCM_INIT();
  DPCHAIN_INIT();
  
  Serial.println("Waiting for input");
  while(!Serial.available()){}
  Serial.println("Starting Now");
  
  DP_pos(MID);
  DCM_MOVE(255,BACK);
  while(IR0_EDGE != 1)
  {
    EDGE_DET();
  }
  DCM_BRAKE();
  delay(2000);
}

void loop()
{
  // Feeder spits shingle
  delay(2000);
  // Floor Version
  FEED();
  delay(2000);
  DP_drop();
  delay(1000);
  
  DCM_MOVE(250,FORWARD);
  delay(3800);
  DCM_BRAKE();
  FEED();
  delay(2000);
  DP_drop();
  delay(1000);
  DCM_MOVE(250,FORWARD);
  delay(3800);
  DCM_BRAKE();
  FEED();
  delay(2000);
  DP_drop();
  delay(1000);
  
  DCM_MOVE(255,BACK);
  delay(800);
  DCM_ROTATE(200,RIGHT);
  delay(1200);
  DCM_MOVE(255,BACK);
  delay(3300);
  DCM_ROTATE(200,LEFT);
  delay(1000);
  DCM_MOVE(255,FORWARD);
  while(IR1_EDGE != 1)
  {
    EDGE_DET();
  }
  DCM_BRAKE();       //1st row2
  FEED();
  delay(2000);
  DP_pos(RIGHT);
  DP_drop();
  delay(1000);
  DP_pos(MID);
  
//  DCM_ROTATE(150,RIGHT);
//  delay(300);
  
  DCM_MOVE(250,BACK);//2rd row2
  delay(1900);
  DCM_BRAKE();
  FEED();
  delay(2000);
  DP_drop();
  delay(1000);
  
  DCM_MOVE(250,BACK);//3rd row2
  delay(3800);
  DCM_BRAKE();
  FEED();
  delay(2000);
  DP_drop();
  delay(1000);
  
  DCM_MOVE(225,BACK);//4rd row2
   while(IR0_EDGE != 1)
  {
    EDGE_DET();
  }
  DCM_BRAKE();
  FEED();
  delay(2000);
  DP_pos(LEFT);
  DP_drop();
  DP_pos(MID);
  
  FEED();
  delay(2000);
  DP_drop();
  while(1){};
}

/**
 * @brief DC Motor Init
 *
 * @param void
 * @return void
 */
void DPCHAIN_INIT()
{
  // Init for Chain Motor 
  pinMode(D1_DIR_ONE, OUTPUT);   // init L2
  pinMode(D1_DIR_TWO, OUTPUT);   // init L1
  pinMode(D1_ENABLE, OUTPUT);    // init enable

  analogWrite(D1_ENABLE,OFF);    // turn DC motor off
  digitalWrite(D1_DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(D1_DIR_TWO, LOW);
  
  // initialize interrupt for Dropchain lozolization position left and right
  pinMode(drop_pos_L,INPUT);
  pinMode(drop_pos_R,INPUT);  
  attachInterrupt(0, posL , RISING);
  attachInterrupt(1, posR , RISING);
  
  // initialize servo for the dropper 
  servoR.attach(14);  // attaches the servo on pin 14 to the servoR
  servoL.attach(15);  // attaches the servo on pin 15 to the servoL
  servoL.write(70);   // initialize servo position to hold shingle
  servoR.write(180);
  
  pinMode(FD_DIR_ONE, OUTPUT);   // init L2
  pinMode(FD_DIR_TWO, OUTPUT);   // init L1
  pinMode(FD_ENABLE, OUTPUT);    // init enable

  digitalWrite(FD_ENABLE,LOW);    // turn DC motor off
  digitalWrite(FD_DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(FD_DIR_TWO, HIGH);
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

  while(error < CMUCAM4_RETURN_SUCCESS)
  {
    Serial.print("CMUCam Fail with error ");
    Serial.println(error);
    error = cam.begin();
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
  int i;
  if(dir == BACK)
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
//    delay(5000);
    analogWrite(M2_ENABLE, desired_speed);
    
    
    
//  for(i = 60; i < 250; i+=10) // slowly speed up
//  {
//    analogWrite(M1_ENABLE, i);
//    analogWrite(M2_ENABLE, i);
//    delay(50);
//  }
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
 * @brief Straightens robot to line
 *        Assumes Robot is flush with line
 *
 * @param dir the direction robot is moving in
 * @return void
 */
void STRAIGHTEN(int dir)
{
  while(1)
  {
    track_line();
    if(cur_angle == NO_IMAGE_FOUND || numPixels < NUM_PIXELS_NOT_NOISE)
    {
      DCM_BRAKE();
      Serial.print("numPixels is ");
      Serial.print(numPixels);
      Serial.print(" and cur_angle is ");
      Serial.println(cur_angle);
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
        Serial.println(0);
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

    STRAIGHTEN(dir);
    
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
      // The seven is an estimate
      if(A > (LOWER_CENTER - 7) && A < (UPPER_CENTER + 7))
      {
        STRAIGHTEN(FORWARD);
        return;
      }
    }
  }
}

/**
 * @brief Stops DC Motors by braking
 *
 * @param void
 * @return void
 */
void DPCHAIN_BRAKE()
{
  analogWrite(D1_ENABLE,OFF);      // turn DC motor off
  digitalWrite(D1_DIR_ONE, LOW); // Both Direction pins low means the motor has braked
  digitalWrite(D1_DIR_TWO, LOW);
}

/**
 * @brief Makes DC Motors move dir at speed rpm
 *
 * @param desired_speed desired speed
 * @param dir moving left or right
 * @return void
 */
void DPCHAIN_MOVE(int desired_speed, int dir)
{
  if(dir == LEFT)
  {
    digitalWrite(D1_DIR_ONE, HIGH);
    digitalWrite(D1_DIR_TWO, LOW);
  }
  if (dir == RIGHT)
  {
    digitalWrite(D1_DIR_ONE, LOW);
    digitalWrite(D1_DIR_TWO, HIGH);
  }  
  analogWrite(D1_ENABLE, desired_speed);
}
//interupt:Right position arrived renew Dropper position = RIGHT  
void posR ()
{
  DPpos= RIGHT;
//  Serial.print("DPpos = RIGHT" );                       
//  Serial.println(DPpos);
}
//interupt:Left position arrived renew Dropper position = LEFT 
void posL ()
{
  DPpos = LEFT;
//  Serial.print("DPpos = LEFT" );                       
//  Serial.println(DPpos);
}

/**
 * @brief Dropper actuator servo
 *
 * @param void
 * @return void
 */
/**
 * @brief Dropper actuator servo
 *
 * @param void
 * @return void
 */
void DP_drop()
{
  servoL.write(180);
  servoR.write(60);
  delay(1000);
  servoL.write(70);
  servoR.write(170);
}

/**
 * @brief Dropperchain locolization
 *
 * @param int pos
 * @return void
 */
void DP_pos(int pos)
{
  int dir = 4;
  while (dir != OFF) 
  {
    // read the input on analog pin 3:
    int sensorValueM = analogRead(drop_pos_M);
//    Serial.print("\t sensor M = " );                       
//    Serial.println(sensorValueM);
    if(sensorValueM < 60)
    DPpos = MID;
    if (pos==DPpos)//the actual pose DPpos reaches the goal pose pos
    {
      DPCHAIN_BRAKE();//stop the chain
      dir=OFF;//stop chain
    }
    else
    {
      switch (pos)
      {
      case LEFT:
        dir= LEFT;
        break;
 
       case MID:
         {
           if (DPpos == LEFT)
             dir= RIGHT;
           if (DPpos == RIGHT)
             dir= LEFT;
         }
         break;
 
       case RIGHT:
         dir= RIGHT;
         break;
      }
      DPCHAIN_MOVE(125,dir);  
    } 
  Serial.print(" DPpos = " );                       
  Serial.print(DPpos);
  Serial.print("\t pos = " );                       
  Serial.println(pos);
  Serial.print("\t dir = " );                       
  Serial.println(dir);
  }  
}

void FEED()
{
  digitalWrite(FD_ENABLE,HIGH);
  delay(2000);
  digitalWrite(FD_ENABLE,LOW);
  delay(500);
}
   
