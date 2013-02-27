#include <linreg.h>
#include <CMUcam4.h>
#include <CMUcom4.h>

#define RED_MIN 230
#define RED_MAX 255
#define GREEN_MIN 230
#define GREEN_MAX 255
#define BLUE_MIN 230
#define BLUE_MAX 255
#define LED_BLINK 5 // 5 Hz
#define WAIT_TIME 5000 // 5 seconds
#define BAUD_RATE 9600
#define NOISE_FILTER 2

CMUcam4 cam(CMUCOM4_SERIAL3);
int error;
double A,B;

void setup()
{
  Serial.begin(BAUD_RATE);
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

void loop()
{
  track_line();
}

void track_line()
{
  LinearRegression lr;
  CMUcam4_tracking_data_t packetT;
  CMUcam4_image_data_t packetF;
  int numPixels = 0;
  
  cam.trackColor();
  cam.getTypeTDataPacket(&packetT);
  cam.getTypeFDataPacket(&packetF);
  
  if(packetT.pixels)
  {
    for(int y = 0; y < CMUCAM4_BINARY_V_RES; y++)
    {
      for(int x = 0; x < CMUCAM4_BINARY_H_RES; x++)
      {
        if(cam.getPixel(&packetF, y, x))
        {
          lr.addXY(x, y);
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

      Serial.print(A);
      Serial.print(" + ");
      Serial.print(B);
      Serial.println("x");
    }
    
    Serial.println();
  }
}

