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

CMUcam4 cam(CMUCOM4_SERIAL3);

#define X1 0
#define Y1 0
#define X2 X1+79
#define Y2 119
#define X3 X2+1
#define Y3 Y1
#define X4 X3+79
#define Y4 Y2

int cent_x1,cent_x2,cent_x3,cent_x4,cent_x5,cent_x6,cent_x7,cent_x8;
int cent_y1,cent_y2,cent_y3,cent_y4,cent_y5,cent_y6,cent_y7,cent_y8;
CMUcam4_tracking_data_t data;

void setup()
{
  Serial.begin(BAUD_RATE);
  cam.begin();

  // Wait for auto gain and auto white balance to run.

  cam.LEDOn(LED_BLINK);
  delay(WAIT_TIME);

  // Turn auto gain and auto white balance off.

  cam.autoGainControl(false);
  cam.autoWhiteBalance(false);
  cam.LEDOn(CMUCAM4_LED_ON);
}

void loop()
{
  print(X1,Y1,X4,Y4);
  print(X1,Y1,X2,Y2);
  print(X3,Y3,X4,Y4);
  
  Serial.println();
}

void print(int x_low, int y_low, int x_high, int y_high)
{
  cam.setTrackingWindow(x_low,y_low,x_high,y_high);
  cam.trackColor(RED_MIN, RED_MAX, GREEN_MIN, GREEN_MAX, BLUE_MIN, BLUE_MAX);
  cam.getTypeTDataPacket(&data);
  Serial.print(data.mx);
  Serial.print(",");
  Serial.println(data.my);
}
