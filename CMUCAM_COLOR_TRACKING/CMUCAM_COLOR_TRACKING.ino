#include <CMUcam4.h>
#include <CMUcom4.h>

#define RED_MIN 40
#define RED_MAX 220
#define GREEN_MIN 0
#define GREEN_MAX 110
#define BLUE_MIN 0
#define BLUE_MAX 60
#define LED_BLINK 5 // 5 Hz
#define WAIT_TIME 5000 // 5 seconds
#define BAUD_RATE 9600

CMUcam4 cam(CMUCOM4_SERIAL3);

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
  CMUcam4_tracking_data_t data;

  cam.trackColor(RED_MIN, RED_MAX, GREEN_MIN, GREEN_MAX, BLUE_MIN, BLUE_MAX);

  cam.getTypeTDataPacket(&data); // Get a tracking packet.
  Serial.print(data.mx);
  Serial.print(",");
  Serial.println(data.my);
}
