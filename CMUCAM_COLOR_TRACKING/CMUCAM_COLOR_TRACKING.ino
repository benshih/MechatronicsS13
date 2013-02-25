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

CMUcam4 cam(CMUCOM4_SERIAL3);

void setup()
{
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

  for(;;)
  {
    cam.getTypeTDataPacket(&data); // Get a tracking packet.

    // Process the packet data safely here.
  }

  // Do something else here.
}
