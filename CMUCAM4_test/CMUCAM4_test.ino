#include <CMUcam4.h>
#include <CMUcom4.h>

#define TEST_PORT CMUCOM4_SERIAL3

#define TEST_BAUD 19200 // Startup baud rate.
#define TEST_WAIT 5000 // Startup wait time in milliseconds.

CMUcam4 cam(TEST_PORT);

void setup()
{
  Serial.begin(TEST_BAUD);
  Serial.println("Testing Serial Port");
  delay(TEST_WAIT);

  cam.begin();

  delay(TEST_WAIT);
  if(cam.begin() == CMUCAM4_RETURN_SUCCESS)
  {
      Serial.println("The camera startup was successful");
  }
  else
  {
      Serial.println("Camera failed to load...quitting program");
  }
}  

void loop()
{
  return;
}
