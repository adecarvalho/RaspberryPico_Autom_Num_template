#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <picoLib.h>

// const
const uint8_t LED_PIN = 21;
const uint8_t BUTTON_PIN = 20;
const uint32_t PERIOD_MS = 100;

// prototype
void action();
void buttonHandler();

// obj
DigitalOut mled;
Button mbutton;
SoftTimer mtimer;

// variables globales
bool isPressed = false;

//*******************
void action()
{
  if (isPressed)
  {
    return;
  }
  else
  {
    mled.toggle();
  }
}
//****************
void buttonHandler()
{
  mbutton.read();
  //
  if (mbutton.isReleased())
  {
    isPressed = true;
    Serial.println("button realeased");
    mled.write(0);
  }
}
//**************
void setup()
{
  Serial.begin(115200);
  //
  mled.begin(LED_PIN, 0);
  //
  mbutton.begin(BUTTON_PIN);
  //
  mtimer.attach(action, PERIOD_MS);
  mtimer.start();
}
//****************
void loop()
{
  mtimer.update();
  buttonHandler();
}
