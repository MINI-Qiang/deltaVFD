#include "deltaVFD.h"
deltaVFD VFD(1);
void setup()
{
  delay(1000);
  Serial.begin(9600, SERIAL_8E1);
  VFD.begin(Serial);

}

void loop()
{
  VFD.SetF(3000);
  VFD.run(1);
  delay(10000);

  VFD.stop();
  delay(10000);

  VFD.SetF(4000);
  VFD.run(0);
  delay(10000);

  VFD.stop();
  delay(10000);
}