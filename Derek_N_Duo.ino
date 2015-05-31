#include <Wire.h>
#include "Robot.h"

using namespace Derek;

///Derek Robot object created
Robot Derek_N_Duo;

void setup()
{
  //Serial.begin(9600);
  Wire.begin();
  Derek_N_Duo.setup();
}

void loop()
{
  Derek_N_Duo.run();
}
