#include <Wire.h>
#include "Robot.h"

using namespace Derek;

///Derek Robot object created
Robot Derek_2;

void setup()
{
  pinMode(14, OUTPUT);    ///Parte per il debug
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  digitalWrite(14, HIGH);  
  Serial.begin(9600);
/////////////////////////////////////////////////////
  Wire.begin();           ///Parte sempre necessaria
  Derek_2.setup();
}

void loop()
{
  Derek_2.run();
}
