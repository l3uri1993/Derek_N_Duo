/**Istruzioni per il debug
 * 
 * Ci sono vari DEFINE per abilitare/disabilitare il debug con seriale
 * In Gyro.cpp
 * In Robot.cpp
 * 
 * Su ogni define è definito il suo scopo
 */
 
#include <Wire.h>
#include "Robot.h"

using namespace Derek;

///Derek Robot object created
Robot Derek_2;

void setup()
{
  Wire.begin();
  Derek_2.setup();
}

void loop()
{
  Derek_2.run();
}
