/**Istruzioni per il debug
 * 
 * Ci sono vari DEFINE per abilitare/disabilitare il debug
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
  pinMode(14, OUTPUT);    ///Led per verificare funzionamento
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  digitalWrite(14, HIGH);
  /////////////////////////////////////////////////////
  Wire.begin();           ///Parte sempre necessaria
  Derek_2.setup();
}

void loop()
{
  Derek_2.run();
}
