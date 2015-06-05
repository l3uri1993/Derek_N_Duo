/*Istruzioni per il debug
 * Ci sono vari DEFINE per abilitare/disabilitare il debug
 * In Gyro.cpp
 * In Robot.cpp
 * in Derek_N_Duo.ino
 * 
 * Su ogni define è definito il suo scopo
 */
 
#include <Wire.h>
#include "Robot.h"

using namespace Derek;

//#define ENABLE_SERIAL         //Abilita il debug in generale per il monitor seriale

///Derek Robot object created
Robot Derek_2;

void setup()
{
  pinMode(14, OUTPUT);    ///Led per verificare funzionamento
  pinMode(15, OUTPUT);
  digitalWrite(15, LOW);
  digitalWrite(14, HIGH);
#ifdef ENABLE_SERIAL
  Serial.begin(9600);
#endif
  /////////////////////////////////////////////////////
  Wire.begin();           ///Parte sempre necessaria
  Derek_2.setup();
}

void loop()
{
  Derek_2.run();
}
