#include <BigSet.h>
#include <DataSet.h>
#include <LoggerWrap.h>
#include <SerialLogger.h>
#include <Wolf.h>
#include <WolfDebug.h>
#include <WolfPins.h>

//Wolf wolfSat;
Wolf* wolfStar;

void setup() 
{
  wolfStar = &Wolf();
  pinMode(13, OUTPUT);
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(1000);
  Serial.begin(9600);
  Serial.println("Hello");
  Serial.end();
}

void loop() 
{
  //Wolf wolfSat = Wolf();
  //WolfPins wolfSat_Pins = wolfSat.get_pinOut();
  wolfStar->get_logOne().stringToLog("Test Complete");
  digitalWrite(13, LOW);
  delay(1000);
  Serial.println("Check");
  //wolfSat.get_logOne().stringToLog("Test Complete");
  Serial.println("Mark");
  digitalWrite(13, HIGH);
  delay(1000);
}



