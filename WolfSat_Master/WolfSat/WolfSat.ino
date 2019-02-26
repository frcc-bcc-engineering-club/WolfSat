#include <SparkFun_Qwiic_OpenLog_Arduino_Library.h>
#include <Wire.h>
#include <wolf.h>

wolf wolfSat;
wolfPins wolfSat_Pins;
OpenLog logOne;

void setup() 
{
  setup_Wolf();
  setup_Wire();
}


void setup_Wolf()
{
  wolfSat = wolf();
  wolfSat_Pins = wolfSat.get_pinOut();
}


void setup_Wire()
{
  Wire.begin();
  logOne.begin();
  logOne.append("DataTest.txt");
  
  if(wolfSat.get_debugger().get_debugMode())
    Serial.println("WIRE SETUP COMPLETE");
}


void loop() {
  // put your main code here, to run repeatedly:
  int x = 0;
  x++;
  logOne.println("X = " + String(x));
  if(wolfSat.get_debugger().get_debugMode())
    Serial.println("X = " + String(x));
  digitalWrite(wolfSat_Pins.getPin_LED(), HIGH);
  delay(1000);
  digitalWrite(wolfSat_Pins.getPin_LED(), LOW);
  delay(1000);
}



