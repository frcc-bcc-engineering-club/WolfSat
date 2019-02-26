#include <Wolf.h>

Wolf wolfSat;
WolfPins wolfSat_Pins;

void setup() 
{
  setup_Wolf();
}


void setup_Wolf()
{
  wolfSat = Wolf();
  wolfSat_Pins = wolfSat.get_pinOut();
}


void loop() {
  // put your main code here, to run repeatedly:
  int x = 0;
  x++;
  if(wolfSat.get_debugger().get_debugMode())
    Serial.println("X = " + String(x));
  digitalWrite(wolfSat_Pins.getPin_LED(), HIGH);
  delay(1000);
  digitalWrite(wolfSat_Pins.getPin_LED(), LOW);
  delay(1000);
}



