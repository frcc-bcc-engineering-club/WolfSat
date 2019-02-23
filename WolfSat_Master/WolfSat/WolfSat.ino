#include <SparkFun_Qwiic_OpenLog_Arduino_Library.h>
#include <Wire.h>
#include <wolf.h>

wolf wolfSat;
wolfPins wolfSat_Pins;
OpenLog logOne;

bool debugMode;


void setup() 
{
  setup_Wolf();
  setup_Debug();
  setup_Wire();

}


void setup_Wolf()
{
  wolfSat = wolf();
  wolfSat_Pins = wolfSat.get_pinOut();
}


void setup_Debug()
{
  if(digitalRead(wolfSat_Pins.getPin_DEBUG()) == HIGH)
  {
    Serial.begin(9600);
    Serial.println("WOLFSAT DEBUG MODE");
    debugMode = true;
  }
  else
    debugMode = false;
}


void setup_Wire()
{
  Wire.begin();
  logOne.begin();
  
  if(debugMode)
    Serial.println("WIRE SETUP COMPLETE");
}


void loop() {
  // put your main code here, to run repeatedly:
  int x = 0;
  x++;
  logOne.println("X = " + String(x));
  if(debugMode)
    Serial.println("X = " + String(x));
  digitalWrite(wolfSat_Pins.getPin_LED(), HIGH);
  delay(1000);
  digitalWrite(wolfSat_Pins.getPin_LED(), LOW);
  delay(1000);
}



