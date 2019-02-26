#include "WolfDebug.h"

WolfDebug::WolfDebug()
{
	debugMode = false;
}


WolfDebug::WolfDebug(int debugPin)
{
	debugMode = false;
	//setup_Debug(debugPin);
}


WolfDebug::~WolfDebug()
{
}


bool WolfDebug::get_debugMode() { return debugMode; }


//void WolfDebug::setup_Debug(int debugPin)
//{
//	if (digitalRead(debugPin) == HIGH)
//	{
//		Serial.begin(9600);
//		Serial.println("WOLFSAT DEBUG MODE");
//		debugMode = true;
//	}
//	else
//		debugMode = false;
//}