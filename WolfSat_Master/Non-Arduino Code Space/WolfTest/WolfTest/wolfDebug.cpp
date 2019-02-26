#include "wolfDebug.h"

wolfDebug::wolfDebug()
{
	debugMode = false;
}


wolfDebug::wolfDebug(int debugPin)
{
	debugMode = false;
	//setup_Debug(debugPin);
}


wolfDebug::~wolfDebug()
{
}


bool wolfDebug::get_debugMode() { return debugMode; }


//void wolfDebug::setup_Debug(int debugPin)
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