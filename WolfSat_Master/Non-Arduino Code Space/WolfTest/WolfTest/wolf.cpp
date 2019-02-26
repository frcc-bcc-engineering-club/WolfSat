#include "Wolf.h"


Wolf::Wolf()
{
	pinOut = WolfPins();
	debugger = WolfDebug(pinOut.getPin_DEBUG());
}


Wolf::~Wolf()
{
}


void Wolf::taskManager()
{
	// do stuff...
}


WolfPins Wolf::get_pinOut() { return pinOut; }
WolfDebug Wolf::get_debugger() { return debugger; }