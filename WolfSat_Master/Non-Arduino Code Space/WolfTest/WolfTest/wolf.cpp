#include "wolf.h"


wolf::wolf()
{
	pinOut = wolfPins();
	debugger = wolfDebug(pinOut.getPin_DEBUG());
}


wolf::~wolf()
{
}


void wolf::taskManager()
{
	// do stuff...
}


wolfPins wolf::get_pinOut() { return pinOut; }
wolfDebug wolf::get_debugger() { return debugger; }