#include "Wolf.h"


Wolf::Wolf()
{
	pinOut = WolfPins();
	debugger = WolfDebug(pinOut.getPin_DEBUG());
	logOne = SerialLogger();
}


Wolf::~Wolf()
{
}


void Wolf::taskManager()
{
	// do stuff...
}


WolfPins Wolf::get_pinOut() const { return pinOut; }
WolfDebug Wolf::get_debugger() const { return debugger; }
SerialLogger Wolf::get_logOne() const { return logOne; }


Wolf& Wolf::operator=(const Wolf& in_wolf)
{
	pinOut = in_wolf.get_pinOut();
	debugger = in_wolf.get_debugger();
	logOne = in_wolf.get_logOne();
	return *this;
}