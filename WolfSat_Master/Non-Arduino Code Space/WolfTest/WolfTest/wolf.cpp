#include "wolf.h"


wolf::wolf()
{
	pinOut = wolfPins();
}


wolf::~wolf()
{
}


void wolf::taskManager()
{
	// do stuff...
}


wolfPins wolf::get_pinOut() { return pinOut; }