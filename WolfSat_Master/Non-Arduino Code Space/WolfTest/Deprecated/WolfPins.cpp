#include "WolfPins.h"



WolfPins::WolfPins()
{
	
	PIN_DEBUG = 12;
	PIN_LED = 13;
	setup_Pins();
}


WolfPins::~WolfPins()
{
}


void WolfPins::setup_Pins()
{

	//pinMode(getPin_LED(), OUTPUT);
	//pinMode(getPin_DEBUG(), INPUT);
	
}


int WolfPins::getPin_DEBUG() { return PIN_DEBUG; }
int WolfPins::getPin_LED() { return PIN_LED; }