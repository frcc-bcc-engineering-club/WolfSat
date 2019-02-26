#include "wolfPins.h"



wolfPins::wolfPins()
{
	
	PIN_DEBUG = 12;
	PIN_LED = 13;
	//setup_Pins();
}


wolfPins::~wolfPins()
{
}


//void wolfPins::setup_Pins()
//{
//
//	pinMode(PIN_LED(), OUTPUT);
//	pinMode(PIN_DEBUG(), INPUT);
//	
//}


int wolfPins::getPin_DEBUG() { return PIN_DEBUG; }
int wolfPins::getPin_LED() { return PIN_LED; }