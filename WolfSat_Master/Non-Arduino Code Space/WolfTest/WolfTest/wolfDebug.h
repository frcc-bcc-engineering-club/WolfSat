#pragma once
#ifndef WOLFDEBUG_H
#define WOLFDEBUG_H

#include "WolfPins.h"

// Arduino version

class WolfDebug
{
public:
	WolfDebug();
	WolfDebug(int debugPin);
	~WolfDebug();
	bool get_debugMode();
private:
	void setup_Debug(int debugPin);
	bool debugMode;
};

#endif // !WolfDebug_H