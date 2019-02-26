#pragma once
#ifndef WOLFDEBUG_H
#define WOLFDEBUG_H

#include "wolfPins.h"

// Visual Studio version, some functions commented out...

class wolfDebug
{
public:
	wolfDebug();
	wolfDebug(int debugPin);
	~wolfDebug();
	bool get_debugMode();
private:
	//void setup_Debug(int debugPin);
	bool debugMode;
};

#endif // !wolfDebug_H