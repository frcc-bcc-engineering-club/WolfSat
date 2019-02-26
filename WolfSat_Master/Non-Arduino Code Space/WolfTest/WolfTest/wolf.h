#pragma once
#ifndef WOLF_H
#define WOLF_H

#include "WolfPins.h"
#include "WolfDebug.h"

// Visual Studio Copy. Some functions commented out...


class Wolf
{
public:
	Wolf();
	void taskManager();
	WolfPins get_pinOut();
	WolfDebug get_debugger();
	~Wolf();
private:
	WolfPins pinOut;
	WolfDebug debugger;
};
#endif // !Wolf_H


