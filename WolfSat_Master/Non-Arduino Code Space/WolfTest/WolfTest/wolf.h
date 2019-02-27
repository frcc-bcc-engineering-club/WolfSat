#pragma once
#ifndef WOLF_H
#define WOLF_H

#include "WolfPins.h"
#include "WolfDebug.h"
#include "SerialLogger.h"

// Visual Studio Copy. Some functions commented out...


class Wolf
{
public:
	void taskManager();
	WolfPins get_pinOut() const;
	WolfDebug get_debugger() const;
	SerialLogger get_logOne() const;

	Wolf& operator=(const Wolf& in_wolf);

	Wolf();
	~Wolf();
private:
	WolfPins pinOut;
	WolfDebug debugger;
	SerialLogger logOne;
};
#endif // !Wolf_H


