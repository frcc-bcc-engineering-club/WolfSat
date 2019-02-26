#pragma once
#ifndef WOLF_H
#define WOLF_H

#include "wolfPins.h"
#include "wolfDebug.h"

// Visual Studio Copy. Some functions commented out...


class wolf
{
public:
	wolf();
	void taskManager();
	wolfPins get_pinOut();
	wolfDebug get_debugger();
	~wolf();
private:
	wolfPins pinOut;
	wolfDebug debugger;
};
#endif // !WOLF_H


