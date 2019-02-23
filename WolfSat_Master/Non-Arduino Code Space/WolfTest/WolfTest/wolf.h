#pragma once
#ifndef WOLF_H
#define WOLF_H

#include "wolfPins.h"

// Visual Studio Copy. Some functions commented out...


class wolf
{
public:
	wolf();
	void taskManager();
	wolfPins get_pinOut();
	~wolf();
private:
	wolfPins pinOut;
};
#endif // !WOLF_H


