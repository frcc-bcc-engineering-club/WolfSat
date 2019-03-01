#pragma once
#ifndef WOLF_H
#define WOLF_H

#include "DataSet.h"
#include "WolfPins.h"
#include "WolfDebug.h"
#include "SerialLogger.h"
#include "Sensor.h"

// Visual Studio Copy. Some functions commented out...


class Wolf
{
public:
	void taskManager();
	WolfPins get_pinOut() const;
	WolfDebug get_debugger() const;
	//SerialLogger get_logOne() const;

	Wolf& operator=(const Wolf& in_wolf);

	Wolf();
	~Wolf();
private:
	DataSet<double> myData;
	WolfPins pinOut;
	WolfDebug debugger;
	//SerialLogger logOne;
};
#endif // !Wolf_H


