#pragma once
#ifndef SENSOR_H
#define SENSOR_H

#include "DataSet.h"

template <class innerType>
class Sensor
{
public:
	Sensor();
	Sensor( int in_lim);
	~Sensor();
	virtual void run() = 0;
	DataSet<class type> & get_data();
private:
	DataSet<class type> sensorData;
	int limit;
};

#endif // !SENSOR_H

