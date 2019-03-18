#include "Sensor.h"

template <class innerType>
Sensor<innerType>::Sensor()
{
	limit = 0;
	sensorData = DataSet<innerType>(limit);
}


template <class innerType>
Sensor<innerType>::Sensor(int in_lim)
{
	limit = in_lim;
	sensorData = DataSet<innerType>(limit);
}


template <class innerType>
Sensor<innerType>::~Sensor()
{

}


template <class innerType>
DataSet<typename type>& Sensor<innerType>::get_data()
{
	return DataSet;
}
