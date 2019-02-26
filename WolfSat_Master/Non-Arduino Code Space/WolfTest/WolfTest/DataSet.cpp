#include "DataSet.h"

DataSet::DataSet()
{
	isIntType = true;
	dataSize = 0;
	dataPos = 0;
}


DataSet::DataSet(bool typeIsInt)
{
	isIntType = true;
	dataSize = 0;
	dataPos = 0;
}


DataSet::~DataSet()
{

}


int DataSet::get_dataPos()
{
	return dataPos;
}


int DataSet::get_dataSize()
{
	return dataSize;
}


void DataSet::set_dataSize(const int in_size)
{
	dataSize = in_size;
}


void DataSet::incr_dataPos()
{
	dataPos++;
}


bool DataSet::checkType()
{
	return isIntType;
}