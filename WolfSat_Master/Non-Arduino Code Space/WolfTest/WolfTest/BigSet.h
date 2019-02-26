#pragma once
#ifndef BIGSET_H
#define BIGSET_H

#include "DataSet.h"

class BigSet : public DataSet
{
public:
	BigSet();
	BigSet(int in_size);
	~BigSet();
	virtual void get_data(int& out_int, double& out_dub, const int in_pos);
	virtual void set_data(const int in_int, const double in_dub);
	//virtual void set_dataSet(const int* in_intSet, const double* in_dubSet);
private:
	virtual void setup_array();
	double * dubSet;
};

#endif // !BIGSET_H
