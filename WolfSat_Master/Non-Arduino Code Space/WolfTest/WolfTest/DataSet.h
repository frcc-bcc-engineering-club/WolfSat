#pragma once
#ifndef DATASET_H
#define DATASET_H

#define Naught 0x00FACADE

class DataSet
{
public:
	DataSet();
	DataSet(bool typeIsInt);
	~DataSet();
	virtual void get_data(int& out_int, double& out_dub, const int in_pos) = 0;
	int get_dataPos();
	int get_dataSize();
	virtual void set_data(const int in_int, const double in_dub) = 0;
	//virtual void set_dataSet(const int* in_intSet, const double* in_dubSet) = 0;
	void set_dataSize(const int in_size);
	void incr_dataPos();
private:
	virtual void setup_array() = 0;
	bool checkType();
	int dataSize;
	int dataPos;
	bool isIntType;
};

#endif // !DATASET_H
