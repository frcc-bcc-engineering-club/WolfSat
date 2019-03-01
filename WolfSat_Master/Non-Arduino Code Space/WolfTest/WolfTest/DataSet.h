#pragma once
#ifndef DATASET_H
#define DATASET_H

#define Naught 0x00FACADE


template<class type> class DataSet
{
public:
	DataSet();
	DataSet(int in_size);
	~DataSet();
	type* get_data(int in_pos);
	int get_pos();
	int get_size();
	void set_data(type in_data);
private:
	type * data;
	int size;
	int pos;
};


template<class type> DataSet<type>::DataSet()
{
	size = 0;
	pos = 0;
	data = new type[size];
}


template<class type> DataSet<type>::DataSet(int in_size)
{
	size = in_size;
	pos = 0;
	data = type[size];
}


template<class type> DataSet<type>::~DataSet()
{

}


template<class type> type* DataSet<type>::get_data(int in_pos)
{
	return data[in_pos];
}


template<class type> int DataSet<type>::get_pos()
{
	return pos;
}


template<class type> int DataSet<type>::get_size()
{
	return size;
}


template<class type> void DataSet<type>::set_data(type in_data)
{
	if ((pos < size) && (pos >= 0))
		data[pos] = in_data;
}


#endif // !DATASET_H
