#include "BigSet.h"



BigSet::BigSet() : DataSet(false)
{
	set_dataSize(0);
	setup_array();
}


BigSet::BigSet(int in_size) : DataSet(false)
{
	set_dataSize(in_size);
	setup_array();
}


BigSet::~BigSet()
{
}

void BigSet::get_data(int& out_int, double& out_dub, const int in_pos)
{
	out_int = Naught;
	out_dub = dubSet[in_pos];
}

void BigSet::set_data(const int in_int, const double in_dub)
{
	dubSet[this->get_dataPos()];
	incr_dataPos();
}

void BigSet::setup_array()
{
	dubSet = new double[this->get_dataSize()];
}
