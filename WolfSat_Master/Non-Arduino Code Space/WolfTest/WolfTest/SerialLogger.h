#pragma once
#ifndef SERIALLOGGER_H
#define SERIALLOGGER_H

#ifdef DATASET_H


#endif // DATASET_H



#include "LoggerWrap.h"
#include "DataSet.h"

class SerialLogger : public LoggerWrap
{
public:
	SerialLogger();
	SerialLogger(int in_number);
	~SerialLogger();
	//OpenLog& get_logger() const;
	virtual void sendToLog(DataSet<class type> in_set);
	//void stringToLog(String in_string);
	SerialLogger& operator=(const SerialLogger& in_logger);
private:
	virtual void setup_Log();
	//void send(String in_string);
	// OpenLog logger;
	int number;
};

//template<class innerType>


#endif // !SERIALLOGGER_H
