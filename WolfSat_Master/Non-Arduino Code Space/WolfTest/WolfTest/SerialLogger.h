#pragma once
#ifndef SERIALLOGGER_H
#define SERIALLOGGER_H

#include "LoggerWrap.h"
class SerialLogger : public LoggerWrap
{
public:
	SerialLogger();
	~SerialLogger();
	virtual void sendToLog(DataSet& in_set);
private:
	virtual void setup_Log();
	// OpenLog logger;
};

#endif // !SERIALLOGGER_H
