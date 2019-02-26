#pragma once
#ifndef LOGGERWRAP_H
#define LOGGERWRAP_H

#include "DataSet.h"

class LoggerWrap
{
public:
	LoggerWrap();
	LoggerWrap(bool in_is);
	~LoggerWrap();
	virtual void sendToLog(DataSet& in_set) = 0;
protected:
	void set_isSerial(bool in_is);
private:
	virtual void setup_Log() = 0;
	bool isSerial;
};

#endif // !LOGGERWRAP_H
