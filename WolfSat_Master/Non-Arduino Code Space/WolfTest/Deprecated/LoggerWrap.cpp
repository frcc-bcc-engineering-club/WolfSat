#include "LoggerWrap.h"



LoggerWrap::LoggerWrap()
{

}


LoggerWrap::LoggerWrap(bool in_is)
{
	set_isSerial(in_is);
}


LoggerWrap::~LoggerWrap()
{

}


void LoggerWrap::set_isSerial(bool in_is)
{
	isSerial = in_is;
}