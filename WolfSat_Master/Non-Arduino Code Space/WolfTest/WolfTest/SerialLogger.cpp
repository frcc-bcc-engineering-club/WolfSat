#include "SerialLogger.h"



SerialLogger::SerialLogger() : LoggerWrap(true)
{

}


SerialLogger::~SerialLogger()
{

}


void SerialLogger::sendToLog(DataSet& in_set)
{
	int lim = in_set.get_dataSize();
	int pos = 0;
	while (pos < lim)
	{
		int smallDat = 0;
		double bigDat = 0.0;
		//String toSend = "";
		in_set.get_data(smallDat, bigDat, pos);
		if (smallDat == Naught)
		{
			// toSend = (String)bigDat;
			//logger.println(toSend);
		}
		else
		{
			// toSend = (String)smallDat;
			// logger.println(toSend);
		}
		pos++;
	}
}


void SerialLogger::setup_Log()
{
	// Wire.begin();
	// logger.begin();
}