#include "SerialLogger.h"



SerialLogger::SerialLogger() : LoggerWrap(true)
{
	number = 0;
	setup_Log();
}


SerialLogger::SerialLogger(int in_number) : LoggerWrap(true)
{
	if ((number < 4) && (number > 0))
		number = in_number;
	else
		number = 0;
	setup_Log();
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
			// send(toSend);
		}
		else
		{
			// toSend = (String)smallDat;
			// send(toSend);
		}
		pos++;
	}
}


//void SerialLogger::stringToLog(String toSend)
//{
// send(toSend);
//}


void SerialLogger::setup_Log()
{
	// Wire.begin();
	// logger.begin();
}


//void SerialLogger::send(String toSend)
//{
//	switch (number)
//	{
//	case 0:
//		Serial.println(toSend);
//		break;
//#if defined (__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//	case 1:
//		Serial1.println(toSend);
//		break;
//	case 2:
//		Serial2.println(toSend);
//		break;
//	case 3:
//		Serial3.println(toSend);
//		break;
//#endif // !(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
//	default:
//		Serial.println(toSend);
//		break;
//	}
//}


SerialLogger& SerialLogger::operator=(const SerialLogger& in_logger)
{
	return *this;
}