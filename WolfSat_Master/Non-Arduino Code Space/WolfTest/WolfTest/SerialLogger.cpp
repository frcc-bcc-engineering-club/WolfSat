#include "SerialLogger.h"
#include "DataSet.h"

//template
SerialLogger::SerialLogger() : LoggerWrap(true)
{
	number = 0;
	setup_Log();
}

//template
SerialLogger::SerialLogger(int in_number) : LoggerWrap(true)
{
	if ((number < 4) && (number > 0))
		number = in_number;
	else
		number = 0;
	setup_Log();
}

//template
SerialLogger::~SerialLogger()
{

}

//template
void SerialLogger::sendToLog(DataSet<class type> in_set)
{
	int lim = in_set.get_size();
	int pos = 0;
	while (pos < lim)
	{
		//DataSet<class type> local = in_set.get_data(pos);
		//String toSend = "";
		//send((string)in_set.get_data(pos));
		pos++;
	}
}

//template
//void SerialLogger::stringToLog(String toSend)
//{
// send(toSend);
//}

//template
void SerialLogger::setup_Log()
{
	// Wire.begin();
	// logger.begin();
}

//template
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

//template
SerialLogger& SerialLogger::operator=(const SerialLogger& in_logger)
{
	return *this;
}