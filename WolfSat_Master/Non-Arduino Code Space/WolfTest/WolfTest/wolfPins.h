#pragma once
#ifndef WOLFPINS_H
#define WOLFPINS_H

// Visual Studio Copy, Some functions commented out.

class wolfPins
{
public:
	wolfPins();
	~wolfPins();
	int getPin_LED();
	int getPin_DEBUG();
private:
	//void setup_Pins();
	int PIN_LED;
	int PIN_DEBUG;
};



#endif // !WOLFPINS_H

