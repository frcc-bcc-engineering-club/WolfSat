#pragma once
#ifndef WOLFPINS_H
#define WOLFPINS_H

// Arduino version

class WolfPins
{
public:
	WolfPins();
	~WolfPins();
	int getPin_LED();
	int getPin_DEBUG();
private:
	void setup_Pins();
	int PIN_LED;
	int PIN_DEBUG;
};



#endif // !WOLFPINS_H

