// WolfTest.cpp : Defines the entry point for the console application.
//

#include "Wolf.h"


Wolf setup();

int main()
{
	Wolf testWolf = setup();
	while (true)
	{
		testWolf.taskManager();
	}

    return 0;
}

Wolf setup()
{
	Wolf loc = Wolf();
	return loc;
}