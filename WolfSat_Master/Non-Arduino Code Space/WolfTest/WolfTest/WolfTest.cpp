// WolfTest.cpp : Defines the entry point for the console application.
//

#include "wolf.h"

wolf setup();

int main()
{
	wolf testWolf = setup();
	while (true)
	{
		testWolf.taskManager();
	}
	int x;

    return 0;
}

wolf setup()
{
	wolf loc = loc.init();
	return loc;
}