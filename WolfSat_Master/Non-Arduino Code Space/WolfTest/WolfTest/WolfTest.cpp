// WolfTest.cpp : Defines the entry point for the console application.
//

#include "Wolf.h"
#include <iostream>

using namespace std;



int main()
{
	DataSet<int> myData;
	Wolf testWolf = Wolf();
	while (true)
	{
		Wolf testWolf = Wolf();
		WolfPins pins = testWolf.get_pinOut();
		cout << "Mark" << endl;

		//testWolf.taskManager();
	}

    return 0;
}
