#include <iostream>
#include <vector>
#include <iterator> 

#include "myTypes.h"
#include "myGlobals.h"
#include "constants.h"


void makeTestPath( std::vector<std::vector<Path>>& path, int param, double range[2]) {
	double inc = (range[1]-range[0]) / ceil(path.size() / 2);
	double setVal = range[0];
	int i = 1;

	for (auto it1 = path.begin(); it1 != path.end(); ++it1, i++) {
		for (auto it2 = (*it1).begin(); it2 != (*it1).end(); ++it2) {
			switch (param)
			{
			default:
				break;
			case 0:
				(*it2).f = setVal;
				break;
			case 1:
				(*it2).e = setVal;
				break;
			}
		}
		if (i % 2) {
			setVal += inc;
		}
	}
}