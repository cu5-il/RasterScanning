#include "MaterialModels.h"
#include "myTypes.h"
#include "myGlobals.h"
#include "constants.h"

#include <iostream>
#include <cmath>

double augerModel(double input, double feedrate, bool invert ) {
	double a, b, c;

	a = 1;
	b = 0.1;
	c = -2.815;

	if (feedrate == 2.0) {
		a = 4.15; 
	}
	else if (feedrate == 3.0) {
		a = 3.95;
	}

	if (invert) { return a * pow(input, b) + c; } // return width
	else { return pow(((input - c) / a), 1 / b); } // return voltage

}