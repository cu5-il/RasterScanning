#include "MaterialModels.h"
#include "myTypes.h"
#include "myGlobals.h"
#include "constants.h"

#include <iostream>
#include <cmath>

double augerModel(double width, double feedrate) {
	double voltage;
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

	voltage = pow(((width - c) / a), 1 / b);
	return voltage;
}