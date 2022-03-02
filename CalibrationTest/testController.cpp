#include "testController.h"
#include <iostream>
#include <vector>
#include <iterator>
#include <fstream>

#include "myTypes.h"
#include "myGlobals.h"
#include "constants.h"
#include "MaterialModels.h"


void testController(Path prevPth, double errWd, Path& nextPth, double kp)
{
	double setWidth = augerModel(prevPth.e, prevPth.f);
	double setpoint;
	if (!isnan(errWd)) {
		setpoint = augerModel(setWidth + errWd, nextPth.f);
		// clamp output
		if (fabs(setpoint) > 4) { setpoint = copysign(4.0, setpoint); } // Saturate output at +/-4
		nextPth.e = setpoint;
	}
}

void t_controllerTEST(std::vector<std::vector<Path>> path, int segsBeforeCtrl) 
{
	errsMsg inMsg;
	pathMsg outMsg;
	int segNum = 0;

	while (segNum < path.size()) {
		// do not modify the initial segment inputs
		if (segNum >= segsBeforeCtrl) {
			q_errsMsg.wait_and_pop(inMsg);
			// Use the errors from the previous segment to calculate the control next segment 
			segNum = inMsg.segmentNum() + 3;
			// If no errors were calculated, do not modify the path
			if (!inMsg.errCL().empty() && !inMsg.errWD().empty()) {
				// Modify path input
				for (int i = 0; i < inMsg.errWD().size(); i++) {
					testController(path[segNum/*inMsg.segmentNum()*/][i], inMsg.errWD()[i], path[segNum][i]);
				}
			}
		}
		// Send path coords to queue
		outMsg.addPath(path[segNum], segNum);
		q_pathMsg.push(outMsg);
		segNum++;
	}
	std::cout << "Ending controller thread" << std::endl;
}

void readErrors(std::string filename, std::vector<double>& errWd)
{
	std::ifstream inFile(filename.c_str());
	std::string single_line;
	double value;
	int col = 0;

	if (inFile.is_open()) {
		while (std::getline(inFile, single_line)) {
			std::stringstream temp(single_line);
			std::string single_value;

			while (std::getline(temp, single_value, '\t') && col < 5) {
				col++;
			}
			value = std::stod(single_value);
			errWd.push_back(value);
			col = 0;
		}
	}
	else {
		std::cout << "Unable to open data file: " << filename << std::endl;
		system("pause");
		return;
	}

	return;

}