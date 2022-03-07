#pragma once
#include "myTypes.h"

#ifndef TESTCONTROLLER_H
#define TESTCONTROLLER_H

double augerModel(double width, double feedrate, bool invert = false);

void testController(Path prevPth, double errWd, Path& nextPth, double kp = 1);

void t_controllerTEST(std::vector<std::vector<Path>> pathSegs, int segsBeforeCtrl);

void readErrors(std::string filename, std::vector<double>& errWd);

#endif // !TESTCONTROLLER_H

