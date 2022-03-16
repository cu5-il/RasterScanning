#include "controlCalib.h"
#include <iostream>
#include <vector>
#include <iterator>
#include <string>
#include <fstream>

#include "myTypes.h"
#include "myGlobals.h"
#include "constants.h"
#include "raster.h"
#include "csvMat.h"
#include "thread_functions.h"
#include "draw.h"
#include <opencv2/imgcodecs.hpp>
#include "MaterialModel.h"


void makeCalibPath( std::vector<std::vector<Path>>& path, char test, double range[2]) {
	double inc = (range[1]-range[0]) / ceil(path.size() / 2.0);
	double setVal = range[0];
	int i = 1;

	switch (test)
	{
	default:
		std::cout << "ERROR: unknown test type" << std::endl;
		return;
	case 'f':
		std::cout << "Feed rate range (";
		break;
	case 'a':
		std::cout << "Auger range (";
		break;
	}
	std::cout << range[0] << ":" << inc << ":" << range[1] << ")" << std::endl;

	for (auto it1 = path.begin(); it1 != path.end(); ++it1, i++) {
		for (auto it2 = (*it1).begin(); it2 != (*it1).end(); ++it2) {
			switch (test)
			{
			default:
				break;
			case 'f':
				(*it2).f = setVal;
				break;
			case 'a':
				(*it2).e = setVal;
				break;
			}
		}
		if (i % 2) {
			setVal += inc;
		}
	}
}

void analyzePrint(Raster raster) {
	double targetWidth = 0;
	double sum, mean, stdev;
	int count;

	// Opening a file to write the results
	std::ofstream outfile;
	outfile.open(std::string(outDir + "widths.txt").c_str());
	outfile.precision(5);

	// calculating the average width for each segment
	for (int i = 0; i < segments.size(); i += 2) {
		sum = 0;
		count = 0;
		// calculate the mean
		for (auto it = segments[i].errWD().begin(); it != segments[i].errWD().end(); ++it) {
			if (!isnan(*it)) {
				sum += *it;
				count++;
			}
		}
		mean = sum / count;
		// calculate the standard deviation
		sum = 0;
		for (auto it = segments[i].errWD().begin(); it != segments[i].errWD().end(); ++it) {
			if (!isnan(*it)) {
				sum += pow((*it - mean), 2);
			}
		}
		stdev = sqrt((sum / count));
		outfile << std::fixed << mean << ",\t" << stdev << std::endl;
	}
	outfile.close();
	return;
}

bool readTestParams(std::string filename, Raster& raster, double& wayptSpc, cv::Point3d& initPos, double& initVel, double& initExt, char& test, double range[2])
{
	std::ifstream inFile(filename.c_str());
	std::string str;
	double value;
	int headerCnt = 0;
	double rasLen = NAN, rasWth = NAN, rodSpc = NAN, rodWidth = NAN;
	double X = NAN, Y = NAN, Z = NAN;
	char const* digits = "0123456789.-";
	char c;

	if (inFile.is_open()) {
		while (std::getline(inFile, str)) {

			// find the number in the string
			std::size_t const n = str.find_first_of(digits);
			if (n != std::string::npos){
				std::size_t const m = str.find_first_not_of(digits, n);
				 value = std::stod(str.substr(n, m != std::string::npos ? m - n : m));
			}
			// Extracting the raster parameters
			if (str.find("raster length") != std::string::npos) { rasLen = value; }
			else if (str.find("raster width") != std::string::npos) { rasWth = value; }
			else if (str.find("rod spacing") != std::string::npos) { rodSpc = value; }
			else if (str.find("rod width") != std::string::npos) { rodWidth = value; }
			else if (str.find("waypoint spacing") != std::string::npos) { wayptSpc = value; }
			// Extracting the test parameters
			else if (str.find("test type") != std::string::npos) { test = str[0]; }
			else if (str.find("feed rate") != std::string::npos) { initVel = value; }
			else if (str.find("auger voltage") != std::string::npos) { initExt = value; }
			else if (str.find("range start") != std::string::npos) { range[0] = value; }
			else if (str.find("range end") != std::string::npos) { range[1] = value; }
			// Extracting the initial coordinates
			else if (str.find("Xpos") != std::string::npos) { X = value; }
			else if (str.find("Ypos") != std::string::npos) { Y = value; }
			else if (str.find("Zpos") != std::string::npos) { Z = value; }
		}
		if (isnan(X) || isnan(Y) || isnan(Z) || isnan(rasLen) || isnan(rasWth) || isnan(rodSpc) || isnan(rodWidth)) { return false; }
		else {
			initPos = cv::Point3d(X, Y, Z);
			raster = Raster(rasLen, rasWth, rodSpc, rodWidth);
			raster.offset(cv::Point2d(initPos.x, initPos.y));
		}
	}
	else {
		std::cout << "Unable to open data file: " << filename << std::endl;
		system("pause");
		return false;
	}
	return true;
}

bool readTestParams(std::string filename, Raster& raster, double& wayptSpc, cv::Point3d& initPos, double& initVel, double& initExt, std::string& test, double range[2], int lineNum)
{
	std::ifstream inFile(filename.c_str());
	std::string str, temp;
	double value;
	int lineCnt = 0;
	double rasLen = NAN, rasWth = NAN, rodSpc = NAN, rodWidth = NAN;
	char const* digits = "0123456789.-";
	char c;

	// read the initial positions and the test
	if (inFile.is_open()) {
		while (std::getline(inFile, str)) {
			std::stringstream ss(str);
			if (lineCnt == 0) {
				ss >> temp;
				initPos.z = std::stod(temp);
			}
			else if (lineCnt == lineNum) {
				ss >> temp;
				initPos.x = std::stod(temp);
				ss >> temp;
				initPos.y = std::stod(temp);
				ss >> temp;
				break;
			}
			lineCnt++;
		}
		inFile.close();
	}
	else {
		std::cout << "Unable to open data file: " << filename << std::endl;
		system("pause");
		return false;
	}
	std::cout << "Test selected: " << temp << " @ " << initPos << std::endl;
	//test = temp;

	outDir.append(temp + "/");
	if (CreateDirectoryA(outDir.c_str(), NULL) || ERROR_ALREADY_EXISTS == GetLastError()) {}
	else {
		std::cout << "Error creating output directory" << std::endl;
		return false;
	}

	// read the test parameters
	filename = filename.substr(0, filename.rfind("/") + 1) + temp + ".txt";
	inFile = std::ifstream(filename.c_str());
	if (inFile.is_open()) {
		while (std::getline(inFile, str)) {

			// find the number in the string
			std::size_t const n = str.find_first_of(digits);
			if (n != std::string::npos) {
				std::size_t const m = str.find_first_not_of(digits, n);
				value = std::stod(str.substr(n, m != std::string::npos ? m - n : m));
			}
			// Extracting the raster parameters
			if (str.find("raster length") != std::string::npos) { rasLen = value; }
			else if (str.find("raster width") != std::string::npos) { rasWth = value; }
			else if (str.find("rod spacing") != std::string::npos) { rodSpc = value; }
			else if (str.find("rod width") != std::string::npos) { rodWidth = value; }
			else if (str.find("waypoint spacing") != std::string::npos) { wayptSpc = value; }
			// Extracting the test parameters
			else if (str.find("test type") != std::string::npos) { 
				test = str.substr( 0, str.find_first_of('\t')); }
			else if (str.find("feed rate") != std::string::npos) { initVel = value; }
			else if (str.find("auger voltage") != std::string::npos) { initExt = value; }
			else if (str.find("range start") != std::string::npos) { range[0] = value; }
			else if (str.find("range end") != std::string::npos) { range[1] = value; }

		}
		if ( isnan(rasLen) || isnan(rasWth) || isnan(rodSpc) || isnan(rodWidth)) { return false; }
		else {
			raster = Raster(rasLen, rasWth, rodSpc, rodWidth);
			raster.offset(cv::Point2d(initPos.x, initPos.y));
		}
	}
	else {
		std::cout << "Unable to open data file: " << filename << std::endl;
		system("pause");
		return false;
	}
	return true;
}

void makeFGS(std::vector<std::vector<Path>>& path, char param, char type, double range[2], MaterialModel model) {
	int numPts;
	double delta;
	double width = range[0];

	if (/*param != 'f' &&*/ param != 'a') {
		std::cout << "ERROR: unknown parameter type" << std::endl;
		return;
	}
	if (type != 'b' && type != 'g' && type != 'c') {
		std::cout << "ERROR: unknown scaffold type" << std::endl;
		return;
	}

	switch (type)
	{
	default:
		break;
	case 'b': // bowtie scaffold
		numPts = path[0].size() - 1;
		delta = (range[1] - range[0]) / (numPts / 2.0);

		for (auto it_seg = path.begin(); it_seg != path.end(); ++it_seg) {
			for (auto it_rod = (*it_seg).begin(); it_rod != (*it_seg).end(); ++it_rod) {

				// Modify the width
				(*it_rod).w = width;
				// if long rods
				if (std::distance(path.begin(), it_seg) % 2 == 0) {
					// decrease for first half of rod then increase for second half
					if (std::distance((*it_seg).begin(), it_rod) < (numPts / 2.0)) {
						width += delta;
					}
					else {
						width -= delta;
					}
				}
				else {
					width = range[0];
				}
			}
		}
		break;
	case 'g': // gradient scaffold
		numPts = path[0].size() - 1;
		delta = (range[1] - range[0]) / numPts;

		for (auto it_seg = path.begin(); it_seg != path.end(); ++it_seg) {
			for (auto it_rod = (*it_seg).begin(); it_rod != (*it_seg).end(); ++it_rod) {

				// Modify the width
				(*it_rod).w = width;
				switch (std::distance(path.begin(), it_seg) % 4)
				{
				case 0: // positive x direction rods
					width += delta;
					break;
				case 1: // y direction rods at max x
					width = range[1];
					break;
				case 2: // negative x direction rods
					width -= delta;
					break;
				case 3: // y direction rods at min x
					width = range[0];
					break;
				}
			}
		}
		break;
	case 'c': // continuous gradient scaffold
		numPts = ceil((path[0].size()-1) * ceil(path.size() / 2.0));
		delta = (range[1] - range[0]) / numPts;

		for (auto it_seg = path.begin(); it_seg != path.end(); ++it_seg) {
			for (auto it_rod = (*it_seg).begin(); it_rod != (*it_seg).end(); ++it_rod) {

				// Modify the width
				(*it_rod).w = width;
				// if long rods
				if (std::distance(path.begin(), it_seg) % 2 == 0) {
					width += delta;
				}
			}
		}
		break;
	}

	// modify the inputs
	for (auto it_seg = path.begin(); it_seg != path.end(); ++it_seg) {
		for (auto it_rod = (*it_seg).begin(); it_rod != (*it_seg).end(); ++it_rod) {

			// Modify the width
			switch (param)
			{
			case 'f':
				(*it_rod).f = model.output((*it_rod).w, (*it_rod).e);
				break;
			case 'a':
				(*it_rod).e = model.output((*it_rod).w, (*it_rod).f);
				break;
			}

		}
	}

}