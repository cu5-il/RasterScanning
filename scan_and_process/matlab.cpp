#include <fstream>
#include <string>
#include <vector>
#include <deque>
#include "raster.h"

#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"

void export2matlab(std::string filename, Raster& raster) {
    int i = 0;
    std::ofstream myfile;
    myfile.open(filename.c_str());
    if (myfile.is_open()) {
        myfile << "V = " << VELOCITY << ";" << std::endl;
        myfile << "resolution = " << RESOLUTION << ";" << std::endl;
        myfile << "dt = " << RESOLUTION / VELOCITY << ";" << std::endl;
        myfile << "R = " << SCAN_OFFSET << ";" << std::endl;
        myfile << "w = " << SCAN_WIDTH << ";" << std::endl;
        myfile << "ras = [";
        for (auto it = raster.coordsMM().begin(); it != raster.coordsMM().end(); ++it) {
            myfile << i <<", " << (*it).x << ", " << (*it).y << std::endl;
            i++;
        }
        myfile << "];" << std::endl;

        myfile << "corners = [";
        for (auto it = raster.cornersMM().begin(); it != raster.cornersMM().end(); ++it) {
            myfile << (*it).x << ", " << (*it).y << std::endl;
        }
        myfile << "]';" << std::endl;
        myfile.close();
    }
    else std::cout << "Unable to open file";
}

void readPath(std::string filename, std::deque<std::vector<double>>& path)
{
	std::ifstream inFile(filename.c_str());
	std::string single_line;
	double value;

	if (inFile.is_open()) {
		while (std::getline(inFile, single_line)) {
			std::vector<double> vec;
			std::stringstream temp(single_line);
			std::string single_value;

			while (std::getline(temp, single_value, ',')) {
				value = std::stod(single_value);
				vec.push_back(value);
			}
			path.push_back(vec);
		}
	}
	else {
		std::cout << "Unable to open data file: " << filename << std::endl;;
		system("pause");
		return;
	}

	return;
}