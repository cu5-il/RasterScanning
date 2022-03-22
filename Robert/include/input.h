#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>
#include "myTypes.h"

#ifndef INPUT_H
#define INPUT_H

class TableInput
{
public:
	TableInput();
	TableInput(std::string filename, int printNumber);

	int printNum, layers, startLayer;
	double length, width, height, rodSpc, wayptSpc, F, E;
	double range[2];
	char type, method;
	cv::Point3d initPos;

private:
	void _readTable(std::string filename);

};

TableInput::TableInput()
	: printNum(0), layers(0), length(0), width(0), height(0), rodSpc(0), wayptSpc(0), startLayer(0), F(0), E(0), range{ 0, 0 }, type(0), method(0) {}

inline TableInput::TableInput(std::string filename, int printNumber)
{
	printNum = printNumber;
	_readTable(filename);
}

inline void TableInput::_readTable(std::string filename)
{
	std::ifstream inFile(filename.c_str());
	std::string line, value, str;
	std::vector<std::string> colNames;
	int lineNum = 0;
	int col = 0;

	if (inFile.is_open()) {
		while (std::getline(inFile, line)) 
		{
			line.erase(remove_if(line.begin(), line.end(), isspace), line.end());
			std::stringstream ss(line);
			if (lineNum == 0)
			{ // read the table headings
				while (std::getline(ss, value, '|'))
				{
					colNames.push_back(value);
				}
			}
			else if (lineNum > 1)
			{ // read the table data
				while (std::getline(ss, value, '|'))
				{
					str = colNames[col];
					if (!value.empty()) {
						if (((str.compare("Test#") == 0)) && printNum != std::stoi(value)) { break; }
						// Position values
						else if (str.compare("X") == 0) { initPos.x = std::stod(value); }
						else if (str.compare("Y") == 0) { initPos.y = std::stod(value); }
						else if (str.compare("Z") == 0) { initPos.z = std::stod(value); }
						// raster parameters
						else if (str.compare("length") == 0) { length = std::stod(value); }
						else if (str.compare("width") == 0) { width = std::stod(value); }
						else if (str.compare("layers") == 0) { layers = std::stoi(value); }
						else if (str.compare("height") == 0) { height = std::stod(value); }
						else if (str.compare("rodspc") == 0) { rodSpc = std::stod(value); }
						else if (str.compare("wptspc") == 0) { wayptSpc = std::stod(value); }
						else if (str.compare("layer0") == 0) { startLayer = std::stoi(value); }
							
						// Functionally generated scaffold parameters
						else if (str.compare("type") == 0) { type = value[0]; }
						else if (str.compare("method") == 0) { method = value[0]; }
						else if (str.compare("W1") == 0) { range[0] = std::stod(value); }
						else if (str.compare("W2") == 0) { range[1] = std::stod(value); }
						else if (str.compare("F") == 0) { F = std::stod(value); }
						else if (str.compare("E") == 0) { E = std::stod(value); }
					}
					col++;
				}
				col = 0;
			}
			lineNum++;
		}
		inFile.close();
	}
	else {
		std::cout << "Unable to open: " << filename << std::endl;
		system("pause");
		return;
	}


}

#endif // !INPUT_H