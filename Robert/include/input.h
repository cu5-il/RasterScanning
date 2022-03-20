#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <opencv2/core.hpp>

#ifndef INPUT_H
#define INPUT_H

class TableInput
{
public:
	TableInput();
	TableInput(std::string filename, int printNumber);

	int printNum, layers;
	double length, width, height, rodSpc, wayptSpc, F, E;
	double range[2];
	char type, method;
	cv::Point3d initPos;

private:
	void _readTable(std::string filename);

};

TableInput::TableInput()
	: printNum(0), layers(0), length(0), width(0), height(0), rodSpc(0), wayptSpc(0), F(0), E(0), range{ 0, 0 }, type(0), method(0) {}

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
						else if (str.find("length") != std::string::npos) { length = std::stod(value); }
						else if (str.find("width") != std::string::npos) { width = std::stod(value); }
						else if (str.find("layers") != std::string::npos) { layers = std::stoi(value); }
						else if (str.find("height") != std::string::npos) { height = std::stod(value); }
						else if (str.find("rodspc") != std::string::npos) { rodSpc = std::stod(value); }
						else if (str.find("wptspc") != std::string::npos) { wayptSpc = std::stod(value); }
						// Functionally generated scaffold parameters
						else if (str.find("type") != std::string::npos) { type = value[0]; }
						else if (str.find("method") != std::string::npos) { method = value[0]; }					
						else if (str.find("W1") != std::string::npos) { range[0] = std::stod(value); }
						else if (str.find("W2") != std::string::npos) { range[1] = std::stod(value); }
						else if (str.find("F") != std::string::npos) { F = std::stod(value); }
						else if (str.find("E") != std::string::npos) { E = std::stod(value); }
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