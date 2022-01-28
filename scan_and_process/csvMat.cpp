#include <iostream>
#include <fstream>
#include <string>
#include <opencv2/core.hpp>

void writeCSV(std::string filename, cv::Mat m)
{
	std::ofstream myfile;
	myfile.open(filename.c_str());
	myfile << cv::format(m, cv::Formatter::FMT_CSV);
	myfile.close();
}


void readCSV(std::string filename, cv::Mat& m)
{
	std::ifstream inFile(filename.c_str());
	std::string single_line;
	std::vector< std::vector<double> > matrix;

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
			matrix.push_back(vec);
		}
	}
	else {
		std::cout << "Unable to open data file: " << filename << std::endl;
		system("pause");
		return;
	}
	m = cv::Mat((int)matrix.size(), (int)matrix[0].size(), CV_64FC1/*, matrix.data()*/);
	m.at<double>(0, 0) = matrix.at(0).at(0);
	for (int i = 0; i < m.rows; ++i)
		for (int j = 0; j < m.cols; ++j)
			m.at<double>(i, j) = matrix.at(i).at(j);
	m = m.clone();
	return;
}