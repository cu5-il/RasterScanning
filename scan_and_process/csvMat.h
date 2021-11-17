#pragma once
#include <string>
#include <opencv2/core.hpp>

void writeCSV(std::string filename, cv::Mat m);

void readCSV(std::string filename, cv::Mat& m);