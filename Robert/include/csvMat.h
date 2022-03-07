#pragma once
#include <string>
#include <opencv2/core.hpp>

#ifndef CSV_MAT_H
#define CSV_MAT_H

void writeCSV(std::string filename, cv::Mat m);

void readCSV(std::string filename, cv::Mat& m);

#endif // CSV_MAT_H