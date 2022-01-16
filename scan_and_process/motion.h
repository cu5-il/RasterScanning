#pragma once
#include <vector>
#include <deque>
#include <opencv2/core.hpp>

#ifndef MOTION_H
#define MOTION_H

void printPath(std::deque<std::vector<double>>& path, cv::Point2d initPos, double speed, double augerSpeed);

#endif // MOTION_H



