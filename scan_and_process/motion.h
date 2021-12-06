#pragma once
#include <vector>
#include <opencv2/core.hpp>

#ifndef MOTION_H
#define MOTION_H

void printPath(const std::vector<cv::Point> pathCoords, cv::Point2d initPos, double speed);

#endif // MOTION_H



