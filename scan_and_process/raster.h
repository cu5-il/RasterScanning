#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>
#include <iterator>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"


#ifndef RASTER_H
#define RASTER_H

class Raster {
private:
    cv::Mat _mat;
    cv::Mat _boundary;
    std::vector<cv::Point> _cornersPix;
    std::vector<cv::Point> _coordsPix;
    std::vector<cv::Point2d> _cornersMM;
    std::vector<cv::Point2d> _coordsMM;

public:
	Raster(double length, double spacing, double border, double bdryWidth);

    const cv::Mat& mat() { return _mat; }
    const cv::Mat& boundary() { return _boundary; };
    const std::vector<cv::Point>& cornersPix() { return _cornersPix; };
    const std::vector<cv::Point>& coordsPix() { return _coordsPix; };
    const std::vector<cv::Point2d>& cornersMM() { return _cornersMM; };
    const std::vector<cv::Point2d>& coordsMM() { return _coordsMM; };

};

inline Raster::Raster(double length, double spacing, double border, double bdryWidth) {

    int i = 0;
    int pixLen = MM2PIX(length);
    int pixSpac = MM2PIX(spacing);
    int pixBord = MM2PIX(border);
    cv::Point diff; 
    double dx, dy, L;
    std::vector<cv::Point2d> temp;


    // initialize matrix to store raster with border
    _mat = cv::Mat(pixLen + 2 * pixBord, pixLen + 2 * pixBord, CV_8U, cv::Scalar(0)).clone();
    _boundary = _mat.clone();

    // add the first point to the raster
    _cornersPix.push_back(cv::Point(pixBord, pixBord));

    while (_cornersPix.back().x <= (pixLen + _cornersPix.front().x) && _cornersPix.back().y <= (pixLen + _cornersPix.front().y)) {
        // Convert the points to mm
        _cornersMM.push_back(PIX2MM(cv::Point2d(_cornersPix.back())));

        switch (i % 4) {
        case 0:
            _cornersPix.push_back(_cornersPix.back() + cv::Point(0, pixLen));
            break;
        case 1:
            _cornersPix.push_back(_cornersPix.back() + cv::Point(pixSpac, 0));
            break;
        case 2:
            _cornersPix.push_back(_cornersPix.back() - cv::Point(0, pixLen));
            break;
        case 3:
            _cornersPix.push_back(_cornersPix.back() + cv::Point(pixSpac, 0));
            break;
        }
        i++;
    }

    // Remove the last point of the raster that was outside of the pattern area
    _cornersPix.pop_back();

    // Draw the raster lines on an image
    cv::polylines(_mat, _cornersPix, false, cv::Scalar(255), 1, 4);
    cv::polylines(_boundary, _cornersPix, false, cv::Scalar(255), MM2PIX(bdryWidth), 8);

    // Interpolate the points between corners
    for (auto it = _cornersPix.begin(); it != std::prev(_cornersPix.end(),1); ++it) {

        diff = *std::next(it, 1) - *it;
        L = cv::norm(diff);
        dx = diff.x / L;
        dy = diff.y / L;
        for (int i = 0; i < L; i++) {
            temp.push_back(cv::Point2d(*it) + i * cv::Point2d(dx,dy));
            _coordsPix.push_back(cv::Point(temp.back()));
            _coordsMM.push_back(PIX2MM(temp.back()));
        }
    }
    temp.push_back(cv::Point2d(_cornersPix.back()));
    _coordsPix.push_back(cv::Point(temp.back()));
    _coordsMM.push_back(PIX2MM(temp.back()));

}

#endif // RASTER_H
