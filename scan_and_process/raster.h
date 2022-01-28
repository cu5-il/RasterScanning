#pragma once
#include <iostream>
#include <fstream>
#include <string>
#include <cmath>
#include <vector>
#include <iterator>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"

#ifndef RASTER_H
#define RASTER_H

class Raster {
private:
    cv::Size _sz;
    cv::Mat _mat;
    cv::Rect2d _roi;
    cv::Mat _boundaryMask;
    std::vector<cv::Point> _boundaryPoints;
    std::vector<cv::Point> _cornersPix;
    std::vector<cv::Point2d> _cornersMM;
    int _rodWidth;
    double _length;
    double _spacing;

public:
    // default constructor
    Raster();

    Raster(double rodLength, double rodSpacing, double rodWidthMax);

    const cv::Mat& draw() { return _mat; }
    const cv::Rect2d& roi() { return _roi; }
    const cv::Mat& boundaryMask() { return _boundaryMask; };
    //const std::vector<cv::Point>& boundaryPoints() { return _boundaryPoints; };
    const std::vector<cv::Point>& px() { return _cornersPix; };
    const std::vector<cv::Point2d>& mm() { return _cornersMM; };
    const cv::Size& size() { return _sz; }
    const int& rodWidth() { return _rodWidth; };
    const double& length() { return _length; };
    const double& spacing() { return _spacing; };
    const cv::Point2d& origin() { return _roi.tl(); };

    void offset(cv::Point2d);

};

inline Raster::Raster()
    : _rodWidth(0), _length(0), _spacing(0) {}

inline Raster::Raster(double rodLength, double rodSpacing, double rodWidthMax) {

    int i = 0;
    double border = rodWidthMax / 2 + 1;
    int pixLen = MM2PIX(rodLength);
    int pixSpac = MM2PIX(rodSpacing);
    int pixBord = MM2PIX(border);

    _rodWidth = MM2PIX(rodWidthMax);
    _length = rodLength;
    _spacing = rodSpacing;
  
    // initialize matrix to store raster with border
    _sz = cv::Size(pixLen + 2 * pixBord, pixLen + 2 * pixBord);
    _mat = cv::Mat(_sz, CV_8U, cv::Scalar(0)).clone();
    _boundaryMask = _mat.clone();

    // add the first point to the raster
    _cornersPix.push_back(cv::Point(pixBord, pixBord));

    while (_cornersPix.back().x <= (pixLen + _cornersPix.front().x) && _cornersPix.back().y <= (pixLen + _cornersPix.front().y)) {
        // Convert the points to mm
        _cornersMM.push_back(PIX2MM(cv::Point2d(_cornersPix.back() /*- _cornersPix.front()*/)));

        switch (i % 4) {
        case 0:
            _cornersPix.push_back(_cornersPix.back() + cv::Point(pixLen, 0));
            break;
        case 1:
            _cornersPix.push_back(_cornersPix.back() + cv::Point(0, pixSpac));
            break;
        case 2:
            _cornersPix.push_back(_cornersPix.back() - cv::Point(pixLen, 0));
            break;
        case 3:
            _cornersPix.push_back(_cornersPix.back() + cv::Point(0, pixSpac));
            break;
        }
        i++;
    }

    // Remove the last point of the raster that was outside of the pattern area
    _cornersPix.pop_back();

    // Draw the raster lines on an image
    cv::polylines(_mat, _cornersPix, false, cv::Scalar(255), 1, 4);
    cv::polylines(_boundaryMask, _cornersPix, false, cv::Scalar(255), _rodWidth, 8);
    
    // Get the boundary points
    std::vector<std::vector<cv::Point> > contour;
    cv::findContours(_boundaryMask, contour, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    _boundaryPoints = contour[0];


    // define the roi of the raster
    _roi = cv::Rect2d(0, 0, 2 * border + rodLength, 2 * border + rodLength);

    // offset the coordinates
    offset(-_cornersMM.front());
}

inline void Raster::offset(cv::Point2d offset) {
    for (auto it = _cornersMM.begin(); it != _cornersMM.end(); ++it) { *it += offset; }
    _roi += offset;
}

#endif // RASTER_H
