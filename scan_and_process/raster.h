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
    double _width;
    double _spacing;

    void _makeRaster(double length, double width, double rodSpacing, double rodWidthMax);

public:
    // default constructor
    Raster();

    /**
     * @brief Construsctor for a square raster
     * @param length length & width of the raster
     * @param rodSpacing spacing between the rods of a raster
     * @param rodWidthMax maximum width of the raster rods (for finding edges)
    */
    Raster(double length, double rodSpacing, double rodWidthMax);

    /**
     * @brief Constructor for a non-square raster
     * @param length length of the raster 
     * @param width  width of the raster
     * @param rodSpacing spacing between the rods of a raster
     * @param rodWidthMax maximum width of the raster rods (for finding edges)
    */
    Raster(double length, double width, double rodSpacing, double rodWidthMax);

    const cv::Rect2d& roi() { return _roi; }
    const cv::Mat& boundaryMask() { return _boundaryMask; }
    const std::vector<cv::Point>& px() { return _cornersPix; }
    const std::vector<cv::Point2d>& mm() { return _cornersMM; }
    const cv::Size& size() { return _sz; }
    const int& rodWidth() { return _rodWidth; }
    const double& length() { return _length; }
    const double& width() { return _width; }
    const double& spacing() { return _spacing; }
    const cv::Point2d& origin() { return _roi.tl(); }
    
    void offset(cv::Point2d);
    const cv::Mat& draw() { return _mat; }
    void draw(cv::Mat src, cv::Mat& dst, const cv::Scalar& color = cv::Scalar(255, 255, 255), int thickness = 1);
    void drawBdry(cv::Mat src, cv::Mat& dst, const cv::Scalar& color, int thickness = 1);
};

inline Raster::Raster()
    : _rodWidth(0), _length(0), _spacing(0) {}

inline Raster::Raster(double length, double rodSpacing, double rodWidthMax) {
    _makeRaster(length, length, rodSpacing, rodWidthMax);
}

inline Raster::Raster(double length, double width, double rodSpacing, double rodWidthMax) {
    _makeRaster(length, width, rodSpacing, rodWidthMax);
}

inline void Raster::_makeRaster(double length, double width, double rodSpacing, double rodWidthMax) {

    int i = 0;
    double border = rodWidthMax / 2 + 1;
    int pixLen = MM2PIX(length);
    int pixWth = MM2PIX(width);
    int pixSpac = MM2PIX(rodSpacing);
    int pixBord = MM2PIX(border);

    _rodWidth = MM2PIX(rodWidthMax);
    _length = length;
    _width = width;
    _spacing = rodSpacing;

    // initialize matrix to store raster with border
    _sz = cv::Size(pixLen + 2 * pixBord, pixWth + 2 * pixBord);
    _mat = cv::Mat(_sz, CV_8U, cv::Scalar(0)).clone();
    _boundaryMask = _mat.clone();

    // add the first point to the raster
    _cornersPix.push_back(cv::Point(pixBord, pixBord));

    while (_cornersPix.back().x <= (pixLen + _cornersPix.front().x) && _cornersPix.back().y <= (pixWth + _cornersPix.front().y)) {
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
    _roi = cv::Rect2d(0, 0, 2 * border + length, 2 * border + width);

    // offset the coordinates
    offset(-_cornersMM.front());
}

inline void Raster::offset(cv::Point2d offset) {
    for (auto it = _cornersMM.begin(); it != _cornersMM.end(); ++it) { *it += offset; }
    _roi += offset;
}

inline void Raster::draw(cv::Mat src, cv::Mat& dst, const cv::Scalar& color, int thickness ) {
    // copy the source to the destination
    src.copyTo(dst);
    if (dst.channels() < 3) {
        cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
    }

    // draw the raster on the image
    cv::polylines(dst, _cornersPix, false, color, thickness, 4);
}

inline void Raster::drawBdry(cv::Mat src, cv::Mat& dst, const cv::Scalar& color, int thickness) {
    // copy the source to the destination
    src.copyTo(dst);
    if (dst.channels() < 3) {
        cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
    }

    // draw the boundary on the image
    cv::polylines(dst, _boundaryPoints, true, color, thickness, 4);
}

#endif // RASTER_H
