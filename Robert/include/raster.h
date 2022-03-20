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
    // variables for the rotated versions of the base variables - used when making a multi-layer raster
    cv::Mat _Rmat;
    cv::Mat _RboundaryMask;
    std::vector<cv::Point> _RcornersPix;
    std::vector<cv::Point2d> _RcornersMM;
    double _rodWidth;
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
    const cv::Mat& boundaryMask(int layer = 0);
    const std::vector<cv::Point>& px(int layer = 0);
    const std::vector<cv::Point2d>& mm(int layer = 0);
    const cv::Size& size() { return _sz; }
    const double& rodWidth() { return _rodWidth; }
    const double& length() { return _length; }
    const double& width() { return _width; }
    const double& spacing() { return _spacing; }
    const cv::Point2d& origin() { return _roi.tl(); }
    
    void offset(cv::Point2d);
    const cv::Mat& draw(int layer = 0);
    void draw(cv::Mat src, cv::Mat& dst, const cv::Scalar& color = cv::Scalar(255, 255, 255), int thickness = 1);
    void drawBdry(cv::Mat src, cv::Mat& dst, const cv::Scalar& color, int thickness = 1);
};

inline Raster::Raster()
    : _rodWidth(0), _length(0), _width(0), _spacing(0) {}

inline Raster::Raster(double length, double rodSpacing, double rodWidthMax) {
    _makeRaster(length, length, rodSpacing, rodWidthMax);
}

inline Raster::Raster(double length, double width, double rodSpacing, double rodWidthMax) {
    _makeRaster(length, width, rodSpacing, rodWidthMax);
}

inline const cv::Mat& Raster::boundaryMask(int layer){
    _boundaryMask = cv::Mat::zeros(_sz, CV_8UC1);
    cv::polylines(_boundaryMask, px(layer), false, cv::Scalar(255), MM2PIX(_rodWidth), 8);
    return _boundaryMask;
}

inline const std::vector<cv::Point>& Raster::px(int layer){
    cv::transform(_cornersPix, _RcornersPix, cv::getRotationMatrix2D(cv::Point2f(_sz / 2), static_cast<__int64>(layer) * 90, 1));
    return _RcornersPix;
}

inline const std::vector<cv::Point2d>& Raster::mm(int layer){
    cv::transform(_cornersMM, _RcornersMM, cv::getRotationMatrix2D(cv::Point2f((_roi.tl() + _roi.br()) * 0.5), static_cast<__int64>(layer) * 90, 1));
    return _RcornersMM;
}

inline void Raster::_makeRaster(double length, double width, double rodSpacing, double rodWidthMax) {

    int i = 0;
    double border = rodWidthMax / 2 + 1;
    int pixLen = MM2PIX(length);
    int pixWth = MM2PIX(width);
    int pixSpac = MM2PIX(rodSpacing);
    int pixBord = MM2PIX(border);
    int pixRodWth = MM2PIX(rodWidthMax);

    _rodWidth = rodWidthMax;
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
    cv::polylines(_boundaryMask, _cornersPix, false, cv::Scalar(255), pixRodWth, 8);

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

inline const cv::Mat& Raster::draw(int layer){
    _mat = cv::Mat::zeros(_sz, CV_8UC1);
    cv::polylines(_mat, px(layer), false, cv::Scalar(255), 1, 4);
    return _mat;
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
