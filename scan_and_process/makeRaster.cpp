#include <vector>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "myGlobals.h"
#include "constants.h"

void makeRaster(double length, double spacing, double border, double bdryWidth, cv::Mat& raster, cv::Mat& edgeBoundary, std::vector<cv::Point>& coords ) {

    int i = 0;
    int pixLen = MM2PIX(length);
    int pixSpac = MM2PIX(spacing);
    int pixBord = MM2PIX(border);


    // initialize matrix to store raster with border
    raster = cv::Mat(pixLen + 2 * pixBord, pixLen + 2 * pixBord, CV_8U, cv::Scalar(0)).clone();
    edgeBoundary = raster.clone();

    // add the first point to the raster
    coords.push_back(cv::Point(pixBord, pixBord));

	while (coords.back().x <= (pixLen + coords.front().x) && coords.back().y <= (pixLen + coords.front().y)){
        switch (i % 4) {
        case 0:
            coords.push_back(coords.back() + cv::Point(pixLen, 0));
            break;
        case 1:
            coords.push_back(coords.back() + cv::Point(0, pixSpac));
            break;
        case 2:
            coords.push_back(coords.back() - cv::Point(pixLen, 0));
            break;
        case 3:
            coords.push_back(coords.back() + cv::Point(0, pixSpac));
            break;
        }
        i++;
	}
    // Remove the last point of the raster that was outside of the pattern area
    coords.pop_back();
    // Draw the raster lines on an image
    cv::polylines(raster, coords, false, cv::Scalar(255), 1, 4);
    cv::polylines(edgeBoundary, coords, false, cv::Scalar(255), MM2PIX(bdryWidth), 8);

}