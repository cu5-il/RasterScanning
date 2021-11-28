#pragma once
#include<vector>
#include<opencv2/core.hpp>

typedef struct {
	double x;
	double y;
	double z;
	double T;
}Coords;

typedef struct {
	cv::Point2d ScanDonePt; //point when the segment should be done scanning in [mm]
	std::vector<cv::Rect> ROI; //bounding box around the segment which should contain the edges
	std::vector<cv::Point> centerline; //centerline of the path in the segment
}segmentInfo;

typedef struct {
	std::vector<cv::Rect> edgeROI;
	std::vector<cv::Point> lEdgePts; //left edge points in the region
	std::vector<cv::Point> rEdgePts; //right edge points in the region
	std::vector<std::vector<double>> errCL; //centerline error
	std::vector<std::vector<double>> errWD; //width error
}pathSegment;

typedef struct {
	std::vector<cv::Rect> edgeROI;
	std::vector<cv::Point> lEdgePts; //left edge points in the region
	std::vector<cv::Point> rEdgePts; //right edge points in the region
	std::vector<std::vector<double>> errCL; //centerline error
	std::vector<std::vector<double>> errWD; //width error
}scan2errorPacket;