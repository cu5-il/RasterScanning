#pragma once
#include<vector>
#include<opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#ifndef MY_TYPES_H
#define MY_TYPES_H
// TODO: change Coords to structure
typedef struct Coords {
	double x;
	double y;
	double z;
	double T;
};

class Segment {
private:
	cv::Rect _ROI; //bounding box around the segment which should contain the edges
	std::vector<cv::Point> _centerline; //centerline of the path in the segment
	cv::Point2d _scanDonePt; //point when the segment should be done scanning in [mm]
	int _dir;
	std::vector<cv::Point> _lEdgePts; //left edge points in the region
	std::vector<cv::Point> _rEdgePts; //right edge points in the region
	std::vector<double> _errCL; //centerline error
	std::vector<double> _errWD; //width error
public:
	Segment(cv::Rect ROI, std::vector<cv::Point> centerline, cv::Point2d scanDonePt, int direction) {
		_ROI = ROI;
		_centerline = centerline;
		_scanDonePt = scanDonePt;
		_dir = direction;
	}
	void addEdges(std::vector<cv::Point> lEdgePts, std::vector<cv::Point> rEdgePts) {
		_lEdgePts = lEdgePts;
		_rEdgePts = rEdgePts;
	}
	void addErrors(std::vector<double> errCL, std::vector<double> errWD, std::vector<cv::Point> centerline) {
		_errCL = errCL;
		_errWD = errWD;
		_centerline = centerline;
	}
	// Get values
	const cv::Rect& ROI() const { return _ROI; }
	const std::vector<cv::Point>& centerline() const { return _centerline; }
	const cv::Point2d& scanDonePt() const { return _scanDonePt; }
	const int& dir() const { return _dir; }
	const std::vector<cv::Point>& lEdgePts() const { return _lEdgePts; }
	const std::vector<cv::Point>& rEdgePts() const { return _rEdgePts; }
	const std::vector<double>& errCL() const { return _errCL; }
	const std::vector<double>& errWD() const { return _errWD; }
};

class edgeMsg {
private:
	cv::Mat _edges;
	int _segmentNum;
	bool _doneScanning;
public:
	void addEdges(cv::Mat edges, int segmentNum, bool doneScanning) {
		_edges = edges;
		_segmentNum = segmentNum;
		_doneScanning = doneScanning;
	}
	// Get values
	const cv::Mat& edges() const { return _edges; }
	const int& segmentNum() const { return _segmentNum; }
	const bool& doneScanning() const { return _doneScanning; }
};

#endif // MY_TYPES_H