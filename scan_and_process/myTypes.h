#pragma once
#include<vector>
#include<opencv2/core.hpp>

// TODO: check if typedef is needed
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
	std::vector<cv::Point> _lEdgePts; //left edge points in the region
	std::vector<cv::Point> _rEdgePts; //right edge points in the region
	std::vector<double> _errCL; //centerline error
	std::vector<double> _errWD; //width error
public:
	Segment(cv::Rect ROI, std::vector<cv::Point> centerline, cv::Point2d scanDonePt) {
		this->_ROI = ROI;
		_centerline = centerline;
		_scanDonePt = scanDonePt;
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
	const std::vector<cv::Point>& lEdgePts() const { return _lEdgePts; }
	const std::vector<cv::Point>& rEdgePts() const { return _rEdgePts; }
	const std::vector<double>& errCL() const { return _errCL; }
	const std::vector<double>& errWD() const { return _errWD; }
};

typedef struct scan2errorPacket {
	std::vector<cv::Rect> edgeROI;
	std::vector<cv::Point> lEdgePts; //left edge points in the region
	std::vector<cv::Point> rEdgePts; //right edge points in the region
	std::vector<std::vector<double>> errCL; //centerline error
	std::vector<std::vector<double>> errWD; //width error
};