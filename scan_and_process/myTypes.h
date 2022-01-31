#pragma once
#include<vector>
#include <string>
#include<opencv2/core.hpp>
#include <opencv2/imgproc.hpp> // TODO: get rid of this but 

#ifndef MY_TYPES_H
#define MY_TYPES_H
// TODO: change Coords to structure
struct Coords {
	double x;
	double y;
	double z;
	double T;
};

///////////////////////////////////////  Path  ///////////////////////////////////////
class Path
{
public:
	double x;
	double y;
	double z;
	double T;
	double f;
	double e;

	Path();
	Path(double _x, double _y, double _z, double _T, double _f, double _e);
	Path(const Path& in);
	Path(const cv::Point2d& pt, double _z, double _T, double _f, double _e);

	const char* cmd();

private:
	std::string _cmd;
};

inline Path::Path()
	 : x(0), y(0), z(0), T(0), f(0), e(0) {}

inline Path::Path(double x_, double y_, double z_, double T_, double f_, double e_)
	 : x(x_), y(y_), z(z_), T(T_), f(f_), e(e_) {}

inline Path::Path(const Path& in)
	 : x(in.x), y(in.y), z(in.z), T(in.T), f(in.f), e(in.e) {}

inline Path::Path(const cv::Point2d& pt, double z_, double T_, double f_, double e_)
	 : x(pt.x), y(pt.y), z(z_), T(T_), f(f_), e(e_) {}

inline const char* Path::cmd(){
	_cmd = "$AO[1].X =" + std::to_string(e) + "\n";
	_cmd += "G1 X " + std::to_string(x) + " Y " + std::to_string(y) + " Z " + std::to_string(z) + " TH " + std::to_string(T) + " F " + std::to_string(f);
	return _cmd.c_str();
}

///////////////////////////////////////  Segment  ///////////////////////////////////////
class Segment {

public:
	Segment(cv::Rect ROI, std::vector<cv::Point> waypoints, cv::Point2d scanDonePt, int direction) {
		_ROI = ROI;
		_waypoints = waypoints;
		_scanDonePt = scanDonePt;
		_dir = direction;
	}
	
	void addEdges(std::vector<cv::Point> lEdgePts, std::vector<cv::Point> rEdgePts) {
		_lEdgePts = lEdgePts;
		_rEdgePts = rEdgePts;
	}
	void addErrors(std::vector<double> errCL, std::vector<double> errWD) {
		_errCL = errCL;
		_errWD = errWD;
	}
	// Get values
	const cv::Rect& ROI() const { return _ROI; }
	const std::vector<cv::Point>& waypoints() const { return _waypoints; }
	const cv::Point2d& scanDonePt() const { return _scanDonePt; }
	const int& dir() const { return _dir; }
	const std::vector<cv::Point>& lEdgePts() const { return _lEdgePts; }
	const std::vector<cv::Point>& rEdgePts() const { return _rEdgePts; }
	const std::vector<double>& errCL() const { return _errCL; }
	const std::vector<double>& errWD() const { return _errWD; }

private:
	cv::Rect _ROI; //bounding box around the segment which should contain the edges
	std::vector<cv::Point> _waypoints; //centerline of the path in the segment
	cv::Point2d _scanDonePt; //point when the segment should be done scanning in [mm]
	int _dir; // direction of the segment. 1 if moving in the positive direction, -1 if moving in the negative directon
	std::vector<cv::Point> _lEdgePts; //left edge points in the region
	std::vector<cv::Point> _rEdgePts; //right edge points in the region
	std::vector<double> _errCL; //centerline error
	std::vector<double> _errWD; //width error
};

///////////////////////////////////////  edgeMsg  ///////////////////////////////////////
class edgeMsg {
private:
	cv::Mat _edges;
	int _segmentNum;
	bool _doneScanning;
public:
	edgeMsg() {
		_segmentNum = 0;
		_doneScanning = false;
	}
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

///////////////////////////////////////  errsMsg  ///////////////////////////////////////
class errsMsg {
private:
	std::vector<double> _errCL; //centerline error
	std::vector<double> _errWD; //width error
	int _segmentNum;
	bool _doneProcessing;
public:
	errsMsg() {
		_segmentNum = 0;
		_doneProcessing = false;
	}
	void addErrors(std::vector<double> errCL, std::vector<double> errWD, int segmentNum) {
		_errCL = errCL;
		_errWD = errWD;
		_segmentNum = segmentNum;
	}
	// Get values
	const std::vector<double>& errCL() const { return _errCL; }
	const std::vector<double>& errWD() const { return _errWD; }
	const int& segmentNum() const { return _segmentNum; }
	const bool& doneProcessing() const { return _doneProcessing; }
};

///////////////////////////////////////  pathMsg  ///////////////////////////////////////
class pathMsg {
private:
	std::vector<Path> _path; 
	int _segmentNum;
public:
	pathMsg() {
		_segmentNum = 0;
	}
	void addPath(std::vector<Path> path, int segmentNum) {
		_path = path;
		_segmentNum = segmentNum;
	}
	// Get values
	const std::vector<Path>& path() const { return _path; }
	const int& segmentNum() const { return _segmentNum; }
};
#endif // MY_TYPES_H