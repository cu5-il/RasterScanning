#pragma once
#include <iostream>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <opencv2/core.hpp>

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
	double x; // X position
	double y; // Y position
	double z; // Z position
	double T; // TH position
	double f; // Feed rate
	double e; // Auger current command
	double w; // Target width at waypoint

	Path();
	Path(double _x, double _y, double _z, double _T, double _f, double _e, double _w);
	Path(const Path& in);
	Path(const cv::Point2d& pt, double _z, double _T, double _f, double _e, double _w);

	const char* cmd();
	const char* cmd(bool cmdTheta);

private:
	std::string _cmd;
};

inline Path::Path()
	 : x(0), y(0), z(0), T(0), f(0), e(0), w(0) {}

inline Path::Path(double _x, double _y, double _z, double _T, double _f, double _e, double _w)
	 : x(_x), y(_y), z(_z), T(_T), f(_f), e(_e), w(_w) {}

inline Path::Path(const Path& in)
	 : x(in.x), y(in.y), z(in.z), T(in.T), f(in.f), e(in.e), w(in.w) {}

inline Path::Path(const cv::Point2d& pt, double _z, double _T, double _f, double _e, double _w)
	 : x(pt.x), y(pt.y), z(_z), T(_T), f(_f), e(_e), w(_w) {}

inline const char* Path::cmd(){
	_cmd = "$AO[1].X =" + std::to_string(e) + "\n";
	_cmd += "G1 X " + std::to_string(x) + " Y " + std::to_string(y) + " Z " + std::to_string(z) + " TH " + std::to_string(T) + " F " + std::to_string(f);
	return _cmd.c_str();
}

inline const char* Path::cmd(bool cmdTheta) {
	_cmd = "$AO[1].X =" + std::to_string(e) + "\n";
	_cmd += "G1 X " + std::to_string(x) + " Y " + std::to_string(y) + " Z " + std::to_string(z);
	if (cmdTheta) { _cmd += " TH " + std::to_string(T); }
	_cmd += " F " + std::to_string(f);
	return _cmd.c_str();
}

///////////////////////////////////////  Segment  ///////////////////////////////////////
class Segment {

public:
	Segment() :_dir(0), _layer(0) {}

	Segment(cv::Rect ROI, std::vector<cv::Point> waypoints, cv::Point2d scanDonePt, int direction, int layer = 0) 
	: _ROI(ROI), _waypoints(waypoints), _scanDonePt(scanDonePt), _dir(direction), _layer(layer){}
	
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
	const int& layer() const { return _layer; }
	const std::vector<cv::Point>& lEdgePts() const { return _lEdgePts; }
	const std::vector<cv::Point>& rEdgePts() const { return _rEdgePts; }
	const std::vector<double>& errCL() const { return _errCL; }
	const std::vector<double>& errWD() const { return _errWD; }

	// Set values
	void setScanDonePt(cv::Point2d point)  { _scanDonePt = point; }

private:
	cv::Rect _ROI; //bounding box around the segment which should contain the edges
	std::vector<cv::Point> _waypoints; //centerline of the path in the segment
	cv::Point2d _scanDonePt; //point when the segment should be done scanning in [mm]
	int _dir; // direction of the segment. 1 if moving in the positive direction, -1 if moving in the negative directon
	std::vector<cv::Point> _lEdgePts; //left edge points in the region
	std::vector<cv::Point> _rEdgePts; //right edge points in the region
	std::vector<double> _errCL; //centerline error
	std::vector<double> _errWD; //width error
	int _layer;
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

///////////////////////////////////////  PrintOptions  ///////////////////////////////////////
class PrintOptions
{
public:
	double leadin; // length of lead in line. Negative values disable lead in
	double leadout; // length of lead out line. Negative values disable lead out
	bool extrude; // flag determining whether or not extrude 
	bool disposal; // flag determining whether or not move to the disposal zone at the end of a print 
	double asyncTheta; // if set greater than 0, moves the theta axis asynchronously at the start of each corner at the set rate

	PrintOptions();
	PrintOptions(double _leadin, double _leadout = -1, bool _extrude = true, bool _disposal = true);

private:
	
};
inline PrintOptions::PrintOptions()
	: leadin(-1), leadout(-1), extrude(true), disposal(true), asyncTheta(0){}

inline PrintOptions::PrintOptions(double _leadin, double _leadout, bool _extrude, bool _disposal)
	: leadin(_leadin), leadout(_leadout), extrude(_extrude), disposal(_disposal) {}


///////////////////////////////////////  PrintDir  ///////////////////////////////////////
class printDir
{
public:
	printDir() {}

	enum dir : int
	{
		X_POS = 0, // positive x direction
		Y_POS = 1, // positive y direction
		X_NEG = 2, // negative x direction
		Y_NEG = 3, // negative y direction
		Z_POS = 4, // positive z direction
		Z_NEG = 5, // negative z direction
	};

	static bool X(int dir) { return (dir == X_POS || dir == X_NEG); }
	static bool Y(int dir) { return (dir == Y_POS || dir == Y_NEG); }
	static bool Z(int dir) { return (dir == Z_POS || dir == Z_NEG); }

private:
};

#endif // MY_TYPES_H