#include "draw.h"
#include <iostream>
#include "constants.h"
#include "myTypes.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iterator>

void mouse_callback(int  event, int  x, int  y, int  flag, void* param)
{
	if (event == cv::EVENT_LBUTTONDOWN) {
		std::cout << "(" << x << ", " << y << ")" << std::endl;
	}
}

static cv::Scalar randomColor(cv::RNG& rng){
	int icolor = (unsigned)rng;
	return cv::Scalar(icolor & 255, (icolor >> 8) & 255, (icolor >> 16) & 255);
}

void addScale(cv::Mat& image, double length, cv::Point offset, double fontScale) {
	cv::Point location(offset.x, image.rows - offset.y);
	char buff[50];
	snprintf(buff, sizeof(buff), "%gmm", length);
	cv::putText(image, buff, location, cv::FONT_HERSHEY_SIMPLEX, fontScale, cv::Scalar(255, 255, 255), 1, cv::LINE_8);
	cv::rectangle(image, cv::Rect(location.x + 6, location.y + 5, MM2PIX(length), 5), cv::Scalar(255, 255, 255), -1);
}

void drawEdges(cv::Mat src, cv::Mat&  dst, cv::Mat edges, const cv::Scalar& color, const int pointSz) {
	
	// copy the source to the destination
	src.copyTo(dst);
	if (dst.channels() < 3) {
		cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
	}

	// Copy the global edges to the  image
	if (pointSz > 0) {
		// show the edges as filled circles
		std::vector<cv::Point> edgePts;
		cv::findNonZero(edges, edgePts);
		for (auto it = edgePts.begin(); it != edgePts.end(); ++it) {
			cv::circle(dst, *it, pointSz, color, -1, cv::LINE_AA);
		}
	}
	else {
		// show the edges as pixels
		cv::Mat(edges.size(), CV_8UC3, color).copyTo(dst, edges);
	}
}

void drawErrors(cv::Mat src, cv::Mat& dst, std::vector<Segment>& seg, int layer) {
	std::vector<cv::Point> allEdgePts, actCenterline, lEdgeErr, rEdgeErr;
	cv::Mat tempLines= cv::Mat::zeros(src.size(), CV_8UC3);
	cv::Mat matlAct = cv::Mat::zeros(src.size(), CV_8UC3);
	cv::Mat mask = cv::Mat(src.size(), CV_8UC3);

	// getting the values from each segment
	for (auto it = seg.begin(); it != seg.end(); ++it) {
		if ((*it).layer() == layer)
		{
			// check if there are edge points
			if (!(*it).lEdgePts().empty() && !(*it).rEdgePts().empty()) {
				// Draw material edges
				cv::polylines(tempLines, (*it).lEdgePts(), false, cv::Scalar(255, 255, 0), 1);
				cv::polylines(tempLines, (*it).rEdgePts(), false, cv::Scalar(255, 255, 0), 1);
				// Fill the region between the edges
				allEdgePts.reserve((*it).lEdgePts().size() + (*it).rEdgePts().size()); // preallocate memory
				allEdgePts.insert(allEdgePts.end(), (*it).lEdgePts().begin(), (*it).lEdgePts().end());
				allEdgePts.insert(allEdgePts.end(), (*it).rEdgePts().rbegin(), (*it).rEdgePts().rend());
				cv::fillPoly(matlAct, allEdgePts, cv::Scalar(255, 255, 0));
				allEdgePts.clear();
			}
			// check if the errors have been calculated
			if (!(*it).errCL().empty() && !(*it).errWD().empty()) {
				for (int i = 0; i < (*it).errCL().size(); i++) {
					if (!isnan((*it).errCL()[i]) && !isnan((*it).errWD()[i])) {
						actCenterline.push_back((*it).waypoints()[i] + cv::Point(0, MM2PIX((*it).errCL()[i])));
						lEdgeErr.push_back(actCenterline.back() - cv::Point(0, MM2PIX((*it).errWD()[i] / 2)));
						rEdgeErr.push_back(actCenterline.back() + cv::Point(0, MM2PIX((*it).errWD()[i] / 2)));
					}
				}
				// Draw the errors
				cv::polylines(tempLines, lEdgeErr, false, cv::Scalar(0, 255, 255), 1);
				cv::polylines(tempLines, rEdgeErr, false, cv::Scalar(0, 255, 255), 1);
				cv::polylines(tempLines, actCenterline, false, cv::Scalar(0, 0, 255), 1);
				lEdgeErr.clear();
				rEdgeErr.clear();
				actCenterline.clear();
			}
		}
	}
	// copy the source to the destination
	src.copyTo(dst);
	if (dst.channels() < 3) {
		cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
	}
	// overlay the fill on the destination
	cv::addWeighted(matlAct, 0.5, dst, 1, 0, dst);
	// add the lines to the destination
	cv::cvtColor(tempLines, mask, cv::COLOR_BGR2GRAY);
	cv::threshold(mask, mask, 1, 255, cv::THRESH_BINARY);
	tempLines.copyTo(dst, mask);
}

void drawMaterial(cv::Mat src, cv::Mat& dst, std::vector<Segment>& seg, std::vector<std::vector<Path>> path, int layer) {
	std::vector<cv::Point> allEdgePts, actCenterline, lEdge, rEdge;
	cv::Mat tempLines = cv::Mat::zeros(src.size(), CV_8UC3);
	cv::Mat matlAct = cv::Mat::zeros(src.size(), CV_8UC3);
	cv::Mat matlDes = cv::Mat::zeros(src.size(), CV_8UC3);
	cv::Mat mask = cv::Mat(src.size(), CV_8UC3);

	cv::Scalar colorMatlAct(255, 255, 0);
	cv::Scalar colorMatlDes(255, 255, 255);

	// getting the values from each segment
	for (auto it = seg.begin(); it != seg.end(); ++it) {
		if ((*it).layer() == layer)
		{
			// check if there are edge points
			if (!(*it).lEdgePts().empty() && !(*it).rEdgePts().empty()) {
				// Draw material edges
				//cv::polylines(tempLines, (*it).lEdgePts(), false, cv::Scalar(255, 255, 0), 1);
				//cv::polylines(tempLines, (*it).rEdgePts(), false, cv::Scalar(255, 255, 0), 1);
				// Fill the region between the edges
				allEdgePts.reserve((*it).lEdgePts().size() + (*it).rEdgePts().size()); // preallocate memory
				allEdgePts.insert(allEdgePts.end(), (*it).lEdgePts().begin(), (*it).lEdgePts().end());
				allEdgePts.insert(allEdgePts.end(), (*it).rEdgePts().rbegin(), (*it).rEdgePts().rend());
				cv::fillPoly(matlAct, allEdgePts, colorMatlAct);
				allEdgePts.clear();
			}
			// check if the errors have been calculated
			if (!(*it).errCL().empty() && !(*it).errWD().empty()) {
				for (int i = 0; i < (*it).errCL().size(); i++) {
					if (!isnan((*it).errCL()[i]) && !isnan((*it).errWD()[i])) {
						actCenterline.push_back((*it).waypoints()[i] + cv::Point(0, MM2PIX((*it).errCL()[i])));

					}
				}
				// Draw the centerline error
				cv::polylines(tempLines, actCenterline, false, cv::Scalar(0, 0, 255), 1);

				actCenterline.clear();
			}
		}
	}

	// drawing the desired material
	for (int i = 0; i < path.size(); i += 2) {
		// loop through all the waypoints
		for (int j = 0; j < path[i].size(); j++) {
			lEdge.push_back(seg[i].waypoints()[j] - cv::Point(0, MM2PIX(path[i][j].w / 2)));
			rEdge.push_back(seg[i].waypoints()[j] + cv::Point(0, MM2PIX(path[i][j].w / 2)));
			
		}
	}
	// Fill the region between the edges
	allEdgePts.reserve(lEdge.size() + rEdge.size()); // preallocate memory
	allEdgePts.insert(allEdgePts.end(), lEdge.begin(), lEdge.end());
	allEdgePts.insert(allEdgePts.end(), rEdge.rbegin(), rEdge.rend());
	cv::fillPoly(matlDes, allEdgePts, colorMatlDes);
	//cv::polylines(tempLines, lEdge, false, cv::Scalar(0, 255, 255), 1);
	//cv::polylines(tempLines, rEdge, false, cv::Scalar(0, 255, 255), 1);

	// copy the source to the destination
	src.copyTo(dst);
	if (dst.channels() < 3) {
		cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
	}
	// overlay the material
	cv::addWeighted(matlDes, 0.5, matlAct, 0.5, 0, matlDes);
	cv::addWeighted(matlDes, 0.75, dst, 1, 0, dst);
	// add the lines to the destination
	cv::cvtColor(tempLines, mask, cv::COLOR_BGR2GRAY);
	cv::threshold(mask, mask, 1, 255, cv::THRESH_BINARY);
	tempLines.copyTo(dst, mask);
	return;
}

void drawSegments(cv::Mat src, cv::Mat& dst, std::vector<Segment>& seg, cv::Point2d origin, int layer, const int pointSz) {
	//cv::Point origin = seg[0].waypoints()[0];
	cv::Point ScanDonePt(0,0);
	cv::Scalar color = cv::Scalar(255, 255, 0);
	cv::RNG rng(0xFFFFFFFF);
	int segNum = 0;
	int samePtCnt = 0;
	char buff[50];
	cv::Size textSz;
	int baseline = 0;

	// copy the source to the destination
	src.copyTo(dst);
	if (dst.channels() < 3) {
		cv::cvtColor(dst, dst, cv::COLOR_GRAY2BGR);
	}

	// getting the values from each segment
	for (auto it = seg.begin(); it != seg.end(); ++it, segNum++) {
		if ((*it).layer() == layer)
		{
			// Randomize the color
			color = randomColor(rng);
			// Draw the segment boundaries
			cv::rectangle(dst, (*it).ROI(), color);
			// Draw the waypoints
			for (auto it2 = (*it).waypoints().begin(); it2 != (*it).waypoints().end(); ++it2) {
				cv::circle(dst, *it2, pointSz, color, -1, cv::LINE_AA);
			}
			// Draw and label the scan done point
			snprintf(buff, sizeof(buff), "%u", segNum);
			ScanDonePt = cv::Point(MM2PIX((*it).scanDonePt().x - origin.x), MM2PIX((*it).scanDonePt().y - origin.y));
			if (it != seg.begin() && ScanDonePt == cv::Point(MM2PIX((*std::prev(it)).scanDonePt().x - origin.x), MM2PIX((*std::prev(it)).scanDonePt().y - origin.y))) {
				samePtCnt++;
				textSz = cv::getTextSize(buff, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseline);
				cv::putText(dst, buff, ScanDonePt + cv::Point(pointSz + samePtCnt * textSz.width, -2), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_8);
			}
			else {
				cv::putText(dst, buff, ScanDonePt + cv::Point(pointSz, -2), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1, cv::LINE_8);
				samePtCnt = 0;
			}
			cv::drawMarker(dst, ScanDonePt, color, cv::MARKER_TILTED_CROSS, 10, 1);
		}
	}
}