#pragma once
#include <vector>
#include <opencv2/core.hpp>

#ifndef EDGE_FNS_H
#define EDGE_FNS_H

void makeSegments(const std::vector<cv::Point>& rasterCoords, double ROIwidth, std::vector<Segment>& seg);

void getMatlEdges(const cv::Rect& segmentROI, const cv::Mat& gblEdges, std::vector<cv::Point>& lEdgePts, std::vector<cv::Point>& rEdgePts, bool interp = false);

void getMatlErrors(std::vector<cv::Point>& centerline, double width, cv::Size rasterSize, const std::vector<cv::Point>& lEdgePts, const std::vector<cv::Point>& rEdgePts, std::vector<double>& errCL, std::vector<double>& errWD);

#endif // EDGE_FNS_H