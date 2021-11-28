#pragma once
#include <vector>
#include <opencv2/core.hpp>

void makeSegments(const std::vector<cv::Point>& rasterCoords, double width, std::vector<cv::Rect>& ROIs, std::vector<std::vector<cv::Point>>& centerlines, std::vector<cv::Point2d>& scanDonePts);

void getMatlEdges(const cv::Rect& edgeRegions, cv::Mat& gblEdges, std::vector<cv::Point>& lEdgePts, std::vector<cv::Point>& rEdgePts, bool interp = false);

void getMatlErrors(std::vector<cv::Point>& centerline, double width, cv::Size rasterSize, std::vector<cv::Point>& lEdgePts, std::vector<cv::Point>& rEdgePts, std::vector <std::vector<double>>& errCL, std::vector<std::vector<double>>& errWD);