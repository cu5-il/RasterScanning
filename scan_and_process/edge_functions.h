#pragma once
#include <vector>
#include <opencv2/core.hpp>

void makeEdgeRegions(const std::vector<cv::Point>& rasterCoords, double width, std::vector<cv::Rect>& edgeRegions);

void getMatlEdges(const cv::Rect& edgeRegions, cv::Mat& gblEdges, std::vector<std::vector<cv::Point>>& lEdgePts, std::vector<std::vector<cv::Point>>& rEdgePts, bool interp = false);

void getMatlErrors(std::vector<cv::Point>& centerline, double width, cv::Size rasterSize, std::vector<cv::Point>& lEdgePts, std::vector<cv::Point>& rEdgePts, std::vector <std::vector<double>>& errCL, std::vector<std::vector<double>>& errWD);