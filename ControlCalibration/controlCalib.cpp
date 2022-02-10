#include <iostream>
#include <vector>
#include <iterator>
#include <string>
#include <fstream>

#include "myTypes.h"
#include "myGlobals.h"
#include "constants.h"
#include "raster.h"
#include "csvMat.h"
#include "thread_functions.h"
#include "draw.h"


void makeTestPath( std::vector<std::vector<Path>>& path, int param, double range[2]) {
	double inc = (range[1]-range[0]) / ceil(path.size() / 2);
	double setVal = range[0];
	int i = 1;

	for (auto it1 = path.begin(); it1 != path.end(); ++it1, i++) {
		for (auto it2 = (*it1).begin(); it2 != (*it1).end(); ++it2) {
			switch (param)
			{
			default:
				break;
			case 0:
				(*it2).f = setVal;
				break;
			case 1:
				(*it2).e = setVal;
				break;
			}
		}
		if (i % 2) {
			setVal += inc;
		}
	}
}

void analyzePrint(Raster raster, std::string filename) {
	double targetWidth = 0;
	double sum, mean, stdev;
	int count;

	edgeMsg msg;
	// loading the data
	cv::Mat edges = cv::Mat::zeros(raster.size(), CV_8U);
	readCSV(filename, edges);
	edges.convertTo(edges, CV_8U);

	cv::Mat imSeg;
	//drawSegments(raster.draw(), imSeg, segments, raster.origin(), 2);
	//drawEdges(imSeg, imSeg, edges, cv::Scalar(0, 255, 0), MM2PIX(0.2));
	
	// calculating the errors
	for (int i = 0; i < segments.size(); i++) {
		msg.addEdges(edges, i, (i == segments.size() - 1));
		q_edgeMsg.push(msg);
	}
	t_GetMatlErrors(raster, targetWidth);
	//drawErrors(imSeg, imSeg, segments);

	// Opening a file to write the results
	std::ofstream outfile;
	outfile.open(std::string(outDir+"widths.txt").c_str());
	outfile.precision(5);

	// calculating the average width for each segment
	for (int i = 0; i < segments.size(); i+=2) {
		sum = 0;
		count = 0;
		// calculate the mean
		for (auto it = segments[i].errWD().begin(); it != segments[i].errWD().end(); ++it) {
			if (!isnan(*it)) {
				sum += *it;
				count++;
			}
		}
		mean = sum / count;
		// calculate the standard deviation
		sum = 0;
		for (auto it = segments[i].errWD().begin(); it != segments[i].errWD().end(); ++it) {
			if (!isnan(*it)) {
				sum += pow((*it-mean),2);
			}
		}
		stdev = sqrt((sum / count));
		outfile << std::fixed << mean << ",\t" << stdev << std::endl;
	}
	outfile.close();
	return;
}

void analyzePrint(Raster raster) {
	double targetWidth = 0;
	double sum, mean, stdev;
	int count;

	// calculating the errors
	t_GetMatlErrors(raster, targetWidth);

	// Opening a file to write the results
	std::ofstream outfile;
	outfile.open(std::string(outDir + "widths.txt").c_str());
	outfile.precision(5);

	// calculating the average width for each segment
	for (int i = 0; i < segments.size(); i += 2) {
		sum = 0;
		count = 0;
		// calculate the mean
		for (auto it = segments[i].errWD().begin(); it != segments[i].errWD().end(); ++it) {
			if (!isnan(*it)) {
				sum += *it;
				count++;
			}
		}
		mean = sum / count;
		// calculate the standard deviation
		sum = 0;
		for (auto it = segments[i].errWD().begin(); it != segments[i].errWD().end(); ++it) {
			if (!isnan(*it)) {
				sum += pow((*it - mean), 2);
			}
		}
		stdev = sqrt((sum / count));
		outfile << std::fixed << mean << ",\t" << stdev << std::endl;
	}
	outfile.close();
	return;
}