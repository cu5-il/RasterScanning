#include <iostream>
#include <fstream>
#include <iomanip>      // std::setw
#include <vector> 
#include <string>
#include <numeric> // std::accumulate

#include <deque>
#include <filesystem> // std::filesystem::copy, std::filesystem::create_directories
namespace fs = std::filesystem;

#include <opencv2/core.hpp>
#include "opencv2/core/utility.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/highgui.hpp>
#include "opencv2/core/utils/logger.hpp"

#include "A3200.h"

#include "constants.h"
#include "myTypes.h"
#include "myGlobals.h"
#include "scanning.h"
#include "draw.h"

#include "A3200_functions.h"
#include "thread_functions.h"
#include "raster.h"
#include <ctime>
#include "path.h"
#include "controlCalib.h"
#include "input.h"
#include "multiLayer.h"

std::string datetime(std::string format = "%Y.%m.%d-%H.%M.%S");

int main() {
	// Disable openCV warning in console
	cv::utils::logging::setLogLevel(cv::utils::logging::LogLevel::LOG_LEVEL_SILENT);
	// Create and set the output path
	outDir.append(datetime("%Y.%m.%d") + "/");
	try { fs::create_directories(outDir); }
	catch (std::exception& e) { std::cout << e.what(); }


	std::thread t_scan, t_process, t_control, t_print;

	// Initialize parameters
	Raster raster;
	double rasterBorder = 2; // INCREASED FROM 1 TO 2 WITH LATER PRINTS
	std::vector<std::vector<Path>> path, ctrlPath;

	// defining the material models
	MaterialModel augerModel = MaterialModel('a',
		std::vector<double>{2, 3},
		std::vector<double>{4.15, 3.95},
		std::vector<double>{0.1, 0.1},
		std::vector<double>{-2.815, -2.815});

	// setting up the controller
	//AugerController controller(augerModel, 0.2, 3.5);

	// setting the print options
	double leadin = 5;
	PrintOptions printOpts(leadin);
	printOpts.extrude = true;
	printOpts.disposal = false;
	printOpts.asyncTheta = 32;

	// Getting user input
	std::string datafile, infile;
	int lineNum;
	infile = "./Input/printTable.md";
	//infile = "./Input/2022.04.06/printTable.md";
	
	std::cout << "Enter Test #: ";
	std::cin >> lineNum;


	TableInput input(infile, lineNum);
	// copy the table to the output folder
	try { fs::copy(infile, outDir, fs::copy_options::overwrite_existing); }
	catch (std::exception& e) { std::cout << e.what(); }

	// Append the output directory with the print number and create the directory
	outDir.append("/print" + std::to_string(lineNum) + "/");
	try { fs::create_directories(outDir); }
	catch (std::exception& e) { std::cout << e.what(); }

	// make the scaffold
	raster = Raster(input.length, input.width, input.rodSpc, input.rodSpc - .1, rasterBorder);
	raster.offset(cv::Point2d(input.initPos.x, input.initPos.y));
	//MultiLayerScaffold scaffold(input, raster);
	FunGenScaf scaffold(input, raster,0, augerModel);

	path = scaffold.path;
	segments = scaffold.segments;

	// draw the segments
	cv::Mat imseg = raster.draw(input.startLayer);
	drawSegments(raster.draw(input.startLayer), imseg, segments, raster.origin(), input.startLayer, 3);
	cv::Mat image = cv::Mat::zeros(raster.size(segments.back().layer()), CV_8UC3);
	drawMaterialSegments(image, image, scaffold.segments, scaffold.path, scaffold.segments.back().layer());

	// drawing the referene
	cv::Mat ref = cv::Mat::zeros(raster.size(segments.back().layer()), CV_8UC1);
	drawMaterial(ref, ref, scaffold.segments, scaffold.path, scaffold.segments.back().layer());
	cv::cvtColor(ref, ref, cv::COLOR_BGR2GRAY);
	cv::threshold(ref, ref, 1, 255, cv::THRESH_BINARY);
	cv::flip(ref, ref, 0);
	cv::cvtColor(ref, ref, cv::COLOR_GRAY2BGR);
	cv::Mat refM;
	cv::Mat mkern = cv::getStructuringElement(cv::MORPH_ELLIPSE,cv::Size(9,9));
	cv::morphologyEx(ref, refM, cv::MORPH_CLOSE, mkern, cv::Point(-1, -1), 2);
	addScale(refM, 1, cv::Point(5, 25), 2);
	cv::imwrite(outDir + "reference.png", refM);

	// Making the images
	int ptSize = MM2PIX(0.18);
	int lnSize = ptSize;// MM2PIX(0.1);
	cv::Mat ptKern = cv::Mat::ones(ptSize, ptSize, CV_8UC1);
	cv::Mat wpMaskpx = cv::Mat::zeros(raster.size(), CV_8UC1);
	cv::Mat wpMask = cv::Mat::zeros(raster.size(), CV_8UC1);
	cv::Mat edgesBin = cv::Mat::zeros(raster.size(), CV_8UC1);
	cv::Mat outEdgesBin = cv::Mat::zeros(raster.size(), CV_8UC1);
	cv::Mat outEdgesMask = cv::Mat::zeros(raster.size(), CV_8UC1);
	cv::Mat inEdgesBin = cv::Mat::zeros(raster.size(), CV_8UC1);
	cv::Mat inEdgesMask = cv::Mat::zeros(raster.size(), CV_8UC1);
	cv::Mat matlFill = cv::Mat::zeros(raster.size(), CV_8UC1);
	cv::Mat cimage = cv::Mat::zeros(raster.size(), CV_8UC3);
	cv::Mat image2 = cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 255, 255));

	// Make a mask for the waypoints
	for (auto it = segments.begin(); it != segments.end(); ++it) {
		for (auto it2 = (*it).waypoints().begin(); it2 != (*it).waypoints().end(); ++it2) {
			wpMaskpx.at<uchar>(cv::Point2i(*it2)) = 255;
		}
	}
	cv::morphologyEx(wpMaskpx, wpMask, cv::MORPH_DILATE, ptKern, cv::Point(-1, -1), 1);

	cv::morphologyEx(wpMaskpx, wpMask, cv::MORPH_DILATE, cv::Mat::ones(3, 3, CV_8UC1), cv::Point(-1, -1), 1);

	std::cout << "Enter file to load ";
	//std::cin >> datafile;
	//datafile = "2022.04.06/scan5_edgedata.png";
	//datafile = "print6/print_edgedata_unfilt_1.png";
	datafile = "print5/scanF_edgedata_0.png";
	//datafile = "print3/scanF_edgedata_0.png";

	cv::Mat edges = cv::imread("./Input/" + datafile, cv::IMREAD_GRAYSCALE);

	edges.copyTo(edgesBin);
	cv::morphologyEx(edgesBin, edgesBin, cv::MORPH_DILATE, ptKern, cv::Point(-1, -1), 1);

	//image = cv::Mat::zeros(raster.size(segments.back().layer()), CV_8UC3);
	//raster.draw(image, image, segments.back().layer());
	//raster.drawBdry(image, image, segments.back().layer(), cv::Scalar(255, 0, 0), MM2PIX(0.05));
	//drawEdges(image, image, edges, cv::Scalar(0, 0, 255), MM2PIX(0.1));
	//cv::imwrite(outDir + "edges_" + ".png", image);

	// draw the material
	edgeMsg edgemsg;
	for (int i = 0; i < segments.size(); i++)
	{
		edgemsg.addEdges(edges, i, (i == segments.size() - 1));
		q_edgeMsg.push(edgemsg);
	}

	t_GetMatlErrors(raster, path);

	
	// draw inner and outer edges
	cv::Mat lredges = 255 * cv::Mat::ones(raster.size(), CV_8UC1);
	cv::cvtColor(lredges, lredges, cv::COLOR_GRAY2BGR);
	//int thick = MM2PIX(0.1);
	//thick = ptSize;
	raster.draw(lredges, lredges, 0, cv::Scalar(0, 0, 0), lnSize);

	cv::Scalar red(102, 85, 187);
	cv::Scalar blue(136, 68, 0);
	cv::Scalar yel(51, 170, 221);
	cv::Scalar lcolor, rcolor, wpcolor;
	lcolor = cv::Scalar(255,0,0);
	rcolor = cv::Scalar(0, 0, 255);
	wpcolor = cv::Scalar(0, 0, 0);

	//// Make a mask for the waypoints
	//cv::Mat ptKern = cv::Mat::ones(MM2PIX(0.1), MM2PIX(0.1), CV_8UC1);
	//cv::Mat wpMask = cv::Mat::zeros(raster.size(), CV_8UC1);
	//for (auto it = segments.begin(); it != segments.end(); ++it) {
	//	for (auto it2 = (*it).waypoints().begin(); it2 != (*it).waypoints().end(); ++it2) {
	//		wpMask.at<uchar>(cv::Point2i(*it2)) = 255;
	//	}
	//}
	//cv::morphologyEx(wpMask, wpMask, cv::MORPH_DILATE, ptKern, cv::Point(-1, -1), 1);

	//for (auto it = segments.begin(); it != segments.end(); ++it) {
	//	cv::polylines(lredges, (*it).lEdgePts(), false, lcolor, thick);
	//	cv::polylines(lredges, (*it).rEdgePts(), false, rcolor, thick);
	//	// draw the waypoints
	//	for (auto it2 = (*it).waypoints().begin(); it2 != (*it).waypoints().end(); ++it2) {
	//		cv::circle(lredges, *it2, thick, wpcolor, -1, cv::LINE_AA);
	//	}
	//}

	// Draw the inner and outer edges
	std::vector<cv::Point> inEdge, outEdge, allEdgePts, arc;
	cv::Size arcSz;
	int delta = 10;
	int arcDelta = 5;
	arcSz = cv::Size(cv::norm(segments.front().rEdgePts().front() - segments.front().lEdgePts().front()) / 4, cv::norm(segments.front().rEdgePts().front() - segments.front().lEdgePts().front()) / 2);
	//cv::ellipse(lredges, (segments.front().rEdgePts().front() + segments.front().lEdgePts().front()) / 2, arcSz, 0, 90, 180, blue, thick);
	//cv::ellipse(lredges, (segments.front().rEdgePts().front() + segments.front().lEdgePts().front()) / 2, arcSz, 0, 180, 270, red, thick);
	cv::ellipse2Poly((segments.front().rEdgePts().front() + segments.front().lEdgePts().front()) / 2, arcSz, 0, 180, 270, delta, arc);
	outEdge.insert(outEdge.end(), arc.begin(), arc.end());
	cv::ellipse2Poly((segments.front().rEdgePts().front() + segments.front().lEdgePts().front()) / 2, arcSz, 0, 90, 180, delta, arc);
	inEdge.insert(inEdge.end(), arc.rbegin(), arc.rend());
	for (int i = 0; i < segments.size(); i += 2) {
		//cv::polylines(lredges, segments[i].lEdgePts(), false, lcolor, thick);
		//cv::polylines(lredges, segments[i].rEdgePts(), false, rcolor, thick);
		if (i % 4 == 0) {
			if (i > 0) {
				arcSz = cv::Size((cv::norm(outEdge.back() - segments[i].lEdgePts().front()))/4, cv::norm(outEdge.back() - segments[i].lEdgePts().front())/2 );
				//cv::ellipse(lredges, (outEdge.back() + segments[i].lEdgePts().front()) / 2, arcSz, 0, 90, 270, blue, thick);
				cv::ellipse2Poly( (outEdge.back() + segments[i].lEdgePts().front()) / 2, arcSz, 0, 90+ arcDelta, 270- arcDelta, delta ,arc);
				outEdge.insert(outEdge.end(), arc.rbegin(), arc.rend());
				arcSz = cv::Size((cv::norm(inEdge.back() - segments[i].rEdgePts().front()))/4, cv::norm(inEdge.back() - segments[i].rEdgePts().front())/2 );
				//cv::ellipse(lredges, (inEdge.back() + segments[i].rEdgePts().front()) / 2, arcSz, 0, 90, 270, red, thick);
				cv::ellipse2Poly((inEdge.back() + segments[i].rEdgePts().front()) / 2, arcSz, 0, 90 + arcDelta, 270 - arcDelta, delta, arc);
				inEdge.insert(inEdge.end(), arc.rbegin(), arc.rend());
			}
			outEdge.insert(outEdge.end(), segments[i].lEdgePts().begin(), segments[i].lEdgePts().end());
			inEdge.insert(inEdge.end(), segments[i].rEdgePts().begin(), segments[i].rEdgePts().end());
		}
		else {
			arcSz = cv::Size(cv::norm(inEdge.back() - segments[i].lEdgePts().back())/4, cv::norm(inEdge.back() - segments[i].lEdgePts().back()) / 2);
			//cv::ellipse(lredges, (inEdge.back() + segments[i].lEdgePts().back()) / 2, arcSz, 180, 90, 270, yel, thick);
			cv::ellipse2Poly((inEdge.back() + segments[i].lEdgePts().back()) / 2, arcSz, 180, 90 + arcDelta, 270 - arcDelta, delta, arc);
			inEdge.insert(inEdge.end(), arc.begin(), arc.end());
			arcSz = cv::Size(cv::norm(outEdge.back() - segments[i].rEdgePts().back())/4, cv::norm(outEdge.back() - segments[i].rEdgePts().back()) / 2);
			//cv::ellipse(lredges, (outEdge.back() + segments[i].rEdgePts().back()) / 2, arcSz, 180, 90, 270, cv::Scalar(0, 255, 0), thick);
			cv::ellipse2Poly((outEdge.back() + segments[i].rEdgePts().back()) / 2, arcSz, 180, 90 + arcDelta, 270 - arcDelta, delta, arc);
			outEdge.insert(outEdge.end(), arc.begin(), arc.end());

			outEdge.insert(outEdge.end(), segments[i].rEdgePts().rbegin(), segments[i].rEdgePts().rend());
			inEdge.insert(inEdge.end(), segments[i].lEdgePts().rbegin(), segments[i].lEdgePts().rend());
		}
	}
	arcSz = cv::Size(cv::norm(segments.back().rEdgePts().front() - segments.back().lEdgePts().front()) / 4, cv::norm(segments.back().rEdgePts().front() - segments.back().lEdgePts().front()) / 2);
	//cv::ellipse(lredges, (segments.back().rEdgePts().front() + segments.back().lEdgePts().front()) / 2, arcSz, 0, 90, 180, blue, thick);
	//cv::ellipse(lredges, (segments.back().rEdgePts().front() + segments.back().lEdgePts().front()) / 2, arcSz, 0, 180, 270, red, thick);
	cv::ellipse2Poly((segments.back().rEdgePts().front() + segments.back().lEdgePts().front()) / 2, arcSz, 0, 90, 180, delta, arc);
	outEdge.insert(outEdge.end(), arc.begin(), arc.end());
	cv::ellipse2Poly((segments.back().rEdgePts().front() + segments.back().lEdgePts().front()) / 2, arcSz, 0, 180, 270, delta, arc);
	inEdge.insert(inEdge.end(), arc.rbegin(), arc.rend());

	cv::approxPolyDP(outEdge, outEdge, 2, false);
	cv::approxPolyDP(inEdge, inEdge, 2, false);
	cv::polylines(lredges, outEdge, false, lcolor, lnSize);
	cv::polylines(lredges, inEdge, false, rcolor, lnSize);
	//cv::polylines(lredges, outEdge, false, yel, thick);
	//cv::polylines(lredges, inEdge, false, red, thick);
	cv::imwrite(outDir + "LRedges" + ".png", lredges);


	// Make the masks
	cv::polylines(outEdgesMask, outEdge, false, cv::Scalar(255), lnSize);
	edges.copyTo(outEdgesBin, outEdgesMask);
	cv::morphologyEx(outEdgesBin, outEdgesBin, cv::MORPH_DILATE, ptKern, cv::Point(-1, -1), 1);
	cv::polylines(inEdgesMask, inEdge, false, cv::Scalar(255), lnSize);
	edges.copyTo(inEdgesBin, inEdgesMask);
	cv::morphologyEx(inEdgesBin, inEdgesBin, cv::MORPH_DILATE, ptKern, cv::Point(-1, -1), 1);
	allEdgePts.insert(allEdgePts.end(), outEdge.begin(), outEdge.end());
	allEdgePts.insert(allEdgePts.end(), inEdge.rbegin(), inEdge.rend());
	cv::fillPoly(matlFill, allEdgePts, cv::Scalar(255));

	// Make the Figures
	cv::imwrite(outDir + "matlMask" + ".png", matlFill);

	// Binary edge points with a path centerline
	image = cv::Mat::zeros(raster.size(), CV_8UC3);
	cv::imwrite(outDir + "edgePtsBin" + ".png", edgesBin);
	cv::cvtColor(edgesBin, image, cv::COLOR_GRAY2BGR);
	raster.draw(image, image, 0, cv::Scalar(0, 255, 0), lnSize/4);
	cv::imwrite(outDir + "edgePtsBin_cl" + ".png", image);

	// Separated edge points with a path centerline
	image = cv::Mat::zeros(raster.size(), CV_8UC3);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 0, 255)).copyTo(image, outEdgesBin);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 255, 0)).copyTo(image, inEdgesBin);
	cv::imwrite(outDir + "edgePtsSep" + ".png", image);
	raster.draw(image, image, 0, cv::Scalar(0, 255, 0), lnSize/4);
	cv::imwrite(outDir + "edgePtsSep_cl" + ".png", image);

	// Separated connected edges with a SOLID path centerline
	image = cv::Mat::zeros(raster.size(), CV_8UC3);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 0, 255)).copyTo(image, outEdgesMask);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 255, 0)).copyTo(image, inEdgesMask);
	cv::imwrite(outDir + "edgesSep" + ".png", image);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(0, 255, 0)).copyTo(image, wpMask);
	cv::imwrite(outDir + "edgesSep_cl" + ".png", image);

	// Separated connected edges with a FILL
	cv::cvtColor(matlFill, image, cv::COLOR_GRAY2BGR);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 0, 255)).copyTo(image, outEdgesMask);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 255, 0)).copyTo(image, inEdgesMask);
	cv::imwrite(outDir + "edgesSepFill" + ".png", image);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(0, 255, 0)).copyTo(image, wpMask);
	cv::imwrite(outDir + "edgesSepFill_cl" + ".png", image);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(0, 0, 0)).copyTo(image, wpMask);
	cv::imwrite(outDir + "edgesSepFill_cl2" + ".png", image);
	// Very thin versions
	cv::Mat thinMask = cv::Mat::zeros(raster.size(), CV_8UC1);
	cv::cvtColor(matlFill, image, cv::COLOR_GRAY2BGR);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(0, 0, 0)).copyTo(image, wpMaskpx);
	cv::polylines(thinMask, outEdge, false, cv::Scalar(255), 1);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 0, 255)).copyTo(image, thinMask);
	thinMask = cv::Mat::zeros(raster.size(), CV_8UC1);
	cv::polylines(thinMask, inEdge, false, cv::Scalar(255), 1);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 255, 0)).copyTo(image, thinMask);
	cv::imwrite(outDir + "edgesSepFill_thin1" + ".png", image);
	// Less thin versions
	cv::cvtColor(matlFill, image, cv::COLOR_GRAY2BGR);
	thinMask = cv::Mat::zeros(raster.size(), CV_8UC1);
	cv::morphologyEx(wpMaskpx, thinMask, cv::MORPH_DILATE, cv::Mat::ones(3, 3, CV_8UC1), cv::Point(-1, -1), 1);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(0, 0, 0)).copyTo(image, thinMask);
	thinMask = cv::Mat::zeros(raster.size(), CV_8UC1);
	cv::polylines(thinMask, outEdge, false, cv::Scalar(255), 3);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 0, 255)).copyTo(image, thinMask);
	thinMask = cv::Mat::zeros(raster.size(), CV_8UC1);
	cv::polylines(thinMask, inEdge, false, cv::Scalar(255), 3);
	cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 255, 0)).copyTo(image, thinMask);
	cv::imwrite(outDir + "edgesSepFill_thin3" + ".png", image);

	// Distance transforms
	cv::Mat Din = cv::Mat::zeros(raster.size(), CV_32FC1);
	cv::Mat Dout = cv::Mat::zeros(raster.size(), CV_32FC1);
	cv::Mat Dimage= cv::Mat::zeros(raster.size(), CV_32FC1);
	cv::Mat invMask = cv::Mat::zeros(raster.size(), CV_8UC1);
	//cv::bitwise_not(outEdgesMask, invMask);
	//cv::distanceTransform(invMask, Dout, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
	//cv::bitwise_not(inEdgesMask, invMask);
	//cv::distanceTransform(invMask, Din, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
	cv::Mat DxMask = 255*cv::Mat::ones(raster.size(), CV_8UC1);
	cv::polylines(DxMask, outEdge, false, cv::Scalar(0), 1);
	cv::distanceTransform(DxMask, Dout, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
	DxMask = 255 * cv::Mat::ones(raster.size(), CV_8UC1);
	cv::polylines(DxMask, inEdge, false, cv::Scalar(0), 1);
	cv::distanceTransform(DxMask, Din, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);

	// outer
	image = cv::Mat::zeros(raster.size(), CV_8UC3);
	Dout.copyTo(Dimage, matlFill);
	cv::normalize(Dimage, image, 0, 255, cv::NORM_MINMAX, CV_8U, matlFill);
	cv::imwrite(outDir + "D_outG" + ".png", image); // grayscale
	image2 = cv::Mat::zeros(raster.size(), CV_8UC3);
	cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
	cv::bitwise_and(cv::Scalar(255, 0, 255), image, image2, matlFill);
	cv::imwrite(outDir + "D_outGC" + ".png", image2); // "magenta"scale
	//cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 0, 255)).copyTo(image, outEdgesMask);
	cv::polylines(image, outEdge, false, cv::Scalar(255, 0, 255), lnSize);
	cv::imwrite(outDir + "D_outE1" + ".png", image); // grayscale with ONE colored material line
	cv::polylines(image, inEdge, false, cv::Scalar(255, 255, 0), lnSize);
	cv::imwrite(outDir + "D_outE2" + ".png", image); // grayscale with TWO colored material line

	cimage = cv::Mat::zeros(raster.size(), CV_8UC3);
	cv::applyColorMap(image, cimage, cv::COLORMAP_VIRIDIS);
	cimage.copyTo(image2, matlFill);
	cv::imwrite(outDir + "D_outV" + ".png", image2);
	image2 = cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 255, 255));
	cimage = cv::Mat::zeros(raster.size(), CV_8UC3);
	cv::applyColorMap(image, cimage, cv::COLORMAP_CIVIDIS);
	cimage.copyTo(image2, matlFill);
	cv::imwrite(outDir + "D_outC" + ".png", image2);

	// inner
	Dimage = cv::Mat::zeros(raster.size(), CV_32FC1);
	image = cv::Mat::zeros(raster.size(), CV_8UC3);
	Din.copyTo(Dimage, matlFill);
	cv::normalize(Dimage, image, 0, 255, cv::NORM_MINMAX, CV_8U, matlFill);
	cv::imwrite(outDir + "D_inG" + ".png", image);// grayscale
	image2 = cv::Mat::zeros(raster.size(), CV_8UC3);
	cv::cvtColor(image, image, cv::COLOR_GRAY2BGR);
	cv::bitwise_and(cv::Scalar(255, 255, 0), image, image2, matlFill);
	cv::imwrite(outDir + "D_inGC" + ".png", image2); // "cyan"scale
	//cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 255, 0)).copyTo(image, inEdgesMask);
	cv::polylines(image, inEdge, false, cv::Scalar(255, 255, 0), lnSize);
	cv::imwrite(outDir + "D_inE1" + ".png", image); // grayscale with ONE colored material line
	cv::polylines(image, outEdge, false, cv::Scalar(255, 0, 255), lnSize);
	cv::imwrite(outDir + "D_inE2" + ".png", image); // grayscale with TWO colored material line
	

	cimage = cv::Mat::zeros(raster.size(), CV_8UC3);
	cv::applyColorMap(image, cimage, cv::COLORMAP_VIRIDIS);
	cimage.copyTo(image2, matlFill);
	cv::imwrite(outDir + "D_inV" + ".png", image2);
	image2 = cv::Mat(raster.size(), CV_8UC3, cv::Scalar(255, 255, 255));
	cimage = cv::Mat::zeros(raster.size(), CV_8UC3);
	cv::applyColorMap(image, cimage, cv::COLORMAP_CIVIDIS);
	cimage.copyTo(image2, matlFill);
	cv::imwrite(outDir + "D_inC" + ".png", image2);
	

	//cv::normalize(Dout, Dout, 0, 1, cv::NORM_MINMAX, -1);
	//cv::normalize(Din, Din, 0, 1, cv::NORM_MINMAX, -1);
	//cv::imwrite(outDir + "D_out" + ".tiff", Dout);
	//cv::imwrite(outDir + "D_in" + ".tiff", Din);

	//// Centerline stuff
	//cv::Mat Ddiff = cv::Mat::zeros(raster.size(), CV_32FC1);
	//cv::Mat Dsum = cv::Mat::zeros(raster.size(), CV_32FC1);
	//cv::Mat Dcl = cv::Mat::zeros(raster.size(), CV_32FC1);
	//cv::distanceTransform(matlFill, Dcl, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32F);
	//Ddiff = Dout - Din;
	//Dsum = Dout + Din;
	//cv::normalize(Dcl, Dcl, 0, 1, cv::NORM_MINMAX, -1);
	//cv::imwrite(outDir + "D_cl" + ".tiff", Dcl);

	// Opening a file to save the results
	std::ofstream outfile;
	outfile.open(std::string(outDir + "InOutEdge.txt").c_str());
	outfile.precision(3);
	//outfile << "segment\tXin\tYin\tXout\tYout\n";// header
	outfile << "segment\t X\t Y\t InOut\n"; // header
	bool InOut = 0;
	// loop through each long segment
	for (int i = 0; i < segments.size(); i += 2) {
		if (i % 4 == 0) {
			InOut = 1;
		}
		else {
			InOut = 0;
		}
		// loop through all the waypoints
		for (int j = 0; j < segments[i].rEdgePts().size(); j++) {
			outfile << std::setw(3) << std::fixed << i << "\t";// segment number
			//outfile << std::setw(4) << std::fixed << segments[i].rEdgePts()[j].x << "\t" << std::setw(4) << std::fixed << segments[i].rEdgePts()[j].y << "\t" << InOut <<"\n";
			outfile << segments[i].rEdgePts()[j].x << "\t" << segments[i].rEdgePts()[j].y << "\t" << InOut << "\n";
		}
		outfile << std::setw(3) << std::fixed << i << "\t" << "nan" << "\t" << "nan"  << "\t" << InOut << "\n";// break point

		for (int j = 0; j < segments[i].lEdgePts().size(); j++) {
			outfile << std::setw(3) << std::fixed << i << "\t";// segment number
			outfile << segments[i].lEdgePts()[j].x << "\t" << segments[i].lEdgePts()[j].y << "\t" << !InOut << "\n";
		}
		outfile << std::setw(3) << std::fixed << i << "\t" << "nan" << "\t" << "nan" << "\t" << !InOut << "\n";// break point
	}
	outfile.close();

	//cv::approxPolyDP(outEdge,outEdge,1,false);
	outfile.open(std::string(outDir + "OutEdge2.txt").c_str());
	for (int i = 0; i < outEdge.size(); i++) {
		outfile << outEdge[i].x << "\t" << outEdge[i].y << "\n";
	}
	outfile.close();

	outfile.open(std::string(outDir + "InEdge2.txt").c_str());
	for (int i = 0; i < inEdge.size(); i++) {
		outfile << inEdge[i].x << "\t" << inEdge[i].y << "\n";
	}
	outfile.close();

	outfile.open(std::string(outDir + "Waypoints.txt").c_str());
	for (auto it = segments.begin(); it != segments.end(); ++it) {
		for (auto it2 = (*it).waypoints().begin(); it2 != (*it).waypoints().end(); ++it2) {
			outfile << (*it2).x << "\t" << (*it2).y << "\n";
		}
	}
	outfile.close();
cleanup:
	//A3200 Cleanup
	//=======================================
	// Freeing the resources used by the data collection configuration
	if (NULL != DCCHandle) {
		if (!A3200DataCollectionConfigFree(DCCHandle)) { A3200Error(); }
	}
	// Disconnecting from the A3200
	if (NULL != handle) {
		printf("Disconnecting from the A3200.\n");
		if (!A3200Disconnect(handle)) { A3200Error(); }
	}

#ifdef _DEBUG

#endif
	return 0;
}

cv::Mat translateImg(cv::Mat& img, int offsetx, int offsety) {
	cv::Mat trans_mat = (cv::Mat_<double>(2, 3) << 1, 0, offsetx, 0, 1, offsety);
	cv::warpAffine(img, img, trans_mat, img.size());
	return img;
}

std::string datetime(std::string format) {
	time_t rawtime = time(NULL);
	struct tm timeinfo;
	char buffer[80];

	localtime_s(&timeinfo, &rawtime);
	strftime(buffer, 80, format.c_str(), &timeinfo);
	return std::string(buffer);
}