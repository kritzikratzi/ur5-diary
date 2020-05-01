#include <iostream>
#include <opencv2/opencv.hpp>
#include "cv_util.h"
#include <args.hxx>
#include "detect_board.h"

using namespace cv;
using namespace std;

const string basedir = "/Users/hansi/Sync/projects/2020/comemak/code/test_9_gummi/obj_detect/images";

void detect(string filename);
void main_dummy();

int main(int argc, char * argv[]){
	return detect_board(argc, argv);
	args::ArgumentParser parser(argv[0], "YT Test");
	args::HelpFlag help(parser, "help", "Display help menu", { 'h', "help" });
	args::ValueFlag<string> queryFlag(parser, "query", "Query the database and display stuff", { "query" });
	args::Flag mtimeFlag(parser, "dummy", "Internal only ^^", {"dummy"});
	
	args::CompletionFlag completion(parser, { "complete" });
	try
	{
		parser.ParseCLI(argc, argv);
	}
	catch (const args::Completion& e)
	{
		std::cout << e.what();
		return 0;
	}
	catch (const args::Help&)
	{
		std::cout << parser;
		return 0;
	}
	catch (const args::ParseError& e)
	{
		std::cerr << e.what() << std::endl;
		std::cerr << parser;
		return 1;
	}
	catch (args::ValidationError e)
	{
		std::cerr << e.what() << std::endl;
		std::cerr << parser;
		return 1;
	}
	
	return detect_board(argc, argv);
}

void main_dummy(){
	vector<string> files = {
		"Foto am 11.04.20 um 03.00 #2 Kopie.jpg",
		"Foto am 11.04.20 um 03.00 #2.jpg",
		"Foto am 11.04.20 um 03.00.jpg",
		"Foto am 11.04.20 um 03.05 #2.jpg",
		"Foto am 11.04.20 um 03.05 #3.jpg",
		"Foto am 11.04.20 um 03.05 #4.jpg",
		"Foto am 11.04.20 um 03.05 #5.jpg",
		"Foto am 11.04.20 um 03.05.jpg",
		"Foto am 11.04.20 um 03.06 #2.jpg",
		"Foto am 11.04.20 um 03.06 #3.jpg",
		"Foto am 11.04.20 um 03.06.jpg"
	};
	for(auto & file : files ) detect(file);
}

void detect(string filename){
	// Read image
	string img_path = basedir + "/" + filename;
	Mat im = imread( img_path, IMREAD_GRAYSCALE );
	float alpha = 1.4;
	float beta = -0.5;
	im.convertTo(im, -1, alpha, beta);
	
	// Setup SimpleBlobDetector parameters.
	cv::SimpleBlobDetector::Params params;
	
	// Change thresholds
	params.minThreshold = 0;
	params.maxThreshold = 255;
	
	// Filter by Area.
	params.filterByArea = true;
	params.minArea = 1200;
	
	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = 0.2;
	
	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = 0.01;
	
	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = 0.0;
	
	// Set up the detector with default parameters.
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);

	// Detect blobs.
	std::vector<KeyPoint> keypoints;
	detector->detect( im, keypoints );
	
	// Draw detected blobs as red circles.
	// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
	Mat im_with_keypoints;
	drawKeypoints( im, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	

	int kidx = 0;
	for(auto & pt : keypoints){
		++kidx;
		
		int w = pt.size*1.5;
		cv::Size size(w,w);
		DrawRotatedRectangle(im_with_keypoints, pt.pt, size, 0);
		
		cv::Point roi_pt(max(0.0f,pt.pt.x-w/2),max(0.0f,pt.pt.y-w/2));
		cv::Rect rect(roi_pt, size);
		cv::Mat roi_grey = im(rect);
		cv::Mat roi;
		int thresh = 50;
		int maxValue = 255;
		threshold(roi_grey, roi, thresh, maxValue, cv::THRESH_BINARY + cv::THRESH_OTSU);
		
		vector<vector<cv::Point> > contours;
		vector<cv::Vec4i> hierarchy;
		cv::findContours( roi, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
		int best_contour = -1;
		float best_area = 0;
		for( size_t i = 0; i< contours.size(); i++ )
		{
			float area = contourArea(contours[i]);
			if(area>best_area && area<w*w*0.5){
				best_contour = i;
				best_area =area;
			}
		}
		
		if(best_contour>0){
			Scalar color = Scalar( 255,0,0 );
			drawContours( im_with_keypoints, contours, (int)best_contour, color, 2, LINE_8, hierarchy, 0,roi_pt );
			
			RotatedRect fit = cv::fitEllipse(contours[best_contour]);
			fit.center.x += roi_pt.x;
			fit.center.y += roi_pt.y;
			DrawRotatedRectangle(im_with_keypoints, fit.center, fit.size, fit.angle);
			
			cv::Mat extracted = MatFromRotatedRect(im,fit);
			imwrite(basedir + "/res/" + filename + to_string(kidx) + ".jpg", extracted);
		}
	}
	
	// Show blobs
	//imshow("keypoints", im_with_keypoints );
	imwrite(basedir + "/res/" + filename, im_with_keypoints);
	
}
