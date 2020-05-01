#include "detect_board.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco/charuco.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>
#include <iostream>
#include "cv_util.h"
#include <thread>
#include <httplib.h>

using namespace std;
using namespace cv;

cv::Mat computeProjMat(cv::Mat camMat, cv::Vec3d rotVec, cv::Vec3d transVec);

namespace {
	const char* about = "Pose estimation using a ChArUco board";
	const char* keys  =
	"{w        |       | Number of squares in X direction }"
	"{h        |       | Number of squares in Y direction }"
	"{sl       |       | Square side length (in meters) }"
	"{ml       |       | Marker side length (in meters) }"
	"{d        |       | dictionary: DICT_4X4_50=0, DICT_4X4_100=1, DICT_4X4_250=2,"
	"DICT_4X4_1000=3, DICT_5X5_50=4, DICT_5X5_100=5, DICT_5X5_250=6, DICT_5X5_1000=7, "
	"DICT_6X6_50=8, DICT_6X6_100=9, DICT_6X6_250=10, DICT_6X6_1000=11, DICT_7X7_50=12,"
	"DICT_7X7_100=13, DICT_7X7_250=14, DICT_7X7_1000=15, DICT_ARUCO_ORIGINAL = 16}"
	"{c        |       | Output file with calibrated camera parameters }"
	"{v        |       | Input from video file, if ommited, input comes from camera }"
	"{ci       | 0     | Camera id if input doesnt come from video (-v) }"
	"{dp       |       | File of marker detector parameters }"
	"{rs       |       | Apply refind strategy }"
	"{r        |       | show rejected candidates too }";
}

/**
 */
static bool readCameraParameters(string filename, Mat &camMatrix, Mat &distCoeffs) {
	FileStorage fs(filename, FileStorage::READ);
	auto node = fs["camera_matrix"];
	if(!fs.isOpened())
		return false;
	fs["cameraMatrix"] >> camMatrix;
	fs["dist_coeffs"] >> distCoeffs;
	return true;
}


/**
 */
static bool readDetectorParameters(string filename, Ptr<aruco::DetectorParameters> &params) {
	FileStorage fs(filename, FileStorage::READ);
	if(!fs.isOpened())
		return false;
	fs["adaptiveThreshWinSizeMin"] >> params->adaptiveThreshWinSizeMin;
	fs["adaptiveThreshWinSizeMax"] >> params->adaptiveThreshWinSizeMax;
	fs["adaptiveThreshWinSizeStep"] >> params->adaptiveThreshWinSizeStep;
	fs["adaptiveThreshConstant"] >> params->adaptiveThreshConstant;
	fs["minMarkerPerimeterRate"] >> params->minMarkerPerimeterRate;
	fs["maxMarkerPerimeterRate"] >> params->maxMarkerPerimeterRate;
	fs["polygonalApproxAccuracyRate"] >> params->polygonalApproxAccuracyRate;
	fs["minCornerDistanceRate"] >> params->minCornerDistanceRate;
	fs["minDistanceToBorder"] >> params->minDistanceToBorder;
	fs["minMarkerDistanceRate"] >> params->minMarkerDistanceRate;
	fs["cornerRefinementMethod"] >> params->cornerRefinementMethod;
	fs["cornerRefinementWinSize"] >> params->cornerRefinementWinSize;
	fs["cornerRefinementMaxIterations"] >> params->cornerRefinementMaxIterations;
	fs["cornerRefinementMinAccuracy"] >> params->cornerRefinementMinAccuracy;
	fs["markerBorderBits"] >> params->markerBorderBits;
	fs["perspectiveRemovePixelPerCell"] >> params->perspectiveRemovePixelPerCell;
	fs["perspectiveRemoveIgnoredMarginPerCell"] >> params->perspectiveRemoveIgnoredMarginPerCell;
	fs["maxErroneousBitsInBorderRate"] >> params->maxErroneousBitsInBorderRate;
	fs["minOtsuStdDev"] >> params->minOtsuStdDev;
	fs["errorCorrectionRate"] >> params->errorCorrectionRate;
	return true;
}

/**
 */
int detect_board(int argc, char *argv[]) {
	httplib::Server svr;
	mutex final_result_mutex;
	vector<cv::Vec6d> final_result;
	svr.Get("/bears", [&](const httplib::Request& req, httplib::Response& res){
		lock_guard<mutex> guard{final_result_mutex};
		stringstream ss;
		ss << "{\"points\":[\n";
		for(int i = 0; i < final_result.size(); i++){
			auto & pt = final_result[i];
			if(i!=0) ss << ",\n";
			ss << "  {";
			ss << "\"x\":" << pt(0) << ",";
			ss << "\"y\":" << pt(1) << ",";
			ss << "\"z\":" << pt(2) << ",";
			ss << "\"rx\":" << pt(3) << ",";
			ss << "\"ry\":" << pt(4) << ",";
			ss << "\"rz\":" << pt(5);
			ss << "}";
		}
		ss << "\n]}";
		res.set_content(ss.str(),"application/json");
	});
	thread svr_threaed([&](){
		svr.listen("localhost",8081);
	});
	
	CommandLineParser parser(argc, argv, keys);
	parser.about(about);
	
	/*if(argc < 6) {
		parser.printMessage();
		return 0;
	}*/
	int squaresX = parser.get<int>("w");
	int squaresY = parser.get<int>("h");
	float squareLength = parser.get<float>("sl");
	float markerLength = parser.get<float>("ml");
	int dictionaryId = parser.get<int>("d");
	bool showRejected = parser.has("r");
	bool refindStrategy = parser.has("rs");
	int camId = parser.get<int>("ci");
	
	camId = 1;
	squaresX = 11;
	squaresY = 17;
	squareLength = 0.015;
	markerLength = 0.011;
	dictionaryId = 0;
	showRejected = true;

	squaresX = 5;
	squaresY = 5;
	squareLength = 0.030;
	markerLength = 0.024;
	dictionaryId = 0;

	
	String video;
	if(parser.has("v")) {
		video = parser.get<String>("v");
	}
	video = "http://192.168.0.18:8080/video";
	httplib::Client cli("192.168.0.18",8080);
	cli.Get("/settings/quality?set=100");


	Mat camMatrix, distCoeffs;
	string pfile = "/Users/hansi/Sync/projects/2020/comemak/code/test_9_gummi/obj_detect/build/Debug/cameraParameters.xml";
	bool readOk = readCameraParameters(pfile, camMatrix, distCoeffs);
	Mat camMatrix_inv = camMatrix.inv();
	if(!readOk) {
		cerr << "Invalid camera file" << endl;
		return 0;
	}
	
	Ptr<aruco::DetectorParameters> detectorParams = aruco::DetectorParameters::create();
	if(parser.has("dp")) {
		bool readOk = readDetectorParameters(parser.get<string>("dp"), detectorParams);
		if(!readOk) {
			cerr << "Invalid detector parameters file" << endl;
			return 0;
		}
	}
	/*
	if(!parser.check()) {
		parser.printErrors();
		return 0;
	}*/
	
	Ptr<aruco::Dictionary> dictionary =
	aruco::getPredefinedDictionary(aruco::PREDEFINED_DICTIONARY_NAME(dictionaryId));
	
	VideoCapture inputVideo;
	int waitTime;
	if(!video.empty()) {
		inputVideo.open(video);
		waitTime = 10;
	} else {
		inputVideo.open(camId);
		waitTime = 10;
	}
	
	float axisLength = 0.5f * ((float)min(squaresX, squaresY) * (squareLength));
	
	// create charuco board object
	Ptr<aruco::CharucoBoard> charucoboard =
	aruco::CharucoBoard::create(squaresX, squaresY, squareLength, markerLength, dictionary);
	Ptr<aruco::Board> board = charucoboard.staticCast<aruco::Board>();
	
	double totalTime = 0;
	int totalIterations = 0;
	
	
	int slider_alpha = 100;
	int slider_beta = 100;
	int slider_minsize = 700;
	int slider_minThreshold = 0;
	int slider_maxThreshold = 255;
	int slider_minCircularity = 0.2*100;
	int slider_minConvexity = 0.01*100;
	int slider_minInertia = 0.0*100;

	while(inputVideo.grab()) {
		Mat image, imageCopy;
		inputVideo.retrieve(image);
		
		double tick = (double)getTickCount();
		
		vector< int > markerIds, charucoIds;
		vector< vector< Point2f > > markerCorners, rejectedMarkers;
		vector< Point2f > charucoCorners;
		Vec3d rvec, tvec;
		
		// detect markers
		aruco::detectMarkers(image, dictionary, markerCorners, markerIds, detectorParams,
							 rejectedMarkers);
		
		// refind strategy to detect more markers
		if(refindStrategy)
			aruco::refineDetectedMarkers(image, board, markerCorners, markerIds, rejectedMarkers,
										 camMatrix, distCoeffs);
		
		// interpolate charuco corners
		int interpolatedCorners = 0;
		if(markerIds.size() > 0)
			interpolatedCorners =
			aruco::interpolateCornersCharuco(markerCorners, markerIds, image, charucoboard,
											 charucoCorners, charucoIds, camMatrix, distCoeffs);
		
		// estimate charuco board pose
		bool validPose = false;
		if(camMatrix.total() != 0)
			validPose = aruco::estimatePoseCharucoBoard(charucoCorners, charucoIds, charucoboard,
														camMatrix, distCoeffs, rvec, tvec);
		
		
		cout << "valid=" << validPose << endl;
		double currentTime = ((double)getTickCount() - tick) / getTickFrequency();
		totalTime += currentTime;
		totalIterations++;
		if(totalIterations % 30 == 0) {
			cout << "Detection Time = " << currentTime * 1000 << " ms "
			<< "(Mean = " << 1000 * totalTime / double(totalIterations) << " ms)" << endl;
		}
		
		
		
		// draw results
		image.copyTo(imageCopy);
		Mat imagePaper;
		if(markerIds.size() > 0) {
			aruco::drawDetectedMarkers(imageCopy, markerCorners);
		}
		
		if(showRejected && rejectedMarkers.size() > 0)
			aruco::drawDetectedMarkers(imageCopy, rejectedMarkers, noArray(), Scalar(100, 0, 255));
		
		if(interpolatedCorners > 0) {
			Scalar color;
			color = Scalar(255, 0, 0);
			aruco::drawDetectedCornersCharuco(imageCopy, charucoCorners, charucoIds, color);
		}
		
		if(validPose){
			aruco::drawAxis(imageCopy, camMatrix, distCoeffs, rvec, tvec, axisLength);
			
			
			vector<Point3d> paper_pts;
			double pad = 0.00;
			paper_pts.push_back(Point3d(0.155+pad,-0.077+pad,0));
			paper_pts.push_back(Point3d(0.155+0.21-2*pad,-0.077+pad,0));
			paper_pts.push_back(Point3d(0.155+0.21-2*pad,0.15+0.07-pad,0));
			paper_pts.push_back(Point3d(0.155+pad,0.15+0.07-pad,0));
			vector<Point2d> imagePoints;
			projectPoints(paper_pts, rvec, tvec, camMatrix, distCoeffs, imagePoints);
			
			cv::Mat rotationMatrix;
			cv::Rodrigues(rvec,rotationMatrix);
			cv::Mat rotationMatrix_inv = rotationMatrix.inv(); 
			
			double z0 = 0;
			double zhi = 0.008; // haribo is 8mm high
			cv::Mat uvPoint = (cv::Mat_<double>(3,1) << 242, 12, 1); // u = 363, v = 222, got this point using mouse callback
			
			cv::Mat leftSideMat  = rotationMatrix_inv * camMatrix_inv * uvPoint;
			cv::Mat rightSideMat = rotationMatrix_inv * tvec;
			double s = (z0 + rightSideMat.at<double>(2,0))/leftSideMat.at<double>(2,0);

			cv::Vec3d P = cv::Mat(rotationMatrix_inv* (s * camMatrix_inv* uvPoint - cv::Mat(tvec)));
			std::cout << "P = " << P << std::endl;
			
			
			
			/*cv::Point3d ra(-91, -505, 0.);
			cv::Point3d rb(59,-513,0.);
			cv::Point3d rc(65.6,-365,0.);
			cv::Point3d rd(-84.,-359,0.);*/
			cv::Point3d ra(-0.092, -0.509, 0);
			cv::Point3d rb(0.058,-0.514,0);
			cv::Point3d rc(0.0635,-0.364,0);
			cv::Point3d rd(-0.0865,-0.359,0);

			
			cout << "go to: " << (ra + (rb-ra)/0.15*P(0) + (rd-ra)/0.15*P(1))*1000 << " mm" << endl;
			
			int clean_w = (210-2*pad*1000)*2;
			int clean_h = (297-2*pad*1000)*2;
			vector<Point2f> pp;
			for(auto & p : imagePoints) pp.push_back(Point2f(p.x,p.y));
			vector<Point2f> clean_pts = {
				Point2f(0,clean_h),
				Point2f(clean_w,clean_h),
				Point2f(clean_w,0),
				Point2f(0,0),
			};
			Mat paper_mat = cv::getPerspectiveTransform(pp, clean_pts);
			Mat paper_mat_inv = paper_mat.inv();
			
			cout << (paper_mat*cv::Vec3d(0,0,1)) << endl;
			cv::warpPerspective(imageCopy, imagePaper, paper_mat, Size(clean_w,clean_h), INTER_CUBIC, BORDER_CONSTANT);
			cv::Mat temp;
			GummiAlgoOptions opts;
			opts.alpha = slider_alpha/100.0;
			opts.beta = (slider_beta-100)/100.0*100.0;
			opts.minArea = slider_minsize;
			opts.minThreshold = slider_minThreshold;
			opts.maxThreshold = slider_maxThreshold;
			opts.minCircularity = slider_minCircularity/100.0;
			opts.minConvexity = slider_minConvexity/100.0;
			opts.minInertia = slider_minInertia/100.0;

			auto bears = gummiAlgo(imagePaper, temp, opts);
			vector<cv::Vec6d> bears_rob;
			imagePaper = temp;

			auto normalize = [](cv::Vec3d & v){ v(0) /= v(2); v(1)/= v(2); v(2) = 1; };
			for(auto & bear : bears){
				cv::Vec3d center_px = Mat( paper_mat_inv*cv::Vec3d(bear.center.x,bear.center.y,1));
				normalize(center_px);
				cv::circle(imageCopy, Point(center_px(0),center_px(1)), 10, Scalar(255, 0, 0), 2);

				cv::Mat leftSideMat  = rotationMatrix_inv * camMatrix_inv * center_px;
				cv::Mat rightSideMat = rotationMatrix_inv * tvec;
				double s = (z0 + rightSideMat.at<double>(2,0))/leftSideMat.at<double>(2,0);

				cv::Vec3d center_marker = cv::Mat(rotationMatrix_inv*(s*camMatrix_inv*center_px - cv::Mat(tvec)));
				
				cv::Vec3d center_rob = Mat(ra + (rb-ra)/0.15*center_marker(0) + (rd-ra)/0.15*center_marker(1));
				cv::Vec6d pos_rob(center_rob(0),center_rob(1),center_rob(2),0,0,bear.angle*M_PI/180);
				cout << "use robot coordinates: " << center_rob << " m" << endl;
				bears_rob.push_back(pos_rob);
			}
			std::sort(bears_rob.begin(),bears_rob.end(),[](const cv::Vec6d & a, const cv::Vec6d & b){
				return a(1) < b(1);
			});
			{
				lock_guard<mutex> guard{final_result_mutex};
				final_result = bears_rob;
			}
			//cv::Vec3d P = cv::Mat(rotationMatrix.inv() * (s * camMatrix.inv() * uvPoint - cv::Mat(tvec)));

			
			// mark paper
			for(auto & pp : imagePoints){
				// draw axes lines
				cv::circle(imageCopy, pp, 10, Scalar(255, 0, 0), 2);
			}
			

			/*
			cv::Mat paper_zero = cv::Mat::eye(4, 4, CV_64F);
			cv::Mat paper_zero_inv = cv::Mat::zeros(4, 4, CV_64F);
			//camMatrix.copyTo(paper_zero);
			camMatrix.copyTo(paper_zero(cv::Rect(0,0,3,3)));
			
			cv::Mat robot_zero = cv::Mat::eye(4, 4, CV_64F);
			cv::Mat robot_zero_inv = cv::Mat::zeros(4, 4, CV_64F);
			
			// 0.0966,-0.3369, 0.0064, 0,0,0
			robot_zero.at<double>(0,3) = 0.0966;
			robot_zero.at<double>(1,3) = -0.3369;
			robot_zero.at<double>(2,3) = 0.0064;
			
			cv::invert(robot_zero, robot_zero_inv);
			cv::invert(paper_zero, paper_zero_inv);
			
			cv::Mat proj = computeProjMat(camMatrix, rvec, tvec);
			cv::Mat mproj = cv::Mat::eye(4, 4, CV_64F);
			cv::Mat mproj_inv;
			invert(mproj, mproj_inv);
			
			proj.copyTo(mproj(cv::Rect(0,0,4,3)));
			
			cv::Mat px = mproj*cv::Vec4d(0.15+0.21,0,0,1);
			cout << "px=" << px << endl;
			
			cv::Mat pt = proj*cv::Vec4d(0,0,0,1);

			cout << "pt=" << pt << endl;

			double cx = px.at<double>(0);
			double cy = px.at<double>(1);
			cout << "  -> " << cx << ", " << cy << endl;
			
			cv::Point2d center(cx, cy);
			cv::circle(imageCopy, center, 10, Scalar(0, 255, 0),4);

			cv::Mat res = robot_zero * paper_zero_inv * cv::Vec4d(0,100,0,1);
			cout << res << endl;
			cout << "---" << endl;*/
		}
		else{
			{
				lock_guard<mutex> guard{final_result_mutex};
				final_result.clear();
			}
		}
		
		namedWindow("paper", WINDOW_AUTOSIZE); // Create Window
		cv::createTrackbar("detect.alpha", "paper", &slider_alpha, 200);
		cv::createTrackbar("detect.beta", "paper", &slider_beta, 200);
		cv::createTrackbar("detect.minsize", "paper", &slider_minsize, 3000);
		cv::createTrackbar("detect.minThreshold", "paper", &slider_minThreshold, 255);
		cv::createTrackbar("detect.maxThreshold", "paper", &slider_maxThreshold, 255);
		cv::createTrackbar("detect.minCircularity", "paper", &slider_minCircularity, 100);
		cv::createTrackbar("detect.minConvexity", "paper", &slider_minConvexity, 100);
		cv::createTrackbar("detect.minInertia", "paper", &slider_minInertia, 100);

		
		cv::imshow("out", imageCopy);
		if(imagePaper.rows>0 && imagePaper.cols>0){
			cv::imshow("paper", imagePaper);
		}
		char key = (char)waitKey(waitTime);
		if(key == 27) break;
		
		std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	
	
	svr.stop();
	svr_threaed.join();
	
	return 0;
}


cv::Mat computeProjMat(cv::Mat camMat, cv::Vec3d rotVec, cv::Vec3d transVec)
{
	cv::Mat rotMat(3, 3, CV_64F), rotTransMat(3, 4, CV_64F); //Init.
	//Convert rotation vector into rotation matrix
	cv::Rodrigues(rotVec, rotMat);
	//Append translation vector to rotation matrix
	cv::hconcat(rotMat, transVec, rotTransMat);
	//Compute projection matrix by multiplying intrinsic parameter
	//matrix (A) with 3 x 4 rotation and translation pose matrix (RT).
	//Formula: Projection Matrix = A * RT;
	return (camMat * rotTransMat);
}
