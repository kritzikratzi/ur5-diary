#include "cv_util.h"
#include <utility>
#include <opencv2/imgproc/imgproc.hpp>
// Include center point of your rectangle, size of your rectangle and the degrees of rotation
void DrawRotatedRectangle(cv::Mat& image, cv::Point centerPoint, cv::Size rectangleSize, double rotationDegrees)
{
	cv::Scalar color = cv::Scalar(0,0,0); // white
	cv::Scalar green = cv::Scalar(0,255,0,10); // white

	// Create the rotated rectangle
	cv::RotatedRect rotatedRectangle(centerPoint, rectangleSize, rotationDegrees);
	
	// We take the edges that OpenCV calculated for us
	cv::Point2f vertices2f[4];
	rotatedRectangle.points(vertices2f);
	
	// Convert them so we can use them in a fillConvexPoly
	cv::Point vertices[4];
	for(int i = 0; i < 4; ++i){
		vertices[i] = vertices2f[i];
	}
	
	// Now we can fill the rotated rectangle with our specified color
	int npts = 4;
	cv::Point * draw_verts[1] = {(cv::Point*)vertices};
	cv::polylines(image, draw_verts, &npts, 1, true, green);
}


cv::Mat MatFromRotatedRect(cv::Mat& src, const cv::RotatedRect & rect){
	cv::Mat M,rotated, cropped;
	// get angle and size from the bounding box
	float angle = rect.angle;
	cv::Size rect_size = rect.size;
	// thanks to http://felix.abecassis.me/2011/10/opencv-rotation-deskewing/
	if (rect.angle < -45.) {
		angle += 90.0;
		std::swap(rect_size.width, rect_size.height);
	}
	// get the rotation matrix
	M = getRotationMatrix2D(rect.center, angle, 1.0);
	// perform the affine transformation
	cv::warpAffine(src, rotated, M, src.size(), cv::INTER_CUBIC);
	// crop the resulting image
	getRectSubPix(rotated, rect_size, rect.center, cropped);
	
	return cropped; 
}



std::vector<cv::RotatedRect> gummiAlgo(cv::Mat im_orig, cv::Mat & im_with_keypoints, GummiAlgoOptions opts){
	cv::Mat im_hsv;
	cv::cvtColor(im_orig, im_hsv, cv::COLOR_BGR2HLS);
	cv::Mat im_gray(im_hsv.size(),CV_8U);
	{
		cv::MatIterator_<cv::Vec3b> it, end;
		cv::MatIterator_<uchar> grey_it = im_gray.begin<uchar>();
		for( it = im_hsv.begin<cv::Vec3b>(), end = im_hsv.end<cv::Vec3b>(); it != end; ++it)
		{
			uchar h = (*it)[0];
			uchar s = (*it)[1];
			uchar v = (*it)[2];
			
			(*grey_it) = s;
			++grey_it;
		}
	}
	cv::Mat im;
	cv::GaussianBlur(im_gray, im,{5,5},255);
	swap(im,im_gray);
	cv::threshold(im_gray, im, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

	
	std::vector<GummiBear> result;
	
	using namespace cv; 
	float alpha = opts.alpha;
	float beta = opts.beta;
	im.convertTo(im, -1, alpha, beta);
	
	// Setup SimpleBlobDetector parameters.
	cv::SimpleBlobDetector::Params params;
	
	// Change thresholds
	params.minThreshold = opts.minThreshold;
	params.maxThreshold = opts.maxThreshold;
	
	// Filter by Area.
	params.filterByArea = true;
	params.minArea = opts.minArea;
	
	// Filter by Circularity
	params.filterByCircularity = true;
	params.minCircularity = opts.minCircularity;
	
	// Filter by Convexity
	params.filterByConvexity = true;
	params.minConvexity = opts.minConvexity;
	
	// Filter by Inertia
	params.filterByInertia = true;
	params.minInertiaRatio = opts.minInertia;
	
	// Set up the detector with default parameters.
	cv::Ptr<cv::SimpleBlobDetector> detector = cv::SimpleBlobDetector::create(params);
	
	// Detect blobs.
	std::vector<KeyPoint> keypoints;
	detector->detect( im, keypoints );
	
	// Draw detected blobs as red circles.
	// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
	drawKeypoints( im, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
	
	
	int kidx = 0;
	for(auto & pt : keypoints){
		++kidx;
		
		int w = pt.size*1.6;
		cv::Size size(w,w);
		DrawRotatedRectangle(im_with_keypoints, pt.pt, size, 0);
		
		cv::Point roi_pt(max(0.0f,pt.pt.x-w/2),max(0.0f,pt.pt.y-w/2));
		if(roi_pt.x+w>=im.cols || roi_pt.y+w>=im.rows) continue;
		cv::Rect rect(roi_pt, size);
		cv::Mat roi_grey = im(rect);
		cv::Mat roi;
		int thresh = 50;
		int maxValue = 255;
		threshold(roi_grey, roi, thresh, maxValue, cv::THRESH_BINARY + cv::THRESH_OTSU);
		
		std::vector<std::vector<cv::Point> > contours;
		std::vector<cv::Vec4i> hierarchy;
		cv::findContours( roi, contours, hierarchy, RETR_TREE, CHAIN_APPROX_SIMPLE );
		int best_contour = -1;
		float best_area = 0;
		for( size_t i = 0; i< contours.size(); i++ )
		{
			float area = contourArea(contours[i]);
			if(area>500 && area<2500 && area>best_area && area<w*w*0.5 && contours[i].size()>5){
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
			
			
			
			result.push_back(fit);
			DrawRotatedRectangle(im_with_keypoints, fit.center, fit.size, fit.angle);
			
			//cv::Mat extracted = MatFromRotatedRect(im,fit);
			//imwrite(basedir + "/res/" + filename + to_string(kidx) + ".jpg", extracted);
		}
	}
	
	// Show blobs
	//imshow("keypoints", im_with_keypoints );
	//imwrite(basedir + "/res/" + filename, im_with_keypoints);
	return result;
}
