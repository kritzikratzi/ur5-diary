#include <opencv2/opencv.hpp>

// Include center point of your rectangle, size of your rectangle and the degrees of rotation
void DrawRotatedRectangle(cv::Mat& image, cv::Point centerPoint, cv::Size rectangleSize, double rotationDegrees);

cv::Mat MatFromRotatedRect(cv::Mat& src, const cv::RotatedRect & rect); 


struct GummiAlgoOptions{
	float alpha = 1.0;
	float beta = 0.0;
	float minArea = 1200;
	float minThreshold = 0;
	float maxThreshold = 255;
	float minCircularity = 0.2;
	float minConvexity = 0.01;
	float minInertia = 0.0; 
};

// returns the post-processed image, and returns a collection of rotated rects
enum class GummiColor{ red, green, yellow};
struct GummiBear{
	cv::RotatedRect rect;
	GummiColor color;
};
std::vector<GummiBear> gummiAlgo(cv::Mat input, cv::Mat & im_with_keypoints, GummiAlgoOptions opts = GummiAlgoOptions());
