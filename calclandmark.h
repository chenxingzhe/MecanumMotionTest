#ifndef CALCLANDMARK_H
#define CALCLANDMARK_H
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>

namespace cameraMotion{
	using namespace std;
	using namespace cv;
	#define PI 3.1415926
	struct color  
	{
		int threshold_value1;
		int threshold_value2;
		int threshold_value3;
		int threshold_value4;
		int threshold_value5;
		int threshold_value6;
	};

	struct cluter{ //ÑÕÉ«Ê¶±ð
		cv::Mat center;
		cv::Mat cov;
		cv::Mat covinv;
		float sampleNum;
		//bool protect_flag; //in history ,true means it has been updated
		//in curr,true means it has helped history to be updated
		cluter() :sampleNum(0), center(cv::Mat::zeros(1, 3, CV_64FC1)), cov(cv::Mat::zeros(3, 3, CV_64FC1)){}
	};

	struct PointCluter{
		vector<Point> allPoint;
		double meanX;
		double meanY;
		double score;
		int pointNum;
		PointCluter() :meanX(0), meanY(0), pointNum(0), score(0){}
		void normlize(){
			meanY /= pointNum;
			meanX /= pointNum;
			score /= pointNum;
		}
	};
	class CalcRTLandmark
	{
	public:

	
		CalcRTLandmark(Mat KK, Mat DD);
		bool Calc(Mat image,Mat &R,Mat &t,double &w);
		~CalcRTLandmark();
	
	private:
		Mat intrinsic;
		Mat distortion;
		color blue;
		color red;
		cluter redGMM;
		cluter blueGMM;
		double s;
		bool isInitialized;
		bool findBestPair(const vector<PointCluter> &blueContour, const vector<PointCluter> &redContour, int &indexBlue, int &indexRed, Mat &img);
		bool loadGMM();
	};
}
#endif