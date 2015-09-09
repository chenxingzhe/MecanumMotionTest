
#include <iostream>
#include <opencv2/opencv.hpp>
#include <sstream>
#include "calclandmark.h"
#include "bwlabel.h"
#include <time.h>
using namespace std;
using namespace cv;
using namespace cameraMotion;
void getThresFromCluter(const cluter &curCluter,color &curColor){
	double h = curCluter.center.at<double>(0, 0);
	double s = curCluter.center.at<double>(0, 1);
	double v = curCluter.center.at<double>(0, 2);
	double sigmaH = sqrt(curCluter.cov.at<double>(0, 0)); 
	double sigmaS = sqrt(curCluter.cov.at<double>(1, 1));
	double sigmaV = sqrt(curCluter.cov.at<double>(2, 2));
	double ratio = 3;
	curColor.threshold_value1 = h - ratio*sigmaH;
	curColor.threshold_value2 = h + ratio*sigmaH;
	curColor.threshold_value3 = s - ratio*sigmaS;
	curColor.threshold_value4 = s + ratio*sigmaS;
	curColor.threshold_value5 = v - ratio*sigmaV;
	curColor.threshold_value6 = v + ratio*sigmaV;
}

CalcRTLandmark::CalcRTLandmark(Mat KK, Mat DD)
{
	intrinsic = KK;
	distortion = DD;
	isCalibed = false;
	//blue = blue1;
	//red = red1;
	s = 0;
	isInitialized = false;
	loadGMM();
	getThresFromCluter(blueGMM, blue);
	getThresFromCluter(redGMM, red);
}
CalcRTLandmark::~CalcRTLandmark()
{}

bool CalcRTLandmark::findBestPair(const vector<PointCluter> &blueContour, const vector<PointCluter> &redContour, int &indexBlue, int &indexRed, Mat &img){
	vector<PointCluter> cluter1, cluter2;
	int index1=0, index2=0;
	
	vector<double> matchScore(blueContour.size()*redContour.size());
	for (int i = 0; i < matchScore.size(); ++i)
		matchScore[i] = -1;
	if (blueContour.size() <= redContour.size()){ //一主一从
		cluter1 = blueContour;
		cluter2 = redContour;
	}
	else{
		cluter1 = redContour;
		cluter2 = blueContour;
	}
	bool validFlag = false;
	int sizeRatio = 2;
	int minNum = 30;
	double minScore = 100000000000000;

	for (int i = 0; i < cluter1.size(); ++i){
		if (cluter1[i].pointNum<minNum)
			continue;
		for (int j = 0; j < cluter2.size(); ++j){
			//if (showFlag){
			//	Mat tempImg;
			//	img.copyTo(tempImg);
			//	vector<vector<Point> > curConture;
			//	curConture.push_back(cluter1[i].allPoint);
			//	drawContours(tempImg, curConture, 0, Scalar(0, 0, 255), 2);
			//	curConture.clear();
			//	curConture.push_back(cluter2[j].allPoint);
			//	drawContours(tempImg, curConture, 0, Scalar(0, 255, 0), 2);
			//	imshow("tempArray", tempImg);
			//	waitKey(10);
			////}
			//if (cluter2[j].pointNum<minNum)
			//	continue;

			if (cluter1[i].pointNum>sizeRatio*cluter2[j].pointNum||
				cluter2[j].pointNum>sizeRatio*cluter1[i].pointNum)
				continue;
			//Point2d p1 = Point2d(cluter1[i].meanX, cluter1[i].meanY);
			//Point2d p2 = Point2d(cluter2[j].meanX, cluter2[j].meanY);

			//double dis = sqrt((p1.x - p2.x)* (p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y));

			//if (dis/5>sqrt(cluter1[i].pointNum) + sqrt(cluter2[j].pointNum)) //两个块之间的距离比较小
			//	continue;

			/*cout << "汉明距离：" << score << endl;*/
			//imshow("con", tempMap);
			//waitKey(0);
			matchScore[j*cluter1.size() + i] = cluter1[i].score + cluter2[j].score;
			validFlag = true;

			if (matchScore[j*cluter1.size() + i] < minScore)
			{
				minScore = matchScore[j*cluter1.size() + i];
				index1 = i;
				index2 = j;
			}
		}
	}

	if (validFlag){
		vector<vector<Point> > curConture;
		curConture.push_back(cluter1[index1].allPoint);
		drawContours(img, curConture, 0, Scalar(0, 0, 255), 2);
		curConture.clear();
		curConture.push_back(cluter2[index2].allPoint);
		drawContours(img, curConture, 0, Scalar(0, 255, 0), 2);
	}
	resize(img, img, Size(640,360));

	if (!validFlag)
		return false;

	cout << "得分：" << matchScore[index2*cluter1.size() + index1] << endl;
	if (matchScore[index2*cluter1.size() + index1] > 8)
		return false;
	//double maxScore = -1;
	//int maxIndex = 0;
	//for (int i = 0; i < matchScore.size(); ++i){
	//	if (matchScore[i]<0)
	//		continue;
	//	if (matchScore[i] > maxScore){
	//		maxIndex = i;
	//		maxScore = matchScore[i];
	//	}
	//}

	//double minScore = 100000000000000;
	//int minIndex = 0;
	//for (int i = 0; i < matchScore.size(); ++i){
	//	if (matchScore[i]<0)
	//		continue;
	//	if (matchScore[i] > maxScore){
	//		maxIndex = i;
	//		maxScore = matchScore[i];
	//	}
	//}

	//index2 = maxIndex / cluter1.size();
	//index1 = maxIndex % cluter1.size();
	//imshow("con", tempMap);
	//waitKey(0);

	if (blueContour.size() <= redContour.size()){ //一主一从
		indexBlue=index1;//cluter1 = blueContour;
		indexRed = index2;
	}
	else{
		indexBlue = index2;//cluter1 = blueContour;
		indexRed = index1;
	}
	//countNonZero(modelHashCode != testHashCode);
	return true;
}

bool CalcRTLandmark::loadGMM(){
	{
		FileStorage fs2("red.xml", FileStorage::READ); //便于多文件计算gauss
		if (!fs2.isOpened()){
			cout << "打开文件错误" << endl;
			return false;
		}

		cluter cur;
		fs2["cov"] >> cur.cov;
		fs2["mu"] >> cur.center;
		fs2["covinv"] >> cur.covinv;
		fs2["sampleNum"] >> cur.sampleNum;
		redGMM = cur;
	}
	{
		FileStorage fs2("blue.xml", FileStorage::READ); //便于多文件计算gauss
		if (!fs2.isOpened()){
			cout << "打开文件错误" << endl;
			return false;
		}

		cluter cur;
		fs2["cov"] >> cur.cov;
		fs2["mu"] >> cur.center;
		fs2["covinv"] >> cur.covinv;
		fs2["sampleNum"] >> cur.sampleNum;
		blueGMM = cur;
	}
	//}
	return true;
}

bool getPointCluterFromImage(const Mat& hsvImage, const cluter &interestCluter,vector<PointCluter> &curPointCluter){
	curPointCluter.clear();

	color interestColor;
	getThresFromCluter(interestCluter, interestColor);

	Mat_<Vec3b> hsv_(hsvImage); Mat out0, refine0;
	inRange(hsvImage, Scalar(interestColor.threshold_value1, interestColor.threshold_value3, interestColor.threshold_value5),
		Scalar(interestColor.threshold_value2, interestColor.threshold_value4, interestColor.threshold_value6), out0);
	Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
	erode(out0, refine0, element);
	dilate(refine0, refine0, element);
	//if (showFlag){
	//	imshow("inrange", refine0);
	//	waitKey(10);
	//}
	int *labels = new int[refine0.rows*refine0.cols];
	int maxLabel = bwlabel(&IplImage(refine0), 8, labels);
	if (maxLabel == 0){
		delete[] labels;
		return false;
	}
	curPointCluter.resize(maxLabel);

	int allNum = refine0.rows*refine0.cols;
	for (int k = 0; k < allNum; ++k){
		if (labels[k] == 0)
			continue;
		int curLabel = labels[k] - 1;
		int x = k%refine0.cols;
		int y = k / refine0.cols;
		curPointCluter[curLabel].allPoint.push_back(Point(x, y));
		curPointCluter[curLabel].meanX += x;
		curPointCluter[curLabel].meanY += y;
		Vec3d p = hsv_(y, x);
		Mat err = Mat(p).reshape(1).t() - interestCluter.center;
		Mat res = err*interestCluter.covinv*err.t();
		curPointCluter[curLabel].score += res.at<double>(0, 0);
		++curPointCluter[curLabel].pointNum;
	}

	for (int i = 0; i < maxLabel; ++i){
		curPointCluter[i].normlize();
	}
	delete[] labels;
	return true;
}

inline string double2str(double val){
	stringstream ss;
	ss << val;
	return ss.str();
}

bool CalcRTLandmark::Calc(Mat image, Mat &R, Mat &t,double &w)
{
	double sizeRatio = 1;
	resize(image, image, Size(image.cols / sizeRatio, image.rows / sizeRatio));
	Mat hsv, out0, out1;//图片hsv矩阵
	Mat refine0, refine1;//腐蚀后结果
	Mat_<Vec3b> hsv_(hsv);
	cvtColor(image, hsv, CV_RGB2HSV_FULL);
	vector<PointCluter> bluePointCluter;
	if (!getPointCluterFromImage(hsv, blueGMM, bluePointCluter))
		return false;
	
	vector<PointCluter> redPointCluter;
	if (!getPointCluterFromImage(hsv, redGMM, redPointCluter))
		return false;

	int blueIndex, redIndex;
	Mat pairImg = Mat::zeros(image.rows, image.cols, CV_8UC3);
	if (!findBestPair(bluePointCluter, redPointCluter, blueIndex, redIndex, pairImg)){
		imshow("Pair", pairImg);
		waitKey(10);
		return false;
	}
	//cout << p1 << endl;
	//circle(refine0, p, 5, Scalar(255, 255, 255), 5);
	//imshow("refine0", refine0);
	//imshow("refine1", refine1);
	imshow("Pair", pairImg);
	waitKey(10);

	Mat p_origin, p_after;
	double a[2] = { bluePointCluter[blueIndex].meanX*sizeRatio, bluePointCluter[blueIndex].meanY*sizeRatio };
	double b[2] = { redPointCluter[redIndex].meanX*sizeRatio, redPointCluter[redIndex].meanY*sizeRatio };
	//double a[2] = { 0, 0 };
	//double b[2] = { 0, 0 };
	p_origin.push_back(Mat(1, 1, CV_64FC2, a));
	undistortPoints(p_origin, p_after, intrinsic, distortion);
	double x0 = p_after.at<double>(0, 0);
	double y0 = p_after.at<double>(0, 1);
	double fx = intrinsic.at<double>(0, 0), cx = intrinsic.at<double>(0, 2), fy = intrinsic.at<double>(1, 1), cy = intrinsic.at<double>(1, 2);
	x0 = x0*fx + cx;
	y0 = y0*fy + cy;
	//cout << "cord  " << p_origin << endl << x0 << " " << y0 << endl;
	p_origin.pop_back();
	p_origin.push_back(Mat(1, 1, CV_64FC2, b));
	undistortPoints(p_origin, p_after, intrinsic, distortion);
	double x1 = p_after.at<double>(0, 0);
	double y1 = p_after.at<double>(0, 1);

	x1 = x1*fx + cx;
	y1 = y1*fy + cy;
	//cout << p_origin << endl << x1 << " " << y1 << endl;
	Mat pic0 = (Mat_<double>(3, 1) << x0, y0, 1);
	Mat pic1 = (Mat_<double>(3, 1) << x1, y1, 1);
	Mat cam0, cam1;
	/*double s0,s1,Zw=1,sinw,cosw,Yw=2.4;//R矩阵的变换系数
	cam0 = intrinsic.inv()*pic0;
	cam1 = intrinsic.inv()*pic1;
	t.at<double>(0, 0) = cam0.at<double>(0, 0);
	t.at<double>(1, 0) = cam0.at<double>(1, 0);
	s0 = cam0.at<double>(2, 0) / Zw;
	s1 = cam1.at<double>(2, 0) / Zw;
	sinw = (cam1.at<double>(0, 0) - t.at<double>(0, 0)) / Yw / s1;
	cosw = (cam1.at<double>(1, 0) - t.at<double>(1, 0)) / Yw / s1;
	cout << t << endl << sinw << " " << cosw << endl << s0 << " " << s1 << endl;*/

	double Zw = 1, sinw, cosw, Yw = 600;//R矩阵的变换系数
	cam0 = intrinsic.inv()*pic0;
	cam1 = intrinsic.inv()*pic1;
	t.at<double>(0, 0) = cam0.at<double>(0, 0);
	t.at<double>(1, 0) = cam0.at<double>(1, 0);
	s = sqrt((cam0.at<double>(0, 0) - cam1.at<double>(0, 0))*(cam0.at<double>(0, 0) - cam1.at<double>(0, 0)) +
		(cam0.at<double>(1, 0) - cam1.at<double>(1, 0))*(cam0.at<double>(1, 0) - cam1.at<double>(1, 0))) / Yw;
	//s0 = cam0.at<double>(2, 0) / Zw;
	//s1 = cam1.at<double>(2, 0) / Zw;
	//cout << cam1 << endl;
	/*sinw = -1 * (cam1.at<double>(0, 0) - t.at<double>(0, 0)) / Yw / s;
	cosw = (cam1.at<double>(1, 0) - t.at<double>(1, 0)) / Yw / s;*/
	cosw = (cam1.at<double>(0, 0) - t.at<double>(0, 0)) / Yw / s;
	sinw = (cam1.at<double>(1, 0) - t.at<double>(1, 0)) / Yw / s;
	double rot[4] = { cosw, -sinw, sinw, cosw };
	double angle = 0;// = atan(sinw / cosw);
	/*if (sinw > 0 && cosw < 0)
	{
		angle = angle + PI;
	}
	if (sinw < 0 && cosw < 0)
	{
		angle = angle - PI;
	}*/
	//cosw = 1;
	//cout << acos(cosw) << endl;
	//angle = acos(cosw);
	//if (sinw < 0)
	//	angle *= -1;


	//cout << "Angle:" << angle * 180 / 3.1415926 << endl;
	/*w = angle * 180 / PI;*/
	//cout << "未修正的t："<<t / s << endl;
	R = Mat(2, 2, CV_64FC1, rot);
	//cout << "未修正的t：" << R.inv()*t / s << endl;
	Mat RT = Mat::eye(Size(3, 3), CV_64FC1);
	Rect rect(0, 0, 2, 2);
	R.copyTo(RT(rect));
	rect = Rect(2, 0, 1, 2);
	t.copyTo(RT(rect));
	RT = RT.inv();
	RT(rect).copyTo(t);
	rect = Rect(0, 0, 2, 2);
	RT(rect).copyTo(R);
	cosw = R.at<double>(0, 0);
	sinw = R.at<double>(1, 0);
	angle = acos(cosw);
	if (sinw < 0)
		angle *= -1;
	//cout << "Angle:" << angle * 180 / 3.1415926 << endl;
	w = angle * 180 / PI+180;
	if (w>180)
		w -= 360;
	//cout << "Trans:" << t / s << endl;
	Mat value = Mat::zeros(320, 320, CV_8UC3);
	//cout << "比例系数："<<s << endl;
	t = t / s;
	//cout << "修正的t:" << t << endl

	double Ra = 190.9250321518893;
	double jiao = 189.5 / 180 * PI;

	//t.at<double>(0, 0) = t.at<double>(0, 0) - Ra*cos(angle - jiao);
	//t.at<double>(1, 0) = t.at<double>(1, 0) - Ra*sin(angle - jiao);
	t.at<double>(0, 0) = t.at<double>(0, 0);
	t.at<double>(1, 0) = t.at<double>(1, 0);
	double showscale = 1;
	putText(value, string("Angle:") + double2str(w), Point(0, 50), CV_FONT_HERSHEY_COMPLEX, 1, cvScalar(200, 200, 200, 0));
	putText(value, string("TransX:") + double2str(t.at<double>(0, 0) - Ra*cos(angle - jiao)), Point(0, 100), CV_FONT_HERSHEY_COMPLEX, showscale, cvScalar(200, 200, 200, 0));
	putText(value, string("TransY:") + double2str(t.at<double>(1, 0) - Ra*sin(angle - jiao)), Point(0, 150), CV_FONT_HERSHEY_COMPLEX, showscale, cvScalar(200, 200, 200, 0));
	imshow("value", value);
	waitKey(1);
	return true;
}

bool CalcRTLandmark::ransacGetRT(const vector<Point2f> &points1, const vector<Point2f> &points2, Mat &RT, double &projectErr, double inLineThres){
	if (points1.size() != points2.size())
		return false;

	if (inLineThres < 0){ //计算inline阈值
		double sumX = 0; double sumY = 0;
		for (int i = 0; i < points2.size(); ++i){
			sumX += points2[i].x;
			sumY += points2[i].y;
		}
		sumX /= points2.size();
		sumY /= points2.size();

		inLineThres = 0;
		//计算距离质心最远的点的距离，选取该距离的20%作为阈值
		for (int i = 0; i < points2.size(); ++i){
			sumX += points2[i].x;
			sumY += points2[i].y;
			double tempErr = sqrt((sumX - points2[i].x)*(sumX - points2[i].x) +
				(sumY - points2[i].y)*(sumY - points2[i].y));
			if (tempErr>inLineThres)
				inLineThres = tempErr;
		}
		inLineThres *= 0.005;
	}


	srand((int)time(0));
	vector<Point2f> temPoints1; vector<Point2f> temPoints2;
	int sampleNum = 5;
	double inRateThres = 0.9;
	double inrate = 0;
	int MaxIteratorTime = 20;
	int runTime = 0;
	Mat_<double> returnRT;
	double projectErrAll = 0;

	while (inrate < inRateThres || (runTime < 3 && inrate<0.99)){
		if (runTime>MaxIteratorTime)
			return false;
		if (runTime == 0 || inrate < inRateThres){ //首次运行或者得到的有效的点太少，则重新选取种子点 
			for (int i = 0; i < sampleNum; ++i){
				int randIndex = rand() % points1.size();
				temPoints1.push_back(points1[randIndex]);
				temPoints2.push_back(points2[randIndex]);
			}
		}
		Mat A, b;
		for (int i = 0; i < temPoints1.size(); ++i){
			Mat_<double> tempMat(2, 4);
			tempMat << temPoints1[i].x, temPoints1[i].y, 1, 0,
				temPoints1[i].y, -1 * temPoints1[i].x, 0, 1;
			A.push_back(tempMat);
			Mat_<double> tempMatb(2, 1);
			tempMatb << temPoints2[i].x, temPoints2[i].y;
			b.push_back(tempMatb);
		}
		A.convertTo(A, CV_64FC1); b.convertTo(b, CV_64FC1);
		returnRT = (A.t()*A).inv()*A.t()*b;

		//计算内点比例
		double dx = returnRT(2); double dy = returnRT(3);
		double cos_ = returnRT(0); double sin_ = returnRT(1);
		projectErrAll = 0;

		temPoints1.clear();
		temPoints2.clear();
		inrate = 0;
		for (int i = 0; i < points1.size(); ++i)
		{
			double px = cos_*points1[i].x + sin_*points1[i].y + dx;
			double py = -1 * sin_*points1[i].x + cos_*points1[i].y + dy;
			double tempErr = sqrt((px - points2[i].x)*(px - points2[i].x) +
				(py - points2[i].y)*(py - points2[i].y));

			if (tempErr>inLineThres)
				continue;
			projectErrAll += tempErr;
			++inrate;
			temPoints1.push_back(points1[i]); //下一次ransac需要的点
			temPoints2.push_back(points2[i]);
		}
		projectErrAll /= inrate;
		inrate /= points1.size();
		++runTime;
		//cout <<"优化过程中的内点比例："<< inrate << endl;
	}

	////eventual plot
	//double dx = returnRT(2); double dy = returnRT(3);
	//double cos_ = returnRT(0); double sin_ = returnRT(1);
	//fstream out("plot.txt", ios::out);
	//for (int i = 0; i < points1.size(); ++i)
	//{
	//	double px = cos_*points1[i].x + sin_*points1[i].y + dx;
	//	double py = -1 * sin_*points1[i].x + cos_*points1[i].y + dy;
	//	out << px << "\t" << py << "\t" << points2[i].x << "\t" << points2[i].y << endl;
	//}
	//cout << "得到的比例：" << sqrt(cos_*cos_ + sin_*sin_) << endl;
	//out.close();
	//cout << "内点比例："<<inrate << endl;
	projectErr = projectErrAll;
	returnRT.copyTo(RT);
	return true;
}

void undistortKeyPoints(vector<Point2f> &keyPoint, vector<Point2f> &remapped, const Mat &cameraMat, const Mat &distMat){ //去畸变操作
	double fx = cameraMat.at<double>(0, 0);
	double cx = cameraMat.at<double>(0, 2);
	double fy = cameraMat.at<double>(1, 1);
	double cy = cameraMat.at<double>(1, 2);
	Mat pointMat;
	for (int i = 0; i < keyPoint.size(); ++i){
		double tempPoint[2] = { keyPoint[i].x, keyPoint[i].y };
		pointMat.push_back(Mat(1, 1, CV_64FC2, tempPoint));
	}
	undistortPoints(pointMat, pointMat, cameraMat, distMat);
	Mat_<Vec2d> pointMat_(pointMat);
	remapped.resize(keyPoint.size());
	for (int i = 0; i < keyPoint.size(); ++i){
		remapped[i].x = pointMat_(i)[0] * fx + cx;
		remapped[i].y = pointMat_(i)[1] * fy + cy;
	}
}


bool  CalcRTLandmark::getRTwithImagePair(const cv::Mat &image1_, const cv::Mat &image2_, cv::Mat &RT){ //只是有图像得到的RT
	Mat image1, image2;
	image1 = image1_;
	image2 = image2_;
	if (image1.channels() > 1){
		cvtColor(image1, image1, CV_RGB2GRAY);
		cvtColor(image2, image2, CV_RGB2GRAY);
	}
	vector<Point2f> points1, points2; //point in src and dst image;
	//点的提取
	goodFeaturesToTrack(image1, points1, 500, 0.01, 10); //10, mask1, 3, 0, 0.04
	if (points1.size() < 5)
		return false;
	double radiusThres = 35000; //70000
	for (int i = 0; i < points1.size();){
		if ((points1[i].x - intrinsic.at<double>(0, 2))*(points1[i].x - intrinsic.at<double>(0, 2)) +
			(points1[i].y - intrinsic.at<double>(1, 2))*(points1[i].y - intrinsic.at<double>(1, 2)) < radiusThres){
			++i;
			continue;
		}
		points1[i] = points1.back();
		points1.pop_back();
	}

	Size subPixWinSize(10, 10), winSize(31, 31);;
	TermCriteria termcrit(CV_TERMCRIT_ITER | CV_TERMCRIT_EPS, 20, 0.03);
	cornerSubPix(image1, points1, subPixWinSize, Size(-1, -1), termcrit);

	//光流法
	vector<uchar> status;
	vector<float> err;
	calcOpticalFlowPyrLK(image1, image2, points1, points2, status, err, winSize,
		3, termcrit, 0, 0.001);
	//在第二幅图像上对位置进行精化，该步骤需要进一步验证是否可行
	cornerSubPix(image2, points2, subPixWinSize, Size(-1, -1), termcrit);
	vector<Point2f> remapped1, remapped2; //point in src and dst image;
	vector<Point2f> remapped1_3D, remapped2_3D; //point in src and dst image;
	undistortKeyPoints(points1, remapped1, intrinsic, distortion);
	undistortKeyPoints(points2, remapped2, intrinsic, distortion);
	remapped1_3D.resize(remapped1.size());
	remapped2_3D.resize(remapped2.size());
	Mat_<double> intrinsicInv = intrinsic.inv();
	cout << intrinsicInv << endl;
	for (int i = 0; i < remapped1.size(); ++i){
		remapped1_3D[i].x = remapped1[i].x*intrinsicInv(0, 0) + intrinsicInv(0, 2);
		remapped1_3D[i].y = remapped1[i].y*intrinsicInv(1, 1) + intrinsicInv(1, 2);
		remapped2_3D[i].x = remapped2[i].x*intrinsicInv(0, 0) + intrinsicInv(0, 2);
		remapped2_3D[i].y = remapped2[i].y*intrinsicInv(1, 1) + intrinsicInv(1, 2);
	}
	Mat validMask;
	findHomography(remapped1, remapped2, CV_RANSAC, 3, validMask);
	remapped1 = remapped1_3D;
	remapped2 = remapped2_3D;

	vector<Point2f> tempRemapped1_3D, tempRemapped2_3D;
	for (int i = 0; i < points1.size(); ++i)
	{
		if (!status[i])
			continue;
		if (!validMask.at<uchar>(i))
			continue;
		tempRemapped1_3D.push_back(remapped1[i]);
		tempRemapped2_3D.push_back(remapped2[i]);
	}
	Mat_<double> ss;
	double errproj;
	if (!ransacGetRT(tempRemapped1_3D, tempRemapped2_3D, ss, errproj)){
		return false;
	}
	ss(0) /= sqrt(ss(0)*ss(0) + ss(1)*ss(1));
	ss(1) /= sqrt(ss(0)*ss(0) + ss(1)*ss(1));
	RT = ss;
	return true;
}

bool CalcRTLandmark::calibCameraAndOdo(const std::vector<cv::Point2f> &pointsCamera, const std::vector<cv::Point2f> &pointsOdo){
	double projectErr;
	Mat_<double> ss;
	if (!ransacGetRT(pointsCamera, pointsOdo, ss, projectErr))
		return false;
	double dx = ss(2); double dy = ss(3);
	double cos_ = ss(0); double sin_ = ss(1);
	cout << "正在标定：" << endl<< ss << endl;
	isCalibed = true;
	return true;
}

bool CalcRTLandmark::getRTwithImagePair2Odo(const cv::Mat &image1, const cv::Mat &image2, cv::Mat &RT){
	if (!getRTwithImagePair(image1, image2, RT))
		return false;
	if (!isCalibed)
		return false;
	Mat_<double> RT_(RT);
	double x = RT_(2); double y = RT_(3);
	RT.at<double>(2) = cos_*x + sin_*y + dx;
	RT.at<double>(3) = -1 * sin_*x + cos_*y + dy;
	return true;
}