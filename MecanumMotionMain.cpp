#include "MecanumMotion.h"
#include <opencv2/opencv.hpp>
#include <opencv2/xphoto.hpp>
#include <windows.h>
#include "calclandmark.h"
#include <fstream>
#include <iostream>
#include <boost/thread/thread.hpp>

USING_NAMESPACE_NEAT_COMMON_ALL();

using namespace std;
using namespace cv;

void motionControl();
void getCameraPose();

int DataProcessFlag=-1;
bool validCameraPose = false;
double xpos=0;
double ypos=0;
double cita=0;
bool camOrOdo = false;

boost::mutex curCamPosMutex; //相机得到的位姿
CurPos curCamPos;

boost::mutex curOdoPosMutex; //Odo得到的位姿
CurPos curOdoPos;

int main()
{
	cout << "记录数据输入0，巡线走输入1，圆标定输入2，混合记录输入3，其他输入任意数：" << endl;
	cin >> DataProcessFlag;
	boost::thread thrMotionControl(motionControl);
	boost::thread thrGetCameraPose(getCameraPose);
	thrGetCameraPose.join();
	thrMotionControl.join();

	system("pause");
	return 0;
}

void motionControl()
{
	MecanumMotion *mctrl = MecanumMotionObj::Instance();

	mctrl->doMotionControlWithCamera();

	return;
}

string num2str(int i){
	stringstream s;
	s << i;
	return s.str();
}

inline string double2str(double val){
	stringstream ss;
	ss << val;
	return ss.str();
}
void getCameraPose()
{
	Mat intrinsic = (Mat_<double>(3, 3) << 199.2101, 0, 303.3105,
		0, 199.2539, 256.8642,
		0, 0, 1.0000);
	Mat distortion = (Mat_<double>(5, 1) << -0.209018326284060, 0.0343916492664589, -0.000265874547218015, 0.00235688593244335, -0.00221045436495917);

	cameraMotion::CalcRTLandmark a(intrinsic, distortion);
	Mat R = (Mat_<double>(2, 2) << 1, 0, 0, 1);
	Mat t = (Mat_<double>(2, 1) << 0, 0);
	double w;
	Mat frame;
	char c;
	cv::VideoCapture videoCapture(0);
	cout << videoCapture.isOpened() << endl;
	videoCapture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
	videoCapture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
	int curIndex = 0;
	static ifstream getfile;
	if (!getfile.is_open()) {
		getfile.open("Rjiao.txt", ios::in);
	}
	double Ra, jiao;
	getfile >> Ra >> jiao;
	CurPos tempPreOdo; //目前原因未明,第一次取值必须要丢弃

	Mat imagePre;
	CurPos tempPrePreOdo;
	
	vector<Point2f> pointsCamera;
	vector<Point2f> pointsOdo; //for calib

	while (1)
	{
		//{
		//	CurPos tempCurOdo;
		//	curOdoPosMutex.lock();
		//	tempCurOdo = curOdoPos;
		//	curOdoPosMutex.unlock();
		//	if (!tempPreOdo.isValid){
		//		tempPreOdo = tempCurOdo;
		//		continue;
		//	}
		//	videoCapture >> frame; //得到图形和里程计数据，中间不能有延时
		//	if (frame.cols==0||frame.rows==0)
		//		continue;
		//	if (!tempPrePreOdo.isValid){ //构建pair
		//		tempPrePreOdo = tempPreOdo;
		//		frame.copyTo(imagePre);
		//		continue;
		//	}
		//	
		//	imshow("org", frame);
		//	c = cvWaitKey(20);
		//	double angleOdo, angle;
		//	Mat_<double> RT;
		//	if (!a.isCalibed){
		//		if (sqrt((tempPreOdo.x - tempPrePreOdo.x)*(tempPreOdo.x - tempPrePreOdo.x)
		//			+ (tempPreOdo.y - tempPrePreOdo.y)*(tempPreOdo.y - tempPrePreOdo.y)) < 5){
		//			tempPreOdo = tempCurOdo;
		//			continue;
		//		}
		//		if (!a.getRTwithImagePair(imagePre, frame, RT)){
		//			cout << "2" << endl;
		//			continue;
		//		}

		//		if (sqrt(RT(2)*RT(2) + RT(3)*RT(3)) == 0)
		//		{
		//			cout << "机器人卡住！！" << endl;
		//			continue;
		//		}

		//		angle = acos(RT(0));
		//		if (RT(1) < 0)
		//			angle *= -1;
		//		angle = angle * 180 / 3.1415926;
		//		angleOdo = (tempPreOdo.cita - tempPrePreOdo.cita) / 3.1415926 * 180;
		//		if (abs(angleOdo)>30)
		//		if (angleOdo>0)
		//			angleOdo -= 360;
		//		else
		//			angleOdo += 360;

		//		if (abs(angle - angleOdo) < 0.5){
		//			pointsCamera.push_back(Point2f(RT(2), RT(3)));
		//			pointsOdo.push_back(Point2f(tempPreOdo.x - tempPrePreOdo.x,
		//				tempPreOdo.y - tempPrePreOdo.y));
		//			cout << "增加了一个点！" << endl;
		//		}
		//		if (pointsCamera.size()>10)
		//			a.calibCameraAndOdo(pointsCamera, pointsOdo);
		//	}
		//	else{
		//		if(!a.getRTwithImagePair2Odo(imagePre, frame, RT))
		//			continue;
		//		angle = acos(RT(0));
		//		if (RT(1) < 0)
		//			angle *= -1;
		//		angle = angle * 180 / 3.1415926;
		//		angleOdo = (tempPreOdo.cita - tempPrePreOdo.cita) / 3.1415926 * 180;
		//		if (abs(angleOdo)>30)
		//		if (angleOdo > 0)
		//			angleOdo -= 360;
		//		else
		//			angleOdo += 360;
		//	}
		//	//todo
		//	Mat value = Mat::zeros(320, 320, CV_8UC3);
		//	putText(value, string("Angle:") + double2str(angle) + " "+double2str(angleOdo), Point(0, 50), CV_FONT_HERSHEY_COMPLEX, 1, cvScalar(200, 200, 200, 0));
		//	putText(value, string("Trans:") + double2str(RT(2)) + " " + double2str(RT(2)), Point(0, 100), CV_FONT_HERSHEY_COMPLEX, 1, cvScalar(200, 200, 200, 0));
		//	putText(value, string("State:") + double2str(a.isCalibed), Point(0, 150), CV_FONT_HERSHEY_COMPLEX, 1, cvScalar(200, 200, 200, 0));
		//	imshow("value", value);
		//	waitKey(20);
		//	

		//	frame.copyTo(imagePre);
		//	tempPrePreOdo = tempPreOdo;
		//	tempPreOdo = tempCurOdo;
		//}
		videoCapture >> frame; 
		if (frame.cols == 0 || frame.rows == 0)
			continue;
		if (++curIndex < 20)
			continue;
		
		if (!a.Calc(frame, R, t, w)){
			cout << "检测失败!" << endl;
			camOrOdo = true;
				imwrite((num2str(curIndex) + ".png").c_str(), frame);
		}
		else{
			//while (validCameraPose);
			if (!validCameraPose){
				//t.at<double>(0, 0) = t.at<double>(0, 0) - Ra*cos(angle - jiao);
				//t.at<double>(1, 0) = t.at<double>(1, 0) - Ra*sin(angle - jiao);
				camOrOdo = false;
				xpos = t.at<double>(0, 0) - Ra*cos((w - jiao) / 180 * PI);
				ypos = t.at<double>(1, 0) - Ra*sin((w - jiao) / 180 * PI);
			cita = w;
			validCameraPose = true;
			}
		}
		//c = cvWaitKey(20);
		//imshow("video", frame);
		//imshow("disvideo", distortframe);
		
		if (c == 32){
			//imwrite((num2str(index) + ".png").c_str(), frame);
			//++index;
		}
		//cout<<int(c)<<endl;
		if (c == 27)break;
	}

	return;
}