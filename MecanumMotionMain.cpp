#include "MecanumMotion.h"
#include <opencv2/opencv.hpp>
#include <opencv2/xphoto.hpp>
#include <windows.h>
#include "calclandmark.h"
#include <fstream>
#include <iostream>
USING_NAMESPACE_NEAT_COMMON_ALL();

using namespace std;
using namespace cv;

DWORD WINAPI motionControl(LPVOID lpParameter);
DWORD WINAPI getCameraPose(LPVOID lpParameter);

int DataProcessFlag=-1;
bool validCameraPose = false;
bool allowNewData = true;
double xpos=0;
double ypos=0;
double cita=0;

int main()
{
	cout << "记录数据输入0，巡线走输入1，圆标定输入2，其他输入任意数：" << endl;
	cin >> DataProcessFlag;

	HANDLE handle1, handle2;
	handle1 = CreateThread(NULL, 0, motionControl, NULL, 0, NULL);
	handle2 = CreateThread(NULL, 0, getCameraPose, NULL, 0, NULL);

	if (NULL == handle1)
	{
		cout << "Create Thread failed !" << endl;
		return -1;
	}
	if (NULL == handle2)
	{
		cout << "Create Thread failed !" << endl;
		return -1;
	}

	CloseHandle(handle1);
	CloseHandle(handle2);

	cout << "The Main Thread is Running !" << endl;

	system("pause");
	return 0;
}

DWORD WINAPI motionControl(LPVOID lpParameter)
{
	MecanumMotion *mctrl = MecanumMotionObj::Instance();

	mctrl->doMotionControlWithCamera();

	return 0;
}

string num2str(int i){
	stringstream s;
	s << i;
	return s.str();
}


DWORD WINAPI getCameraPose(LPVOID lpParameter)
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
	while (1)
	{
		videoCapture >> frame;
		//frame = imread("2.png");
		//cv::xphoto::balanceWhite(frame, frame, cv::xphoto::WHITE_BALANCE_SIMPLE);
		//Mat tempFrame;
		//resize(frame, tempFrame, Size(frame.cols / 2, frame.rows / 2));
		//imshow("org", frame);
		//c = cvWaitKey(5);

		if (++curIndex < 20)
			continue;

		if (!a.Calc(frame, R, t, w)){
			cout << "检测失败!" << endl;
				imwrite((num2str(curIndex) + ".png").c_str(), frame);
		}
		else{
			//while (validCameraPose);
			if (!validCameraPose){
				//t.at<double>(0, 0) = t.at<double>(0, 0) - Ra*cos(angle - jiao);
				//t.at<double>(1, 0) = t.at<double>(1, 0) - Ra*sin(angle - jiao);
				xpos = t.at<double>(0, 0) - Ra*cos((w - jiao) / 180 * PI);
				ypos = t.at<double>(1, 0) - Ra*sin((w - jiao) / 180 * PI);
			cita = w;
			validCameraPose = true;
			}
		}
		
		//imshow("video", frame);
		//imshow("disvideo", distortframe);
		
		if (c == 32){
			//imwrite((num2str(index) + ".png").c_str(), frame);
			//++index;
		}
		//cout<<int(c)<<endl;
		if (c == 27)break;
	}

	return 0;
}