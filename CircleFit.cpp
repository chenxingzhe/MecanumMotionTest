#include "CircleFit.h"
#include <time.h>

using namespace std;

//对数据进行拟合
int CircleFit::fit_circle(vector<PointOnCircle> points, double &A, double &B, double &R)
{
	double X1, X2, X3, Y1, Y2, Y3, X1Y1, X1Y2, X2Y1;
	double C, D, E, G, H, N;
	double a, b, c;

	//拟合数据数量判断
	if (points.size() < 3){
		printf("Error: fit data number is less than 3!\n");
		return 0;
	}

	X1 = X2 = X3 = Y1 = Y2 = Y3 = X1Y1 = X1Y2 = X2Y1 = 0;

	for (int i = 0; i < points.size(); ++i){
		X1 = X1 + points[i].x;
		Y1 = Y1 + points[i].y;
		X2 = X2 + points[i].x * points[i].x;
		Y2 = Y2 + points[i].y * points[i].y;
		X3 = X3 + points[i].x * points[i].x*points[i].x;
		Y3 = Y3 + points[i].y * points[i].y*points[i].y;
		X1Y1 = X1Y1 + points[i].x * points[i].y;
		X1Y2 = X1Y2 + points[i].x * points[i].y*points[i].y;
		X2Y1 = X2Y1 + points[i].x * points[i].x*points[i].y;
	}

	N = points.size();
	C = N * X2 - X1 * X1;
	D = N * X1Y1 - X1 * Y1;
	E = N * X3 + N * X1Y2 - (X2 + Y2) * X1;
	G = N * Y2 - Y1 * Y1;
	H = N * X2Y1 + N * Y3 - (X2 + Y2) * Y1;

	a = (H * D - E * G) / (C * G - D * D);
	b = (H * C - E * D) / (D * D - G * C);
	c = -(a * X1 + b * Y1 + X2 + Y2) / N;

	A = a / (-2);
	B = b / (-2);
	R = sqrt(a * a + b * b - 4 * c) / 2;
	return 0;
}

bool CircleFit::ranSacCircleFit(vector<PointOnCircle> points, double &A, double &B, double &R){
	srand((int)time(0));
	int sampleNum = 5;
	vector<PointOnCircle> tempPoint;
	double inrate = 0;
	bool iniFlag = true;
	double inThres = 50;
	int runTime = 0;
	double inRateThres = 0.8;
	int MaxIteratorTime = 20;
	while (inrate < inRateThres || runTime<3){
		if (runTime>MaxIteratorTime)
			return false;
		if (iniFlag || inrate < inRateThres){
			iniFlag = false;
			for (int i = 0; i < sampleNum; ++i)
				tempPoint.push_back(points[rand() % points.size()]);
		}
		fit_circle(tempPoint, A, B, R);
		//check inrate, push the inpoint into the next iterate
		tempPoint.clear();
		inrate = 0;
		for (int i = 0; i < points.size(); ++i){
			double tempR = sqrt((points[i].x - A)*(points[i].x - A) + (points[i].y - B)*(points[i].y - B));
			if (abs(R - tempR) < inThres){
				tempPoint.push_back(points[i]);
				++inrate;
			}
		}
		inrate /= points.size();
		++runTime;
	}
	return true;
}
