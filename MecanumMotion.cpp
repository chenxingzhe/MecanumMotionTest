//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////
// include files
#include "NRF_Messages/MessageInteraction.h"
#include "common/param_reader.h"
#include "common/utils.h"

USING_NAMESPACE_NEAT_COMMON_ALL();

#include "MecanumMotion.h"
#include "MecanumMotion/MecanumModule.h"
#include "MecanumMotion/OdometryCalculation.h"
#include "MecanumMotion/OdometryCapture.h"
#include "datatype/speeds.h"
#include "datatype/odometer.h"
#include <windows.h>
#include "CircleFit.h"
// Other Needed
#include <Windows.h>
#include <fstream>
#include <iostream>
//#include <opencv2/opencv.hpp>
// MCU Control;
#include "mmsystem.h"
#pragma comment(lib,"winmm")

#include <Eigen/Dense>

using namespace std;

#ifndef PI
#define PI 3.1415926
#endif

#define EPSMEC 10
#define pp 40
#define vs 100
#define num 3
double sleeptime = 2.5;

extern boost::mutex curOdoPosMutex;
extern CurPos curOdoPos;
// camera Module

//#include "calclandmark.h"
//#include <videoio.hpp>

//////////////////////////////////////////////////////////////////////////
// define the module for capture odometer
COdometerCapture odoCap;

extern int DataProcessFlag;
extern bool validCameraPose;
extern double xpos;
extern double ypos;
extern double cita;
double worldv=0, worldw=0;
double vthre=5, wthre=PI/80;
//////////////////////////////////////////////////////////////////////////
// define the callback function
void SetSpeedCallBack(const Home_RobotSpeed &speed)
{
    neat::RobotSpeed my_speed;
    my_speed.vx = speed.vx() / 10.0f;			// mm/s -> cm/s
    my_speed.vy = speed.vy() / 10.0f;			// mm/s -> cm/s
    my_speed.w  = speed. w()*180.0 / PI;		// rad/s -> deg/s
    odoCap.SetCurSpeed(my_speed);

    return;
}
// define the Odometer Publish Function
void OdometerPublisher(neat::Odometer& odometer)
{
	Home_Odometer odometerData;
	odometerData.set_x(odometer.x);
	odometerData.set_y(odometer.y);
	odometerData.set_angle(odometer.angle);
	Home_VelocityOmni* lv = new Home_VelocityOmni();
	Home_VelocityOmni* gv = new Home_VelocityOmni();
	Home_WheelsOmni* wheels = new Home_WheelsOmni();
	lv->set_vx(odometer.lv.vx);
	lv->set_vy(odometer.lv.vy);
	lv->set_w(odometer.lv.w);
	gv->set_vx(odometer.gv.vx);
	gv->set_vy(odometer.gv.vy);
	gv->set_w(odometer.gv.w);
	wheels->set_w1(odometer.wheels.w1);
	wheels->set_w2(odometer.wheels.w2);
	wheels->set_w3(odometer.wheels.w3);
	wheels->set_w4(odometer.wheels.w4);
	odometerData.set_allocated_lv(lv);
	odometerData.set_allocated_gv(gv);
	odometerData.set_allocated_wheels(wheels);
	odometerData.set_cycle(odometer.cycle);
	SubPubManager::Instance()->m_odometer.GetPublisher()->publish(odometerData);
}
void speedSlowIncrease(Home_RobotSpeed &cur_speed, double v)
{
	cout << "world:" << worldv << " ev:" << v << endl;
	//worldw = 0;
	if (worldv == v)
	{
		cur_speed.set_vx(worldv);
		cur_speed.set_vy(0);
		cur_speed.set_w(worldw);
		SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
		Sleep(sleeptime);
	}
			
	    if (worldv < v)
		{
			worldv = worldv + vthre;
			cur_speed.set_vx(worldv);
			cur_speed.set_vy(0);
			cur_speed.set_w(worldw);
			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
			Sleep(sleeptime);
		}
		else
		{
			worldv = worldv - vthre;
			cur_speed.set_vx(worldv);
			cur_speed.set_vy(0);
			cur_speed.set_w(worldw);
			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
			Sleep(sleeptime);
		}
	
}
void citaSlowIncrease(Home_RobotSpeed &cur_speed, double w)
{
	
	worldw = w;
	speedSlowIncrease(cur_speed, 50);
	/*if (worldw == w)
	{
	speedSlowIncrease(cur_speed, 0);

	}
	if (worldw < w)
	{
	worldw = worldw + wthre;
	speedSlowIncrease(cur_speed, 0);
	}
	else
	{
	worldw = worldw - wthre;
	speedSlowIncrease(cur_speed, 0);
	}*/
}
int bigCita(Home_RobotSpeed &cur_speed, double dw,double w)
{
	cout << "big cita" << endl; 
	double detx, dety, detw, detxc, detyc;
	detw = cita / 180 * PI - w;
	detxc = detx*cos(-w) - dety*sin(-w);
	detyc = detx*sin(-w) + dety*cos(-w);
	if (detw > PI)
		detw -= 2 * PI;
	if (detw < -PI)
		detw += 2 * PI;
	double detw2 = -(dw - detw);
	if (detw2 >= PI)
		detw2 -= 2 * PI;
	if (detw2 <= -PI)
		detw2 += 2 * PI;
	worldv = 0;
	if (fabs(detw2 * 180 / PI) > EPSMEC / 2)
	{
		if (detw2 < 0)
		{
			worldw = PI / 20;
			cur_speed.set_vx(worldv);
			cur_speed.set_vy(0);
			cur_speed.set_w(worldw);
			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
		}
		else
		{
			worldw =- PI / 20;
			cur_speed.set_vx(worldv);
			cur_speed.set_vy(0);
			cur_speed.set_w(worldw);
			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
		}
	}
	if (detw2 * 180 / PI < EPSMEC / 2)
		return 1;
	return 0;

}
int refineOdometer(Home_RobotSpeed &cur_speed,Home_Odometer &cur_odo, double x, double y, double w, double dx, double dy)
{
	neat::Odometer odometerData;
	double detx, dety, detw, detxc, detyc;
		odoCap.GetCurOdometry(odometerData);
		curOdoPosMutex.lock();
		curOdoPos = CurPos(odometerData.x, odometerData.y, odometerData.angle);
		curOdoPosMutex.unlock();

		detx = odometerData.x - x;
		dety = odometerData.y - y;
		detw = odometerData.angle - w;
		detxc = detx*cos(-w) - dety*sin(-w);
		detyc = detx*sin(-w) + dety*cos(-w);
		if (detw > PI)
			detw -= 2 * PI;
		if (detw < -PI)
			detw += 2 * PI;
		double vx = dx - detxc;
		double vy = dy - detyc;
		double dw2 = 0;
		if (vx > 0 && vy > 0)
			dw2 = atan(fabs(vy / vx));
		else if (vx<0 && vy > 0)
			dw2 = PI - atan(fabs(vy / vx));
		else if (vx > 0 && vy < 0)
			dw2 = -atan(fabs(vy / vx));
		else if (vx < 0 && vy < 0)
			dw2 = atan(fabs(vy / vx)) - PI;
		if (vx == 0 && vy>0)
			dw2 = PI / 2;
		if (vx == 0 && vy < 0)
			dw2 = -PI / 2;
		double redw2 = dw2 + PI;
		if (redw2 >= PI)
			redw2 -= 2 * PI;
		if (redw2 <= -PI)
			redw2 += 2 * PI;
		double detw2 = -(dw2 - detw);
		if (detw2 >= PI)
			detw2 -= 2 * PI;
		if (detw2 <= -PI)
			detw2 += 2 * PI;
		double redetw2 = -(redw2 - detw);
		if (redetw2 >= PI)
			redetw2 -= 2 * PI;
		if (redetw2 <= -PI)
			redetw2 += 2 * PI;
		bool flag;
		if (fabs(detw2)<fabs(redetw2))
			flag = false;
		else
		{
			detw2 = redetw2;
			flag = true;
		}
		if (flag)
		{
			if (fabs(detw2 * 180 / PI) > EPSMEC/2)
			{
				if (detw2 < 0)
				{
					cur_speed.set_vx(0);
					cur_speed.set_vy(0);
					cur_speed.set_w(PI / 10);
					SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
				}
				else
				{
					cur_speed.set_vx(0);
					cur_speed.set_vy(0);
					cur_speed.set_w(-PI / 10);
					SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
				}
			}
			else
			{
				cur_speed.set_vx(-100);
				cur_speed.set_vy(0);
				cur_speed.set_w(0);
				SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
			}
		}
		else
		{
			if (fabs(detw2 * 180 / PI) > EPSMEC/2)
			{
				if (detw2 < 0)
				{
					cur_speed.set_vx(0);
					cur_speed.set_vy(0);
					cur_speed.set_w(PI / 10);
					SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
				}
				else
				{
					cur_speed.set_vx(0);
					cur_speed.set_vy(0);
					cur_speed.set_w(-PI / 10);
					SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
				}
			}
			else
			{
				cur_speed.set_vx(100);
				cur_speed.set_vy(0);
				cur_speed.set_w(0);
				SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
			}
		}
		
		cout << "error refine" << endl;
		if (fabs(detxc - dx) < EPSMEC&&fabs(detyc - dy) < EPSMEC)
		{
			return 1;
		}
		return 0;

}

int refineCamera(Home_RobotSpeed &cur_speed, Home_Odometer &cur_odo, double x, double y, double w, double dx, double dy)
{

	/*neat::Odometer odometerData;
	odoCap.GetCurOdometry(odometerData);*/
	//double speed = 20;
	double detx, dety, detw, detxc, detyc;
	
	detx = xpos/10 - x;
	dety = ypos/10 - y;
	detw = cita/180*PI - w;
	detxc = detx*cos(-w) - dety*sin(-w);
	detyc = detx*sin(-w) + dety*cos(-w);
	if (detw > PI)
		detw -= 2 * PI;
	if (detw < -PI)
		detw += 2 * PI;
	double vx = dx - detxc;
	double vy = dy - detyc;
	double dw2 = 0;
	if (vx > 0 && vy > 0)
		dw2 = atan(fabs(vy / vx));
	else if (vx<0 && vy > 0)
		dw2 = PI - atan(fabs(vy / vx));
	else if (vx > 0 && vy < 0)
		dw2 = -atan(fabs(vy / vx));
	else if (vx < 0 && vy < 0)
		dw2 = atan(fabs(vy / vx)) - PI;
	if (vx == 0 && vy>0)
		dw2 = PI / 2;
	if (vx == 0 && vy < 0)
		dw2 = -PI / 2;
	double redw2 = dw2 + PI;
	if (redw2 >= PI)
		redw2 -= 2 * PI;
	if (redw2 <= -PI)
		redw2 += 2 * PI;
	double detw2 = -(dw2 - detw);
	if (detw2 >= PI)
		detw2 -= 2 * PI;
	if (detw2 <= -PI)
		detw2 += 2 * PI;
	double redetw2 = -(redw2 - detw);
	if (redetw2 >= PI)
		redetw2 -= 2 * PI;
	if (redetw2 <= -PI)
		redetw2 += 2 * PI;
	bool flag;
	if (fabs(detw2)<fabs(redetw2))
		flag = false;
	else
	{
		detw2 = redetw2;
		flag = true;
	}
	cout << "角度差："<<detw2 <<"vx:"<<vx<<"vy:"<<vy<< endl;
	if (flag)
	{
		if (fabs(detw2 * 180 / PI) > EPSMEC/3)
		{
			if (detw2 < 0)
			{
				
				worldv = 0;
				worldw = PI / pp;
				cur_speed.set_vx(worldv);
				cur_speed.set_vy(0);
				cur_speed.set_w(worldw);
				SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
			}
			else
			{
				worldv = 0;
				worldw = -PI / pp;
				cur_speed.set_vx(worldv);
				cur_speed.set_vy(0);
				cur_speed.set_w(worldw);
				SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
			}
		}
		else
		{
			//speedSlowIncrease(cur_speed, -50);
			worldv = -50;
			worldw = 0;
			cur_speed.set_vx(worldv);
			cur_speed.set_vy(0);
			cur_speed.set_w(worldw);
			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
		}
	}
	else
	{
		if (fabs(detw2 * 180 / PI) > EPSMEC/3)
		{
			if (detw2 < 0)
			{
				worldv = 0;
				worldw = PI / pp;
				cur_speed.set_vx(worldv);
				cur_speed.set_vy(0);
				cur_speed.set_w(worldw);
				SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
			}
			else
			{
				worldv = 0;
				worldw = -PI / pp;
				cur_speed.set_vx(worldv);
				cur_speed.set_vy(0);
				cur_speed.set_w(worldw);
				SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
			}
		}
		else
		{
			//speedSlowIncrease(cur_speed, 50);
			worldv = 50;
			worldw = 0;
			cur_speed.set_vx(worldv);
			cur_speed.set_vy(0);
			cur_speed.set_w(worldw);
			SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
		}
	}
	
	//cout << "error refine" << endl;
	if (fabs(detxc - dx) < EPSMEC/2&&fabs(detyc - dy) < EPSMEC/2)
	{
		return 1;
	}
	return 0;

}
void goWithOdoAndCam()
{
	double x, y, w, x1, y1, w1;
	double detx, dety, detw, detxc, detyc, detxcc, detycc;
	double detx1, dety1, detw1, detxc1, detyc1;
	double fx, fy, fw, fx1, fy1, fw1;
	double xpos1, ypos1, cita1;
	neat::Odometer odometerData;
	odoCap.GetCurOdometry(odometerData);
	curOdoPosMutex.lock();
	curOdoPos = CurPos(odometerData.x, odometerData.y, odometerData.angle);
	curOdoPosMutex.unlock();
	x = curOdoPos.x;
	y = curOdoPos.y;
	w = curOdoPos.cita;

	while (!validCameraPose){ Sleep(1); }
	x1 = xpos / 10;
	y1 = ypos / 10;
	w1 = cita / 180 * PI;
	validCameraPose = false;
	while (1)
	{
		static ofstream outfile;
		if (!outfile.is_open()) {
			cout << "not open" << endl;
			outfile.open("Compare.txt", ios::out);
		}
		
		if (!validCameraPose){
			//cout << "无有效数据！" << endl;
			odoCap.GetCurOdometry(odometerData);
			curOdoPosMutex.lock();
			curOdoPos = CurPos(odometerData.x, odometerData.y, odometerData.angle);
			curOdoPosMutex.unlock();
			detx = odometerData.x - x;
			dety = odometerData.y - y;
			detw = odometerData.angle - w;
			w = w;
			detxc = detx*cos(-w) - dety*sin(-w);
			detyc = detx*sin(-w) + dety*cos(-w);
			//double jiao = 0.1125;
			double jiao = 0;
			detxcc = detxc*cos(w1 + jiao) - detyc*sin(w1 + jiao);
			detycc = detxc*sin(w1 + jiao) + detyc*cos(w1 + jiao);
			fx = detxcc + x1;
			fy = detycc + y1;
			fw = detw + w1;
			outfile << fx << '\t' << fy << '\t' << fw * 180 / PI << endl;
			Sleep(5);
			continue;
		}
		
		xpos1 = xpos / 10;
		ypos1 = ypos / 10;
		cita1 = cita / 180 * PI;
		detx1 = xpos1 - x1;
		dety1 = ypos1 - y1;
		detw1 = cita1 - w1;
		detxc1 = detx1*cos(-w1) - dety1*sin(-w1);
		detyc1 = detx1*sin(-w1) + dety1*cos(-w1);
		if (detw > PI)
			detw -= 2 * PI;
		if (detw < -PI)
			detw += 2 * PI;
		if (detw1 > PI)
			detw1 -= 2 * PI;
		if (detw1 < -PI)
			detw1 += 2 * PI;
		/*fx = detxc;
		fy = detyc;
		fw = detw;
		fx1 = detxc1;
		fy1 = detyc1;
		fw1 = detw1;*/
	
		if (fw > PI)
			fw -= 2 * PI;
		if (fw < -PI)
			fw += 2 * PI;
		double bili = 0.982;
		fx1 = xpos1*bili;
		fy1 = ypos1*bili;
		fw1 = cita1;
		
		
		outfile << fx1 << '\t' << fy1 << '\t' << fw1 * 180 / PI << endl;
		validCameraPose = false;
		Sleep(10);
		

	}
	
}
void MecanumMotion::doMotionControlWithCamera(){

	Home_RobotSpeed cur_speed;//速度设置
	Home_Odometer cur_odo;//里程计数据读取
	/// 1. Params Initialize
	ParamReader::Instance()->readParams(Params::NavLoc_XML.c_str());
	/// 2. Module initialize
	MECANUM->Initialize();
	if (MECANUM->Succeed()) {
		std::cout << "Mecanum module initialize successfully!" << std::endl;
	}
	else {
		std::cout << "Mecanum module initialize failed!" << std::endl;
		std::cout << "Please check : Power off, or Device invalid ? Press any key to exit!" << endl;
		getchar();
		return;
	}

	/// 3. Setup node
	STD_OUT_DEBUG(Module::Module_Mecanum, "is setting up ...");
	NODE.init(Module::Module_Mecanum);
	SubPubManager::Instance()->m_robotspeed.Initialize(Message::Message_Speed, SetSpeedCallBack);
	SubPubManager::Instance()->m_odometer.Initialize(Message::Message_Odometer, NULL);
	std::cout << "Mecanum node has been setup, wait for 1 seconds ..." << std::endl;
	Sleep(1000);
	STD_OUT_DEBUG(Module::Module_Mecanum, "is on, it's okay now!!!");

	/// 4. Setup timer
	//std::function<void(neat::Odometer&)> fp = OdometerPublisher;
	

	odoCap.SetOdometerPublishFunc(OdometerPublisher);
	odoCap.StartOdometryCapture();
	Sleep(1000);
	//里程计数据读取,走个方格
	double x = 0, y = 0, w = 0, detx = 0, dety = 0, detw = 0, detxc = 0, detyc = 0, detwc = 0;//xc，yc，wc表示坐标系变换之后的值
	double x1 = 0, y1 = 0, w1 = 0, detx1 = 0, dety1 = 0, detw1 = 0, detxc1 = 0, detyc1 = 0, detwc1 = 0;//视觉里程计的变化值
	neat::Odometer odometerData;
	cur_speed.set_vx(0);
	cur_speed.set_vy(0);
	cur_speed.set_w(0);
	SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
	//cin >> flag;
	if (DataProcessFlag == 3)
		goWithOdoAndCam();
	while (DataProcessFlag != 0 && DataProcessFlag != 1 && DataProcessFlag != 2){
		odoCap.GetCurOdometry(odometerData);
		curOdoPosMutex.lock();
		curOdoPos = CurPos(odometerData.x, odometerData.y, odometerData.angle);
		curOdoPosMutex.unlock();
		Sleep(1); //100hz
	};

	std::cout << "It's OK, just go on !" << std::endl;
	/// 5. Wait Node to exit
	Eigen::Matrix3d cam2OdoRT;
	odoCap.GetCurOdometry(odometerData);
	curOdoPosMutex.lock();
	curOdoPos = CurPos(odometerData.x, odometerData.y, odometerData.angle);
	curOdoPosMutex.unlock();
	//SubPubManager::Instance()->m_odometer.GetData(cur_odo);
	x = odometerData.x;
	y = odometerData.y;
	w = odometerData.angle;
	while (!validCameraPose){ Sleep(1); }
	x1 = xpos/10;
	y1 = ypos/10;
	w1 = cita / 180 * PI;
	double xpos1, ypos1, cita1;
	while (DataProcessFlag == 0){
		if (!validCameraPose){
			//cout << "无有效数据！" << endl;
			Sleep(1);
			continue;
		}
		//视觉里程计
		xpos1 = xpos / 10;
		ypos1 = ypos / 10;
		cita1 = cita / 180 * PI;
		detx1 = xpos1 - x1;
		dety1 = ypos1 - y1;
		detw1 = cita1- w1;
		detxc1 = detx1*cos(-w1) - dety1*sin(-w1);
		detyc1 = detx1*sin(-w1) + dety1*cos(-w1);
		if (detw1 > PI)
			detw1 -= 2 * PI;
		if (detw1 < -PI)
			detw1 += 2 * PI;
		//里程计
		odoCap.GetCurOdometry(odometerData);
		curOdoPosMutex.lock();
		curOdoPos = CurPos(odometerData.x, odometerData.y, odometerData.angle);
		curOdoPosMutex.unlock();

		detx = odometerData.x - x;
		dety = odometerData.y - y;
		detw = odometerData.angle - w;
		detxc = detx*cos(-w) - dety*sin(-w);
		detyc = detx*sin(-w) + dety*cos(-w);
		
		if (detw > PI)
			detw -= 2 * PI;
		if (detw < -PI)
			detw += 2 * PI;


		/*cout << "----------------------------Xpos：" << detxc1 << endl;
		cout << "----------------------------Ypos：" << detyc1 << endl;
		cout << "----------------------------cita：" << detw1 << endl;*/
		static ofstream outfile;
		if (!outfile.is_open()) {
			cout << "not open" << endl;
			outfile.open("Camera_Capture.txt", ios::out);
		}
		outfile //<< m_cal_time_step
			<<detxc1 << '\t' << detyc1 << '\t' << detw1 * 180 / PI << endl;
		//outfile //<< m_cal_time_step
			//<< "odocap:" << detxc << '\t' << detyc << '\t' << detw * 180 / PI << endl;
		Sleep(5);
		validCameraPose = false;

	}
	bool initializedFlag = false;
	bool finish = true;
	int count = 0;
	double dx, dy, dw;
	bool kk = false;
	double prex=0, prey=0, prew=0;
	double sumx = 0, sumy = 0, sumw = 0;
	queue<double> q;
	while (!q.empty())
	{
		q.pop();
	}
	for (int i = 0; i < 15; i++)
	{
		q.push(0);
	}
	double detw2 = 0;
	while (DataProcessFlag == 1)
	{

		if (finish)
		{

			static ifstream infile;
			if (!infile.is_open()) {
				infile.open("Camera_Capture.txt", ios::in);
			}
			cout << "读取数据" << endl;
			if (!(infile >> dx >> dy >> dw))
			{
				cout << "end" << endl;
				infile.close();
				infile.open("Camera_Capture.txt", ios::in);
				infile >> dx >> dy >> dw;

			}
			dw = dw / 180 * PI;
			/*sumx -= q.front();
			q.pop();
			sumy -= q.front();
			q.pop();
			sumw -= q.front();
			q.pop();
			sumx += dx;
			sumy += dy;
			sumw += dw;
			q.push(dx);
			q.push(dy);
			q.push(dw);
			dx = sumx / 5;
			dy = sumy / 5;
			dw = sumw / 5;*/
			kk = true;
			if (fabs(detxc1 - dx) < EPSMEC * num && fabs(detyc1 - dy) < EPSMEC * num &&detw2 * 180 / PI < EPSMEC * num / 3)
			{
				finish = true;
				continue;
			}
			static ofstream outfile;
			if (!outfile.is_open()) {
				cout << "not open" << endl;
				outfile.open("error.txt", ios::out);
			}
			outfile //<< m_cal_time_step
				<< dx << '\t' << dy << '\t' << dw << endl;

			//cout << dx << " " << dy << " " << dw<< endl;
			//dw = dw / 180 * PI;
			finish = false;
		}
		if (!validCameraPose){
			//cout << "无有效数据！" << endl;
			Sleep(1);
			continue;
		}
		//里程计
		/*odoCap.GetCurOdometry(odometerData);
		detx = odometerData.x - x;
		dety = odometerData.y - y;
		detw = odometerData.angle - w;
		detxc = detx*cos(-w) - dety*sin(-w);
		detyc = detx*sin(-w) + dety*cos(-w);

		if (detw > PI)
		detw -= 2 * PI;
		if (detw < -PI)
		detw += 2 * PI;
		double vx = dx - detxc;
		double vy = dy - detyc;
		double dw2 = 0;
		if (vx>0 && vy > 0)
		dw2 = atan(fabs(vy / vx));
		else if (vx<0 && vy > 0)
		dw2 = PI - atan(fabs(vy / vx));
		else if (vx>0 && vy < 0)
		dw2 = -atan(fabs(vy / vx));
		else if (vx<0 && vy < 0)
		dw2 = atan(fabs(vy / vx)) - PI;
		if (vx == 0 && vy>0)
		dw2 = PI / 2;
		if (vx == 0 && vy<0)
		dw2 = -PI / 2;
		double detw2 = -(dw - detw);//按照里程计走
		if (detw2 >= PI)
		detw2 -= 2 * PI;
		if (detw2 <= -PI)
		detw2 += 2 * PI;
		//double detw2 = -(dw2 - detw);//走斜边
		cout << dw << " " << detw << endl;*/
		//视觉里程计
		xpos1 = xpos / 10;
		ypos1 = ypos / 10;
		cita1 = cita / 180 * PI;
		detx1 = xpos1 - x1;
		dety1 = ypos1 - y1;
		detw1 = cita1 - w1;
		cout << detx1 << " " << dety1 << " " << detw1 << endl;
		detxc1 = detx1*cos(-w1) - dety1*sin(-w1);
		detyc1 = detx1*sin(-w1) + dety1*cos(-w1);
		if (detw1 > PI)
			detw1 -= 2 * PI;
		if (detw1 < -PI)
			detw1 += 2 * PI;
		double dw2 = 0;
		double vx = dx - detxc1;
		double vy = dy - detyc1;
		if (vx > 0 && vy > 0)
			dw2 = atan(fabs(vy / vx));
		else if (vx<0 && vy > 0)
			dw2 = PI - atan(fabs(vy / vx));
		else if (vx > 0 && vy < 0)
			dw2 = -atan(fabs(vy / vx));
		else if (vx < 0 && vy < 0)
			dw2 = atan(fabs(vy / vx)) - PI;
		if (vx == 0 && vy>0)
			dw2 = PI / 2;
		if (vx == 0 && vy < 0)
			dw2 = -PI / 2;
		detw2 = -(dw2 - detw1);//走斜边
		//double detw2 = -(dw - detw1);//按照里程计走
		if (detw2 >= PI)
			detw2 -= 2 * PI;
		if (detw2 <= -PI)
			detw2 += 2 * PI;
		/*	static ofstream outfile;
		if (!outfile.is_open()) {
		outfile.open("error.txt", ios::out);
		}
		outfile //<< m_cal_time_step
		<< fabs(detxc - dx)
		<< '\t'
		<< fabs(detyc - dy)
		<< '\t'
		<< fabs(detw2 * 180 / PI)

		<< endl;*/

		/*if (fabs(detxc1 - dx) < EPSMEC&&fabs(detyc1 - dy) < EPSMEC&&fabs(detw2 * 180 / PI) < EPSMEC)
		{
		finish = true;
		}*/
		if (fabs(detw2 * 180 / PI) > EPSMEC * 3)
		{

			while (1)
			{
				if (!validCameraPose){
					//cout << "无有效数据！" << endl;
					Sleep(1);
					continue;
				}
				int flag = bigCita(cur_speed, dw2, w1);
				validCameraPose = false;
				if (flag)
					break;
			}
			continue;
		}
		if (fabs(detxc1 - dx) < EPSMEC&&fabs(detyc1 - dy) < EPSMEC)
		{
			finish = true;
		}
		else
		{


			cout << "误差：" << fabs(detxc1 - dx) << " " << fabs(detyc1 - dy) << " " << fabs(detw2 * 180 / PI) << endl;
			//cout << "dx:" << dx << "dy:" << dy << "detxc1:" << detxc1 << "detyc1:" << detyc1;

			//||fabs(detxc - dx) > EPS*10||fabs(detyc - dy) > EPS*10
			if (fabs(detxc1 - dx) > EPSMEC * (num + 1) || fabs(detyc1 - dy) > EPSMEC * (num + 1))
			{

				/*
				cur_speed.set_vx(0);
				cur_speed.set_vy(0);
				cur_speed.set_w(0);
				SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
				*/
				cout << "refine!!!" << endl;
				while (1)
				{
					if (!validCameraPose){
						//cout << "无有效数据！" << endl;
						Sleep(1);
						continue;
					}
					int flag = refineCamera(cur_speed, cur_odo, x1, y1, w1, dx, dy);
					validCameraPose = false;
					if (flag)
						break;
				}
				if (kk == false)
				{
					finish = true;
				}
				kk = false;


				//cout << "error" << endl;
				//finish = true;
				//continue;
			}
			if (fabs(detw2 * 180 / PI) < EPSMEC / 3)
			{
				//不能使用detxc - dx的正负判断~
				speedSlowIncrease(cur_speed, vs);
				/*cur_speed.set_vx(vs);
				cur_speed.set_vy(0);
				cur_speed.set_w(0);
				SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);*/



			}
			else
			{

				if (detw2 < 0)
				{
					/*cur_speed.set_vx(0);
					cur_speed.set_vy(0);
					cur_speed.set_w(PI / 20);
					SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);*/
					citaSlowIncrease(cur_speed, PI / pp);

				}
				else
				{
					/*cur_speed.set_vx(0);
					cur_speed.set_vy(0);
					cur_speed.set_w(-PI / 20);
					SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);*/
					citaSlowIncrease(cur_speed, -PI / pp);
				}

			}
		}
		validCameraPose = false;
	}
	if (DataProcessFlag == 2)
	{
		
		static ofstream outfile;
		double coordinate[3][500];
		vector<PointOnCircle> points;
		if (!outfile.is_open()) {
			cout << "not open" << endl;
			outfile.open("sss.txt", ios::out);
		}
		int count = 0;
		int big = -1;
		double comp=-9999;
		bool f = false;
		while (count < 400)
		{
			
			if (!validCameraPose){
				//cout << "无有效数据！" << endl;
				Sleep(1);
				continue;
			}
				cur_speed.set_vx(0);
				cur_speed.set_vy(0);
				cur_speed.set_w(PI / 20);
				SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);		
			outfile << xpos << '\t' << ypos << '\t' << cita << endl;
			coordinate[0][count] = xpos;
			coordinate[1][count] = ypos;
			coordinate[2][count] = cita;
			points.push_back(PointOnCircle(xpos, ypos, cita/180*PI));
			if (comp < xpos)
			{
				big = count;
				comp = xpos;
			}
			validCameraPose = false;
			count++;
		}
		cout << "complete!!" <<endl;
		outfile.close();
		cur_speed.set_vx(0);
		cur_speed.set_vy(0);
		cur_speed.set_w(0);
		SubPubManager::Instance()->m_robotspeed.GetPublisher()->publish(cur_speed);
		double A, B, R;
		CircleFit circleFit;
		cout << circleFit.ranSacCircleFit(points, A, B, R) << endl;
		//printf("fit data num is %d, A = %.3f, B = %.3f, R = %.3f\n", num, A, B, R);
		cout << "R:" << R << " jiao:" << coordinate[2][big] << endl;
		ofstream Rfile;
		if (!Rfile.is_open()) {
			cout << "not open" << endl;
			Rfile.open("Rjiao.txt", ios::out);
		}
		Rfile << R << '\t' << coordinate[2][big] << endl;
		Rfile.close();


	}
	NODE.spin();

	return;
}
