/************************************************************************/
/* Description:	Define the interface for the whole MecanumMotion module	*/
/* Note:        Mainly ported from Home_MecanumMain of NR_All with some 
                modifications.                                          */  
/* Author:      Zhan Jianbo                                             */    
/* Date:        2015-5-8                                                */
/************************************************************************/

#ifndef MECANUM_MOTION_H
#define MECANUM_MOTION_H

#include "common/singleton.h"

USING_NAMESPACE_NEAT_COMMON_ALL();

struct CurPos{
	double x;
	double y;
	double cita;
	bool isValid; //用于表示当前数据是否有效，使用数据前应对该项进行校验
	CurPos() :isValid(false), x(0), y(0), cita(0){}
	CurPos(double x_, double y_, double cita_) :isValid(true), x(x_), y(y_), cita(cita_){}
};

class MecanumMotion
{
public:    
	void doMotionControlWithCamera();
private:    
    MecanumMotion(){};
    ~MecanumMotion(){};
    
    friend NormalSingleton<MecanumMotion>;
};

typedef NormalSingleton<MecanumMotion> MecanumMotionObj;


#endif