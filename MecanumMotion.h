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