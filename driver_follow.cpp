/***************************************************************************

    file                 : user2.cpp
    author            : Xuangui Huang
    email              : stslxg@gmail.com
    description    :  user module for CyberFollower

 ***************************************************************************/

/*      
     WARNING !
     
	 DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_follow.h"
#include "time.h"


static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_follow(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_follow";	// name of the module (short).
	modInfo[0].desc    =  "user module for CyberFollower" ;	// Description of the module (can be long).
	modInfo[0].fctInit = InitFuncPt;			// Init function.
	modInfo[0].gfId    = 0;
	modInfo[0].index   = 0;
	return 0;
}

// Module interface initialization.
static int InitFuncPt(int, void *pt)
{
	tUserItf *itf = (tUserItf *)pt;
	itf->userDriverGetParam = userDriverGetParam;
	itf->userDriverSetParam = userDriverSetParam;
	return 0;
}

/*
     WARNING!

	 DO NOT MODIFY CODES ABOVE!
*/

/* 
    define your variables here.
    following are just examples
*/
static float _midline[200][2];
static float _yaw, _yawrate, _speed, _acc, _width, _rpm;
static int _gearbox;
static float _Leader_X, _Leader_Y;

static void userDriverGetParam(float LeaderXY[2], float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm){
	/* write your own code here */
	
	_Leader_X = LeaderXY[0];
	_Leader_Y = LeaderXY[1];
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;
	

	/* you can modify the print code here to show what you want */
	printf("speed %.3f Leader XY(%.3f, %.3f) \n", _speed,  _Leader_X,_Leader_Y);
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){
	/* write your own code here */
	
	if (_Leader_X*_Leader_X + _Leader_Y*_Leader_Y < 100) 
	{
		*cmdAcc = 0;
		*cmdBrake = 1;
		*cmdSteer =atan2( _Leader_X, _Leader_Y)/3.14;
		*cmdGear = 0;
	} 
	else if (_Leader_X*_Leader_X + _Leader_Y*_Leader_Y >= 100&&_Leader_X*_Leader_X + _Leader_Y*_Leader_Y < 400) 
	{
		*cmdAcc = 0;
		*cmdBrake = 1;
		*cmdSteer =atan2( _Leader_X*0.7+_midline[15][0]*0.3, _Leader_Y*0.7+_midline[15][1]*0.3)/3.14;
		*cmdGear = 0;
	} 
	else if(_Leader_X*_Leader_X + _Leader_Y*_Leader_Y < 900&&_Leader_X*_Leader_X + _Leader_Y*_Leader_Y >= 400)
	{
		if(_speed>100)
		{
			*cmdAcc = 0;
			*cmdBrake = 1;
			*cmdSteer = (_yaw -12*atan2(_Leader_X*0.5+_midline[20][0]*0.5, _Leader_Y*0.5+_midline[20][1]*0.5))/3.14 ;
			*cmdGear = 2;
		}
		else 
		{
			*cmdAcc = 0.8;
			*cmdBrake = 0;
			*cmdSteer = (_yaw -8*atan2( _Leader_X, _Leader_Y))/3.14;
			if(_speed<69) *cmdGear = 1;
			if(_speed>71) *cmdGear = 2;
		}
	}
	else if(_Leader_X*_Leader_X + _Leader_Y*_Leader_Y <1600&&_Leader_X*_Leader_X + _Leader_Y*_Leader_Y >= 900)
	{
		if(_speed>120)
		{
			*cmdAcc = 0;
			*cmdBrake = 1;
			*cmdSteer = (_yaw -4*atan2( _midline[30][0], _midline[30][1]))/3.14 ;
			*cmdGear = 2;
		}
		else 
		{
			*cmdAcc = 0.8;
			*cmdBrake = 0;
			*cmdSteer = (_yaw -8*atan2(  _midline[30][0], _midline[30][1]))/3.14 ;
			if(_speed<69) *cmdGear = 1;
			if(_speed>71) *cmdGear = 2;
		}
	}
	else if(_Leader_X*_Leader_X + _Leader_Y*_Leader_Y <2500&&_Leader_X*_Leader_X + _Leader_Y*_Leader_Y >= 1600)
	{
		if(_speed>140)
		{
			*cmdAcc = 0;
			*cmdBrake = 1;
			*cmdSteer = (_yaw -4*atan2( _midline[30][0]-4, _midline[30][1]))/3.14 ;
			*cmdGear = 3;
		}
		else 
		{
			*cmdAcc = 0.8;
			*cmdBrake = 0;
			*cmdSteer = (_yaw -8*atan2(  _midline[30][0], _midline[30][1]))/3.14 ;
			if(_speed<69) *cmdGear = 1;
			if(_speed>71&&_speed<122) *cmdGear = 2;
			if(_speed>124) *cmdGear = 3;
		}
	}
	else if(_Leader_X*_Leader_X + _Leader_Y*_Leader_Y >= 2500&&_Leader_X*_Leader_X + _Leader_Y*_Leader_Y<3600)
	{
		if(_speed>160)
		{
			*cmdAcc = 0;
			*cmdBrake = 1;
			*cmdSteer = (_yaw -4*atan2( _midline[30][0]-4, _midline[30][1]))/3.14 ;
			*cmdGear = 3;
		}
		else 
		{
			*cmdAcc = 0.8;
			*cmdBrake = 0;
			*cmdSteer = (_yaw -8*atan2(  _midline[30][0], _midline[30][1]))/3.14 ;
			if(_speed<69) *cmdGear = 1;
			if(_speed>71&&_speed<122) *cmdGear = 2;
			if(_speed>124) *cmdGear = 3;
		}
	}
	else if(_Leader_X*_Leader_X + _Leader_Y*_Leader_Y >=3600)
	{
		if (abs(_midline[120][0]-_midline[0][0])<0.25*_width)
	{
		*cmdAcc = 1;
		*cmdBrake = 0;
		*cmdSteer = (_yaw- 8*atan2(_midline[30][0]-_midline[0][0],_midline[30][1]-_midline[0][1]))/3.14 ;

		if (_speed<120) *cmdGear = 2;
		if (_speed>=122&&_speed<167) *cmdGear = 3;
		if (_speed>=169&&_speed<216) *cmdGear = 4;
		if (_speed>=218)  *cmdGear = 5;
	}
		else
	{
		if (abs(atan2(_midline[80][0]-_midline[50][0],_midline[80][1]-_midline[50][1]))<0.524)
		{
			if (abs(_midline[0][0])>0.25*_width&&(_midline[0][0]*_yaw)>0)
				*cmdSteer = (_yaw- 4*atan2(_midline[40][0],_midline[40][1]))/3.14 ;
			else
				*cmdSteer = (_yaw- 14*atan2(_midline[40][0],_midline[40][1]))/3.14;
			
			
			if (_speed<180)
			{ 
				*cmdAcc = 0.5;
				*cmdBrake = 0;
				if (_speed<120) *cmdGear = 2;
				if (_speed>=122&&_speed<167) *cmdGear = 3;
				if (_speed>=169) *cmdGear = 4;
			}
			else
			{
				*cmdGear = 4;
				*cmdAcc = 0;
				*cmdBrake = 0.8;
			}
		}

		if (abs(atan2(_midline[80][0]-_midline[50][0],_midline[80][1]-_midline[50][1]))>=0.524)
		{
			if (abs(_midline[0][0])>0.25*_width&&(_midline[0][0]*_yaw)>0)
				*cmdSteer = (_yaw- 1*atan2(_midline[30][0]+4*abs(_midline[30][0])/_midline[30][0],_midline[30][1]))/3.14;
			else
				*cmdSteer = (_yaw- 12*atan2(_midline[30][0]+4*abs(_midline[30][0])/_midline[30][0],_midline[30][1]))/3.14 ;
		    *cmdGear = 3;
			if (_speed>130)
			{ 
				*cmdAcc = 0;
				if (_speed>170) 	*cmdBrake = 0.8;
				else	*cmdBrake = 0.3;
			}
			else
			{
				*cmdAcc = 0.5;
				*cmdBrake = 0;
			}
		}
		
		
	}
}
	}