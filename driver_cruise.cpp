/***************************************************************************

    file                 : user1.cpp
    author            : Xuangui Huang
    email              : stslxg@gmail.com
    description    :  user module for CyberCruise

 ***************************************************************************/

/*      
     WARNING !
     
	 DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include "driver_cruise.h"

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_cruise(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_cruise";	// name of the module (short).
	modInfo[0].desc    =  "user module for CyberCruise" ;	// Description of the module (can be long).
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

static void userDriverGetParam(float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm){
	/* write your own code here */
	
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;

	printf("speed %f rpm %f gear %d 10m far target(%f, %f)\n", _speed, _rpm, _gearbox, _midline[10][0], _midline[10][1]);
}

static void userDriverSetParam(float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear)
{
	/* write your own code here */
	if (abs(_midline[120][0]-_midline[0][0])<0.25*_width)//直道判断
	{
		*cmdAcc = 1;
		*cmdBrake = 0;
		*cmdSteer = (_yaw- 8*atan2(_midline[30][0]-_midline[0][0],_midline[30][1]-_midline[0][1]))/3.14 ;
		/*加速上档*/
		if (_speed<120) *cmdGear = 2;
		if (_speed>=122&&_speed<167) *cmdGear = 3;
		if (_speed>=169&&_speed<216) *cmdGear = 4;
		if (_speed>=218)  *cmdGear = 5;
	}
	else
	{
		if (abs(atan2(_midline[80][0]-_midline[50][0],_midline[80][1]-_midline[50][1]))<0.524)//在前方50-80米有<30°的弯
		{
			if (abs(_midline[0][0])>0.25*_width&&(_midline[0][0]*_yaw)<0)//判断小车是否位于赛道外侧（与弯道反向距离中线大于1/4道路宽度）
				*cmdSteer = (_yaw- 4*atan2(_midline[40][0],_midline[40][1]))/3.14 ;//外道时的舵机转向略大于内道
			else
				*cmdSteer = (_yaw- 14*atan2(_midline[40][0],_midline[40][1]))/3.14;
			
			/*减速或加速进弯，过弯速度180*/
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

		if (abs(atan2(_midline[80][0]-_midline[50][0],_midline[80][1]-_midline[50][1]))>=0.524&&abs(atan2(_midline[80][0]-_midline[50][0],_midline[80][1]-_midline[50][1]))<1.047)//前方有30°-60°的弯
		{
			if (abs(_midline[0][0])>0.25*_width&&(_midline[0][0]*_yaw)<0)//外道
				*cmdSteer = (_yaw- 1*atan2(_midline[30][0]+4*abs(_midline[30][0])/_midline[30][0],_midline[30][1]))/3.14;//本意是想沿内道距中线4米处过弯，效果不明显
			else
				*cmdSteer = (_yaw- 12*atan2(_midline[30][0]+4*abs(_midline[30][0])/_midline[30][0],_midline[30][1]))/3.14 ;
		    *cmdGear = 3;
			if (_speed>130)//车速130过弯
			{ 
				*cmdAcc = 0;
				if (_speed>170) 	*cmdBrake = 0.8;//若车速过快加大刹车程度
				else	*cmdBrake = 0.3;
			}
			else
			{
				*cmdAcc = 0.5;
				*cmdBrake = 0;
			}
		}
		
		if (abs(atan2(_midline[80][0]-_midline[50][0],_midline[80][1]-_midline[50][1]))>=1.047&&abs(atan2(_midline[80][0]-_midline[50][0],_midline[80][1]-_midline[50][1]))<2)//前方有60°-114.6°的弯（114.6是调试出来的）
		{
			if (abs(_midline[0][0])>0.25*_width&&(_midline[0][0]*_yaw)<0)//外道
				*cmdSteer = (_yaw- 1*atan2(_midline[30][0]+5*abs(_midline[30][0])/_midline[30][0],_midline[30][1]))/3.14 ;
			else
				*cmdSteer = (_yaw- 12*atan2(_midline[30][0]+5*abs(_midline[30][0])/_midline[30][0],_midline[30][1]))/3.14 ;			
		    *cmdGear = 2;
			if (_speed>80)
			{ 
				*cmdAcc = 0;
				if (_speed>180) 	*cmdBrake = 1;
				else
				{
					if (_speed>120) 	*cmdBrake = 0.8;
					else	*cmdBrake = 0.3;
				}
			}
			else
			{
				*cmdAcc = 0.5;
				*cmdBrake = 0;
			}
		}
		if (abs(atan2(_midline[80][0]-_midline[50][0],_midline[80][1]-_midline[50][1]))>=2)//前方有>114.60°的弯
		{
			*cmdSteer = (_yaw- 1*atan2(_midline[30][0]+5*abs(_midline[30][0])/_midline[30][0],_midline[30][1]))/3.14 ;
			*cmdGear = 1;
			if (_speed>65)
			{ 
				*cmdAcc = 0;
				if (_speed>130) 	*cmdBrake = 1;
				else
				{
					if (_speed>100) 	*cmdBrake = 0.8;
					else	*cmdBrake = 0.3;
				}
			}
			else
			{
				*cmdAcc = 0.5;
				*cmdBrake = 0;
			}
		}
	}
}
