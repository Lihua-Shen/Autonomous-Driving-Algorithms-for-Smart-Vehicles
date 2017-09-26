/***************************************************************************

    file                 : user3.cpp
    author            : Xuangui Huang
    email              : stslxg@gmail.com
    description    :  user module for CyberParking

 ***************************************************************************/

/*      
     WARNING !
     
	 DO NOT MODIFY CODES BELOW!
*/

#ifdef _WIN32
#include <windows.h>
#endif

#include <math.h>
#include "driver_parking.h"

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm);
static void userDriverSetParam (bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear);
static int InitFuncPt(int index, void *pt);

// Module Entry Point
extern "C" int driver_parking(tModInfo *modInfo)
{
	memset(modInfo, 0, 10*sizeof(tModInfo));
	modInfo[0].name    = "driver_parking";	// name of the module (short).
	modInfo[0].desc    =  "user module for CyberParking" ;	// Description of the module (can be long).
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
	printf("OK!\n");
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
static float _yaw, _yawrate, _speed, _acc, _width, _rpm, _lotX, _lotY, _lotAngle, _carX, _carY, _caryaw;
static int _gearbox;
static bool _bFrontIn;

static void userDriverGetParam(float lotX, float lotY, float lotAngle, bool bFrontIn, float carX, float carY, float caryaw, float midline[200][2], float yaw, float yawrate, float speed, float acc, float width, int gearbox, float rpm){
	/* write your own code here */
	
	_lotX = lotX;
	_lotY = lotY;
	_lotAngle = lotAngle;
	_bFrontIn = bFrontIn;
	_carX = carX;
	_carY = carY;
	_caryaw = caryaw;
	for (int i = 0; i< 200; ++i) _midline[i][0] = midline[i][0], _midline[i][1] = midline[i][1];
	_yaw = yaw;
	_yawrate = yawrate;
	_speed = speed;
	_acc = acc;
	_width = width;
	_rpm = rpm;
	_gearbox = gearbox;

	printf("speed %.3f yaw %.2f distance^2 %.3f\n", _speed, _caryaw, (_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY) );
}

static int flag,out=0;
static float k,k2,b,d,dist,s,l,_aimX,_aimY,a,t;
static float x=1;


static void userDriverSetParam (bool* bFinished, float* cmdAcc, float* cmdBrake, float* cmdSteer, int* cmdGear){   
	/* write your own code here */
	_aimX=_lotX+3*cos(_lotAngle);               //定义新的预瞄点为车位中心沿车位朝向外3米处的点
	_aimY=_lotY+3*sin(_lotAngle);

	if(abs(_lotAngle)>(PI/2 - 0.05) && abs(_lotAngle)< (PI/2 + 0.05))                       //计算车辆中心与泊车位所在直线的距离，用以判断是否开始泊车
	{
		dist = abs(_carX - _lotX);
		k=tan(PI/2-0.01);
	}
	else
	{
		k = tan(_lotAngle);
		b = (_lotY - k * _lotX);
		dist = abs(k*_carX - _carY +b)/sqrt(k*k + 1);
	}
	if(_lotX==_carX) k2=(_lotY-_carY)/(_lotX-_carX-0.01);//k2为小车中心与车位中心所在直线斜率
	else k2=(_lotY-_carY)/(_lotX-_carX);

	

	l=(_carX-_aimX) * (_carX - _aimX) + (_carY - _aimY) * (_carY - _aimY);//l为预瞄点与车中心距离平方
	d=(_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY);//d为车位点与车中心距离平方

	if(_lotAngle>0) s=_lotAngle-PI;//将车位朝向反向，方便车头入库计算
	else  s=_lotAngle+PI;
	
	if (_bFrontIn==0) {                                                                         //车头入库
		if(*bFinished==0){
		 if(d<6)
		 {
			 if( _speed < 0.01) {*bFinished = true; flag=9;}  //根据车速判断是否完成泊车
			 a=atan(abs((k-k2)/(1+k*k2)));  //a为小车-车位所在直线与车位朝向所在直线的夹角
				 if(d>0.03&&flag!=8)    //判断小车是否足够接近车位中心
				 {
					 if(d<x) x=d;         //记录小车到车为中心的最小值
					 if((d<1)&&(d-x>0.02)) {*cmdBrake = 1;*cmdAcc = 0;}
					 else
					 {
						 if(abs(s-_caryaw)<0.05) *cmdSteer=0;
						 else *cmdSteer=(20*(s-_caryaw))/3.14;
						 if( _speed >8 ){*cmdBrake = 0.5;*cmdGear = 1;*cmdAcc = 0;}
						 else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
						 flag=1;
					 }
				 }
				 else
				 {
					 flag=8;
					 *cmdSteer=20*(s-_caryaw);
					 if( _speed >= 0.01){*cmdBrake = 0.5;*cmdGear = 1;*cmdAcc = 0;}
					
				 }			
		}
		 else if(l<4)//小车接近预瞄点
		 {
			 a=atan(abs((k-k2)/(1+k*k2)));
			 flag=2;
		
			 if(abs(s-_caryaw)<0.04) *cmdSteer=0;
			 else if(abs(s-_caryaw)<0.17)
			 {
				 if(s-_caryaw>0) *cmdSteer=4*a/3.14;
				 else *cmdSteer=-4*a/3.14;
			 }
			 else if(s-_caryaw>0) *cmdSteer=3*(5*(s-_caryaw)-8*a)/3.14;
			 else  *cmdSteer=3*(5*(s-_caryaw)+8*a)/3.14;
			 if( _speed >10 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
			 else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
		 }
		

		 else if (l<400&&flag>=3){	
		    if((t<0.05&&l< 180 && dist < 8)||(t<0.12&&t>=0.05&&l< 210 && dist < 8.8)||(t>=0.12&&l<150&&dist<9.5)) //根据小车20米处角速度决定什么位置进站
					{
						
						*cmdSteer = 1;
						if( _speed >20 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
						else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
						flag = 3;		
					}
			else{                 //减速缓冲
				*cmdSteer = (_yaw -atan2( _midline[10][0]+4,_midline[10][1]))/3.14;
				if( _speed > 30 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
				else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			}

				}
		
			
		 else if (l < 8000) { //到一定范围时，将车子调整至道路右侧，增加转弯半径
			*cmdSteer = (_yaw -atan2( _midline[10][0]+4,_midline[10][1]))/3.14;
			if( _speed > 50 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			flag = 4;
			t=abs(_yawrate);//记录小车距预瞄点20米处时小车的角速度
		}
		else {			                                                                         //其它路段按巡线方式行驶
			*cmdAcc = 1;
		    *cmdBrake = 0;
		    *cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14 ;
		    *cmdGear = 1;
			flag = 5;
			
	    }

	}
		else                   //开始出库
		{
			if (d < 1.5 ) {    //无转向倒车
			*cmdGear = -1;
			*cmdSteer = 0 ;
			*cmdBrake = 0;
			*cmdAcc = 0.8;
			out=1;
		}else if (d < 10&&out <= 2) { //无转向倒车，适当加速
			*cmdGear = -1;
			*cmdSteer =0 ;
			*cmdBrake = 0;
			*cmdAcc = 1;
			out = 2;
		} else if (d < 90&out <= 3 ){//远离停车位时，给一个大的转向			
		   *cmdGear = -1;
			*cmdSteer = 1;
			*cmdBrake = 0;
			*cmdAcc = 0.1;
			out = 3;
		} else if (d < 5000&&out <= 4) { //准备出站
			*cmdSteer = (_yaw -atan2( _midline[10][0],_midline[10][1]))/3.14;
			*cmdGear = 1;
			if( _speed <0 ){*cmdBrake = 1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdAcc = 0.8;}
			out = 4;
		}
		}
	}
	printf("Steer:%.2f\tflag:%d\tout:%.d\tdist:%.2f\tlotAngle:%.2f\tyaw:%.2f\ta:%.2f\td:%.2f\tl:%.2f\tt:%.2f\n",*cmdSteer,flag,out,dist,s,s-_caryaw,a,d,l,t);
}
