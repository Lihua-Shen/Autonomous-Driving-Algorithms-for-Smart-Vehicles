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
	_aimX=_lotX+3*cos(_lotAngle);               //�����µ�Ԥ���Ϊ��λ�����س�λ������3�״��ĵ�
	_aimY=_lotY+3*sin(_lotAngle);

	if(abs(_lotAngle)>(PI/2 - 0.05) && abs(_lotAngle)< (PI/2 + 0.05))                       //���㳵�������벴��λ����ֱ�ߵľ��룬�����ж��Ƿ�ʼ����
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
	if(_lotX==_carX) k2=(_lotY-_carY)/(_lotX-_carX-0.01);//k2ΪС�������복λ��������ֱ��б��
	else k2=(_lotY-_carY)/(_lotX-_carX);

	

	l=(_carX-_aimX) * (_carX - _aimX) + (_carY - _aimY) * (_carY - _aimY);//lΪԤ����복���ľ���ƽ��
	d=(_carX-_lotX) * (_carX - _lotX) + (_carY - _lotY) * (_carY - _lotY);//dΪ��λ���복���ľ���ƽ��

	if(_lotAngle>0) s=_lotAngle-PI;//����λ�����򣬷��㳵ͷ������
	else  s=_lotAngle+PI;
	
	if (_bFrontIn==0) {                                                                         //��ͷ���
		if(*bFinished==0){
		 if(d<6)
		 {
			 if( _speed < 0.01) {*bFinished = true; flag=9;}  //���ݳ����ж��Ƿ���ɲ���
			 a=atan(abs((k-k2)/(1+k*k2)));  //aΪС��-��λ����ֱ���복λ��������ֱ�ߵļн�
				 if(d>0.03&&flag!=8)    //�ж�С���Ƿ��㹻�ӽ���λ����
				 {
					 if(d<x) x=d;         //��¼С������Ϊ���ĵ���Сֵ
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
		 else if(l<4)//С���ӽ�Ԥ���
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
		    if((t<0.05&&l< 180 && dist < 8)||(t<0.12&&t>=0.05&&l< 210 && dist < 8.8)||(t>=0.12&&l<150&&dist<9.5)) //����С��20�״����ٶȾ���ʲôλ�ý�վ
					{
						
						*cmdSteer = 1;
						if( _speed >20 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
						else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
						flag = 3;		
					}
			else{                 //���ٻ���
				*cmdSteer = (_yaw -atan2( _midline[10][0]+4,_midline[10][1]))/3.14;
				if( _speed > 30 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
				else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			}

				}
		
			
		 else if (l < 8000) { //��һ����Χʱ�������ӵ�������·�Ҳ࣬����ת��뾶
			*cmdSteer = (_yaw -atan2( _midline[10][0]+4,_midline[10][1]))/3.14;
			if( _speed > 50 ){*cmdBrake = 0.2;*cmdGear = 1;*cmdAcc = 0;}
			else{*cmdBrake = 0;*cmdGear = 1;*cmdAcc = 0.1;}
			flag = 4;
			t=abs(_yawrate);//��¼С����Ԥ���20�״�ʱС���Ľ��ٶ�
		}
		else {			                                                                         //����·�ΰ�Ѳ�߷�ʽ��ʻ
			*cmdAcc = 1;
		    *cmdBrake = 0;
		    *cmdSteer = (_yaw -8*atan2( _midline[30][0],_midline[30][1]))/3.14 ;
		    *cmdGear = 1;
			flag = 5;
			
	    }

	}
		else                   //��ʼ����
		{
			if (d < 1.5 ) {    //��ת�򵹳�
			*cmdGear = -1;
			*cmdSteer = 0 ;
			*cmdBrake = 0;
			*cmdAcc = 0.8;
			out=1;
		}else if (d < 10&&out <= 2) { //��ת�򵹳����ʵ�����
			*cmdGear = -1;
			*cmdSteer =0 ;
			*cmdBrake = 0;
			*cmdAcc = 1;
			out = 2;
		} else if (d < 90&out <= 3 ){//Զ��ͣ��λʱ����һ�����ת��			
		   *cmdGear = -1;
			*cmdSteer = 1;
			*cmdBrake = 0;
			*cmdAcc = 0.1;
			out = 3;
		} else if (d < 5000&&out <= 4) { //׼����վ
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
