

#include <math.h>
#include <string.h>
#include <unistd.h>
#include <stdbool.h>

typedef unsigned char   BYTE;

#include <SLABCP2110.h>
#include <SLABHIDtoUART.h>
#include <CP2114_Common.h>

/*--------------		マクロ		----------------*/
/*----	通信に関する設定	----*/
//ロボットのVender ID
#define	VID	(0x10C4)

//ロボットのProduct ID
#define	PID	(0xEA80)

/*----	本体の寸法に関する設定	----*/
//第一関節の軸間距離（mm）
#define	AXISLEN_A	(80.0)
//第二関節の軸間距離（mm）
#define	AXISLEN_B	(80.0)

//ステージの縦幅(mm)
#define	FIELD_H	(210.0)
//ステージの横幅(mm)
#define	FIELD_W	(230.0)
//ステージ中央を原点(0,0)とした、アームの根元（ID1モータの出力軸）のX座標(mm)
#define	X_OFS	(FIELD_W/2 - 53.0)

//上下軸の距離換算係数
#define	HEIGHT_RATE	(148.54)
//上下軸の可動範囲(mm)
#define	HEIGHT_RANGE	(35.0)
//モータ角度から距離に変換するマクロ関数。r=モータ角度値(0.1度単位)
#define	RAD_TO_HEIGHT(r)	( ((double)r/(HEIGHT_RATE*10.0))*HEIGHT_RANGE)
//距離からモータ角度に変換するマクロ関数。h=距離(mm)
#define	HEIGHT_TO_RAD(h)	(short) (h/HEIGHT_RANGE*HEIGHT_RATE*10.0)

//ハンド軸の距離換算係数
#define	WIDTH_RATE	(31.83)
//ハンド軸の可動範囲(mm)
#define	WIDTH_RANGE	(5.0*2.0)
//爪の穴位置の幅(mm)
#define	GROW_W	(5.0*2.0)
//モータ角度から幅に変換するマクロ関数。r=モータ角度値(0.1度単位)、p=爪の取り付け位置（0～3）
#define	RAD_TO_WIDTH(r,p)	( ((double)-r/(WIDTH_RATE*10.0))*WIDTH_RANGE + GROW_W*(p+1))
//幅からサーボ角度に変換するマクロ関数。w=爪の幅(mm)、p=爪の取り付け位置（0～3）
#define	WIDTH_TO_RAD(w,p)	(short) (-(w-GROW_W*(p+1))/WIDTH_RANGE*WIDTH_RATE*10.0)

//アームのモータの可動範囲（0.1度単位）
#define	ARM_RAD_RANGE	(1350)
//ハンド開閉軸のモータの可動範囲（0.1度単位）
#define	HAND_WIDTH_RANGE	(350)

/*----	座標系に関する設定	----*/
//本体の角度
#define	BASE_ANGLE	(180.0)
//本体の位置(mm)
#define	BASE_OFFSET_X	(+0.0)
#define	BASE_OFFSET_Y	(+0.0)

//現在のハンド軸の爪のねじ穴番号（0～3 = ①～④）
#define	CROW_POS	(1)

/*----	その他必要な設定	----*/
//モータの目標位置への移動に対する遷移時間(msec)
#define	MOVE_TIME	(1000)

//モータの現在位置をラジアン角に変換（※モータの角度は0.1度）
#define	SVPOS_TO_RAD(p)	(((double)(p)/1800.0)*M_PI)

void pos_to_rad(double x, double y, double z,double yaw, double w,short *sPos,int sign,int num);
void rad_to_pos(double *x,double *y, double *z ,double *yaw, double *w ,short *sPos,int num);
int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num);
int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam);
int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num);
int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,unsigned int data_len);
int RSWriteMem( HID_UART_DEVICE dev , BYTE address , BYTE size , BYTE id , BYTE *data , int num);
int SetTXOpenDrain(HID_UART_DEVICE dev );
