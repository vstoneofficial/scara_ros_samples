#include <stdio.h>
#include <stdlib.h>
#include "scara_lib.h"

/************************************************

	void pos_to_rad(double x, double y, double z,double yaw, double w,short *sPos,int sign,int num)
	
	概要：与えられた直交座標（X/Y/Z）とハンド軸の角度・開閉幅を、モータの目標位置に変換する関数。

	引数：
		double x,y,z	… 変換元のX/Y/Z座標
		double yaw		… 変換元のハンド回転軸の角度
		double w		… 変換元のハンド開閉軸の幅
		short sPos		… 計算後のモータ角度を代入する配列変数へのポインタ
		int sign		… アームの折れ曲がる向き。正の値だと時計回り、負の値だと反時計回りに折れ曲がる
		int num			… 使用するモータ数

	戻り値：
		無し

*************************************************/
void pos_to_rad(double x, double y, double z,double yaw, double w,short *sPos,int sign,int num)
{

	double a,b,tx,ty,lx,ly;		//余弦定理を求める変数
	double thete1,thete2;		//ID1、ID2の角度を求める変数
	double s;					//アームの折れ曲がる向き（符号）に使用する変数。int signの符号に応じて+1.0か-1.0を代入する

	//引数の符号からアームの折れ曲がる向きを決定
	if(sign<0) s=-1.0;
	else s=1.0;

	//本体の位置と角度の設定を反映させる（変換式を逆転させて、向き・位置共に0の位置を基準にした値に変換）
	lx= x-BASE_OFFSET_X;
	ly= y-BASE_OFFSET_Y;
	x = (lx*cos(-BASE_ANGLE/180.0*M_PI)-ly*sin(-BASE_ANGLE/180.0*M_PI));
	y = (lx*sin(-BASE_ANGLE/180.0*M_PI)+ly*cos(-BASE_ANGLE/180.0*M_PI));

	ty = y;
	tx = x+X_OFS;


	//もしリンク長より目標値が遠ければ、根元の軸を目標値までの角度に合わせて、腕を伸ばす
	if(hypot(tx,ty)>=AXISLEN_A+AXISLEN_B){

		sPos[0] = (short) (atan2(ty,tx)/M_PI*1800.f);
		sPos[1] = 0;

	}
	else{

		//余弦定理を演算
		a = acos( (-(tx*tx+ty*ty) + AXISLEN_A*AXISLEN_A + AXISLEN_B*AXISLEN_B) / (2 * AXISLEN_A * AXISLEN_B));
		b = s*acos( (- AXISLEN_A*AXISLEN_A + AXISLEN_B*AXISLEN_B + (tx*tx+ty*ty)) / (2 * AXISLEN_A * sqrt(tx*tx+ty*ty)));

		//指定のX/Y座標の符号によって、角度の計算方法を選択
		if(tx<0){
			if(ty>=0) thete1 = M_PI + atan(ty/tx) +b;
			else thete1 = atan(ty/tx) + b - M_PI;
		}
		else thete1 = atan(ty/tx)+b;
		thete2 = -(M_PI-a)*s;


		//求まった角度を360度法に変換
		if(thete1!=0.0) thete1 = thete1 / M_PI * 180.0;
		if(thete2!=0.0) thete2 = thete2 / M_PI * 180.0;

		//変換した角度を0.1度単位に変換し、モータの出力方向に合わせて符号を変更（ID1）
		sPos[0] = (short) (thete1*10.0);
		sPos[1] = (short) (thete2*10.0);


	}
	//アームの2軸の可動範囲制限
	if(sPos[0]<-ARM_RAD_RANGE) sPos[0]=-ARM_RAD_RANGE;
	else if(sPos[0]>ARM_RAD_RANGE) sPos[0]=ARM_RAD_RANGE;

	if(sPos[1]<-ARM_RAD_RANGE) sPos[1]=-ARM_RAD_RANGE;
	else if(sPos[1]>ARM_RAD_RANGE) sPos[1]=ARM_RAD_RANGE;

	//Z座標の変換
	sPos[2] = HEIGHT_TO_RAD(z);

	//ハンドの2軸の変換
	if(num>3){
		//グリップ幅
		sPos[4] = WIDTH_TO_RAD(w,CROW_POS);

		if(sPos[4]<-HAND_WIDTH_RANGE) sPos[4]=-HAND_WIDTH_RANGE;
		else if(sPos[4]>HAND_WIDTH_RANGE) sPos[4]=HAND_WIDTH_RANGE;

		sPos[3] = -sPos[0]-sPos[1];
		sPos[3] += (short) (yaw*10.0);
	}

}


/************************************************

	void rad_to_pos(double *x,double *y, double *z ,double *yaw, double *w ,short *sPos,int num)
	
	概要：与えられたモータ角度をX/Y座標に変換する関数。

	引数：
		double *x,*y,*z	… 変換後のX/Y/Z座標を代入する変数へのポインタ
		double *yaw		… 変換後の回転軸の角度を代入する変数へのポインタ
		double *w		… 変換後のハンド軸の幅を代入する変数へのポインタ
		short *sPos		… 座標計算の元となるモータ角度を記録した配列変数へのポインタ
		int num			… モータの個数を代入。4以上の場合、ハンド回転軸とハンド軸幅の計算を行う

	戻り値：
		無し

*************************************************/
void rad_to_pos(double *x,double *y, double *z ,double *yaw, double *w ,short *sPos,int num)
{
	double tx,ty;
	double lx,ly;

	//ID1,ID2のモータの現在位置より、順運動学でアームの座標を求める
	ty = sin( SVPOS_TO_RAD(sPos[0]) )*AXISLEN_A + sin( SVPOS_TO_RAD(sPos[0]+sPos[1]) )*AXISLEN_B;
	tx = cos( SVPOS_TO_RAD(sPos[0]) )*AXISLEN_A + cos( SVPOS_TO_RAD(sPos[0]+sPos[1]) )*AXISLEN_B;

	//X座標を原点に合わせてオフセットする
	tx-=X_OFS;

	//本体の位置と角度の設定を反映させる
	lx=tx;
	ly=ty;

	tx =  (lx*cos(BASE_ANGLE/180.0*M_PI)-ly*sin(BASE_ANGLE/180.0*M_PI)) + BASE_OFFSET_X;
	ty =  (lx*sin(BASE_ANGLE/180.0*M_PI)+ly*cos(BASE_ANGLE/180.0*M_PI)) + BASE_OFFSET_Y;

	//求まった座標をポインタ変数へ代入
	*x=tx;
	*y=ty;
	*z= RAD_TO_HEIGHT(sPos[2]);
	if(num>3){
		*yaw = (double) (sPos[3] +sPos[0]+sPos[1])/10.0;
		*w = RAD_TO_WIDTH(sPos[4],CROW_POS);
	}


}

/************************************************

	int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num)
	
	概要：モータのトルクを切り替える関数。IDが連続した複数のモータを同時に設定可能。

	引数：
		HID_UART_DEVICE dev	… 通信ハンドル
		short sMode			… ゲインのON/OFFを指定。0=off、1=ON
		BYTE id				… 切り替えを開始するモータのID（複数のモータを切り替える場合、先頭のモータのID）
		int num				… 切り替えを行うモータ数

	戻り値：
		正常に処理を実行できたらTRUE、そうでなければFALSE

*************************************************/
int RSTorqueOnOff( HID_UART_DEVICE dev, short sMode ,BYTE id,int num)
{
	unsigned char	sendbuf[256],*bufp;				//送信バッファ関係
	unsigned char	sum;							//チェックサム計算用
	int				ret;							//戻り値記録用
	unsigned int			data_len=0,len=0;				//パケット長と書き込みサイズ取得用
	unsigned char	i;


	//送信バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	//パケット作成
	//1．ヘッダ・共通部分の作成
	sendbuf[0]  = (unsigned char)0xFA;				// ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;				// ヘッダー2
	sendbuf[2]  = (unsigned char)0x00;				// サーボID(常に0)
	sendbuf[3]  = (unsigned char)0x00;				// フラグ(常に0)
	sendbuf[4]  = (unsigned char)0x24;				// アドレス(トルクON/OFF 0x24=36)
	sendbuf[5]  = (unsigned char)0x01+1;			// 長さ(1byte)
	sendbuf[6]  = (unsigned char)num;				// モータの個数

	//共通部分のパケット長を記録
	data_len = 7;			

	//2．サーボ個別部分のパケット作成
	bufp = &sendbuf[7];								// 送信バッファの個別メッセージ部分の開始アドレスを代入

	//書き換えるモータの個数分だけ、個別のパケットを追加
	for(i=0;i<num;i++){
		*bufp++ = id+i;								//モータのID
		data_len++;									//パケット長を1byte加算

		*bufp++ = (unsigned char)(sMode&0x00FF);	//ON・OFFフラグ
		data_len++;									//パケット長を1byte加算
	}

	//3．チェックサムの計算
	//チェックサムは、送信バッファの3byte目(サーボID)～終端を1byteずつXORした値です。
	sum = sendbuf[2];
	for( i = 3; i < data_len; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[data_len] = sum;						// 求まったチェックサムを送信バッファの最後に代入
	data_len++;										//パケット長を1byte加算

	//4．メッセージの送信
	ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );

	if(ret!=HID_UART_SUCCESS) return FALSE;

	//5．ローカルエコーの読み取り
	return ReadLocalEcho(dev,sendbuf,data_len);
}



/************************************************

	int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam)
	
	概要：モータの現在位置を取得する。現在位置の取得は同時に1個のモータしか対応していない

	引数：
		HID_UART_DEVICE dev	… 通信ハンドル
		BYTE id				… 現在位置を取得するモータのID
		short *getParam		… 取得した現在位置を格納する変数へのポインタ

	戻り値：
		正常に処理を実行できたらTRUE、そうでなければFALSE

*************************************************/
int RSGetAngle( HID_UART_DEVICE dev ,BYTE id,short *getParam)
{
	unsigned char	sendbuf[32];
	unsigned char	readbuf[128];
	unsigned char	sum;
	unsigned int			i;
	int				ret;
	unsigned int	len, readlen;
	short			angle;

	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// パケット作成
	sendbuf[0]  = (unsigned char)0xFA;				// ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;				// ヘッダー2
	sendbuf[2]  = (unsigned char)id;				// サーボID
	sendbuf[3]  = (unsigned char)0x0f;				// フラグ(0x0f=指定アドレスから指定バイト取得)
	sendbuf[4]  = (unsigned char)0x2a;				// 取得すつデータのアドレス(現在位置=0x2a)
	sendbuf[5]  = (unsigned char)0x02;				// 取得するデータの長さ(現在位置=2byte)
	sendbuf[6]  = (unsigned char)0x00;				// 個数(0)

	
	// チェックサムの計算
	sum = sendbuf[2];
	for( i = 3; i < 7; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[7] = sum;								// チェックサム

	// パケットを送信
	ret = HidUart_Write(dev,sendbuf, 8, &len);

	//ローカルエコーを読み取り
	if(!ReadLocalEcho(dev,sendbuf,len)) return FALSE;

	// もし送信できたパケット長がデータサイズよりも小さい場合、エラー
	if( len < 8 ) return FALSE;


	// 受信バッファの読み込み
	memset( readbuf, 0x00, sizeof( readbuf ));

	//受信バッファのパケット長の計算
	//	Header(2byte) + ID(1byte) + Flags(1byte) + Address(1byte) + Length(1byte) + Count(1byte) + Dada(2byte) + Sum(1byte)
	readlen = (2) + (1) + (1) + (1) + (1) + (1) + (2) + (1);
	len = 0;

	//バッファの受信
	HidUart_Read(dev,readbuf, readlen, &len);

	//受信バッファのパケット長が、計算で求めた長さ(readlen)と異なる場合、エラー
	if( len < readlen)  return FALSE;

	// 受信データのチェックサム確認
	sum = readbuf[2];
	for( i = 3; i < readlen; i++ ){
		sum = sum ^ readbuf[i];
	}

	//チェックサムが異なる場合、エラー
	if( sum ) return FALSE;

	//受信バッファから、読み取った現在位置のデータを取り出す
	angle = ((readbuf[8] << 8) & 0x0000FF00) | (readbuf[7] & 0x000000FF);
	if(getParam) *getParam = angle;

	return TRUE;
}


/************************************************

	int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num)
	
	概要：モータを指定の角度に動かす。IDが連続した複数のモータを一度に動かすことが可能。

	引数：
		HID_UART_DEVICE dev	… 通信ハンドル
		short *sPoss		… 目標位置を記録した配列変数へのポインタ。開始IDの目標値は、バッファの先頭から順に使用される
		unsigned short sTime … 目標位置までの遷移時間(10ミリ秒単位)
		BYTE id				… 動かすモータのID（複数のモータを動かす場合、先頭のモータのID）
		int num				… 動かすモータ数

	戻り値：
		正常に処理を実行できたらTRUE、そうでなければFALSE

*************************************************/
int RSMove( HID_UART_DEVICE dev , short *sPoss, unsigned short sTime ,BYTE id,int num)
{
	unsigned char	sendbuf[256],*bufp;
	unsigned char	sum;
	unsigned char	i;
	int				ret;
	unsigned int	len,data_len;


	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// パケット作成
	sendbuf[0]  = (unsigned char)0xFA;				    // ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;				    // ヘッダー2
	sendbuf[2]  = (unsigned char)0;						// ID(0)
	sendbuf[3]  = (unsigned char)0x00;				    // フラグ(0x00)
	sendbuf[4]  = (unsigned char)0x1E;				    // アドレス(0x1E=30)
	sendbuf[5]  = (unsigned char)0x04+1;			    // 長さ(4byte)
	sendbuf[6]  = (unsigned char)num;				    // モータの個数

	//共通部分のパケット長を記録
	data_len = 7;

	//個別のデータ作成
	bufp = &sendbuf[7];
	for(i=0;i<num;i++){
		*bufp++ = id+i;								//モータID
		data_len++;									//パケット長を1byte加算

		//目標位置をバッファに書き込み(2byte)
		*bufp++ = (unsigned char)(sPoss[i]&0x00FF);
		*bufp++ = (unsigned char)((sPoss[i]&0xFF00)>>8);
		data_len+=2;								//パケット長を2byte加算

		//遷移時間をバッファに書き込み(2byte)
		*bufp++ = (unsigned char)(sTime&0x00FF);
		*bufp++ = (unsigned char)((sTime&0xFF00)>>8);
		data_len+=2;								//パケット長を2byte加算
	}


	// チェックサムの計算
	sum = sendbuf[2];
	for( i = 3; i < data_len; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[data_len] = sum;						// 送信バッファにチェックサムを追加
	data_len++;										//パケット長を1byte加算

	// パケットを送信
	ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );
	if(ret!=HID_UART_SUCCESS) return FALSE;

	//ローカルエコーの読み取り
	return ReadLocalEcho(dev,sendbuf,data_len);

}


/************************************************

	int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,unsigned int data_len)
	
	概要：
		送信メッセージのローカルエコーを読み出す関数。
		引数 data_len の長さだけメッセージを受信し、 引数 *sendbuf の内容と同一か比較を行う

	引数：
		HID_UART_DEVICE dev		… 通信ハンドル
		unsigned char *sendbuf	… 送信メッセージ内容へのポインタ
		unsigned int data_len			… 送信メッセージのサイズ

	戻り値：
		送信メッセージと同一の内容を受信できたらTRUE、そうでなければFALSE

*************************************************/
int ReadLocalEcho(HID_UART_DEVICE dev ,unsigned char *sendbuf,unsigned int data_len)
{

	unsigned char readbuf[1024];
	unsigned int len=0;
	memset(readbuf,0,sizeof(readbuf));

	//data_len のサイズだけメッセージを受信
	HidUart_Read( dev, (BYTE*) readbuf, data_len, &len );

	//受信メッセージのサイズが異なる場合、エラー
	if(data_len!=len) return FALSE;

	//受信メッセージと送信メッセージを比較
	for(unsigned int i=0;i<len;i++){

		//受信メッセージと送信メッセージが異なる場合、エラー
		if(readbuf[i]!=sendbuf[i]) return FALSE;
	}
	return TRUE;
}




/************************************************

	int int RSWriteMem( HID_UART_DEVICE dev , BYTE address , BYTE size , BYTE id , BYTE *data , int num)
	
	概要：
		モータのパラメータを書き換える。連続した複数のモータ・複数のアドレスに書き込みが可能
		アドレス・サイズ・モータ数・実データを指定する

	引数：
		HID_UART_DEVICE dev	… 通信ハンドル
		BYTE address		… データを書き込むアドレス（アドレスの詳細はRS304MDの資料を参照）
		BYTE size			… モータ1個当たりに書き込むデータのサイズ(byte単位)
		BYTE id				… 動かすモータのID（複数のモータを動かす場合、先頭のモータのID）
		BYTE *data			… 書き込みデータ内容の配列変数へのポインタ
		int num				… 動かすモータ数

	戻り値：
		正常に処理を実行できたらTRUE、そうでなければFALSE

*************************************************/
int RSWriteMem( HID_UART_DEVICE dev , BYTE address , BYTE size , BYTE id , BYTE *data , int num)
{
	unsigned char	sendbuf[256],*bufp;
	unsigned char	sum;
	unsigned char	i,j;
	int				ret;
	unsigned int	len,data_len;


	// バッファクリア
	memset( sendbuf, 0x00, sizeof( sendbuf ));

	// パケット作成
	sendbuf[0]  = (unsigned char)0xFA;				    // ヘッダー1
	sendbuf[1]  = (unsigned char)0xAF;				    // ヘッダー2
	sendbuf[2]  = (unsigned char)0;						// ID(0)
	sendbuf[3]  = (unsigned char)0x00;				    // フラグ(0x00)
	sendbuf[4]  = (unsigned char)address;				    // アドレス(0x1E=30)
	sendbuf[5]  = (unsigned char)size+1;			    // 長さ(4byte)
	sendbuf[6]  = (unsigned char)num;				    // モータの個数

	//共通部分のパケット長を記録
	data_len = 7;

	//個別のデータ作成
	bufp = &sendbuf[7];
	for(i=0;i<num;i++){
		*bufp++ = id+i;								//モータID
		data_len++;									//パケット長を1byte加算

		for(j=0;j<size;j++){
			*bufp++ = (unsigned char)data[j];		//データ部を1byteずつパケットに追加
			data_len++;								//パケット長を2byte加算
		}
	}


	// チェックサムの計算
	sum = sendbuf[2];
	for( i = 3; i < data_len; i++ ){
		sum = (unsigned char)(sum ^ sendbuf[i]);
	}
	sendbuf[data_len] = sum;						// 送信バッファにチェックサムを追加
	data_len++;										//パケット長を1byte加算

	// パケットを送信
	ret = HidUart_Write( dev, (BYTE*) sendbuf, data_len, &len );
	if(ret!=HID_UART_SUCCESS) return FALSE;

	//ローカルエコーの読み取り
	return ReadLocalEcho(dev,sendbuf,data_len);

}

/************************************************

	int SetTXOpenDrain(HID_UART_DEVICE dev )
	
	概要：
		USB-to-UART（CP2110）のポート設定について、TXをOpen-Drainに設定変更する。
		ロボット本体と通信する際には、必ずこの設定が必要（設定を変更するとロボット本体に記録される）。

	引数：
		HID_UART_DEVICE dev	… 通信ハンドル

	戻り値：
		HID_UART_STATUS にて定義されている各数値。HID_UART_SUCCESSであれば成功、それ以外は失敗

*************************************************/
int SetTXOpenDrain(HID_UART_DEVICE dev )
{
	//GPIOの設定で、TX を Open-Drain に設定する必要がある
	BYTE pinConfig[13];
	BOOL useSuspendValues;
	WORD suspendValue;
	WORD suspendMode;
	BYTE rs485Level;
	BYTE clkDiv;

	//現在の Pin Config を取得
	HidUart_GetPinConfig(dev,  pinConfig,  &useSuspendValues, &suspendValue, &suspendMode, &rs485Level, &clkDiv);

	//TX を TX-Open-Drain(0x01) に変更
	pinConfig[10] = 0x01;

	//Pin Config を更新
	return HidUart_SetPinConfig(dev,  pinConfig, useSuspendValues, suspendValue, suspendMode,rs485Level, clkDiv);

}
