#include "GPS_Structs.h"
using namespace std;
//extern ROVERCFGINFO ANinfo;

/*-----------------------------------------------
	数据时间同步函数（文件）
------------------------------------------------*/
int SynRTKObsPP(FILE* FBase, FILE* FRover, RAWDAT* Raw)
{
	/* 1. 获取流动站的观测数据 */
	// 保存前一个历元的数据
	//memcpy(&Raw->BasEpk, &Raw->BasEpk_cur, sizeof(Raw->BasEpk_0));
	//memcpy(&Raw->RovEpk, &Raw->RovEpk_cur, sizeof(Raw->RovEpk_0));

	// 解码流动站的数据
	static unsigned char buff_R[MAXRAWLEN];
	static unsigned char buff_D[MAXRAWLEN];
	static int lenD_R = 0;	static int lenD_B = 0;	// 有效数据长度
	int lenR_R = 0; int lenR_B = 0;					// 剩余数据长度
	double dt;				// 时间差

	while (!feof(FRover))
	{
		// 读取流动站数据
		lenR_R = fread(buff_R + lenD_R, sizeof(unsigned char), MAXRAWLEN - lenD_R, FRover);
		if (lenR_R < MAXRAWLEN - lenD_R)
			return -1;	// 文件读取到尾

		lenD_R += lenR_R;

		// 解码流动站数据
		if (DecodeNovOem7Dat(buff_R, lenD_R, &Raw->RovEpk, Raw->GpsEph, Raw->BdsEph) == 0)
			break;	// 解码成功
	}
	if (feof(FRover))
	{
		cout << "The file of Rover comes to end!\n";
		return -1;
	}

	/* 2. 比较观测时刻的差值 */
	// 计算流动站和基站的时间差
	dt = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800.0
		+ Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek;

	// ① 若时间差在限差内，则同步成功
	if (fabs(dt) < 0.01) return 1;

	// ② 流动站数据滞后，无法解算
	else if (dt < 0) return 0;

	// ③ 基站数据滞后，循环读取基站数据
	while (!feof(FBase))
	{
		// 读取基站数据
		lenR_B = fread(buff_D + lenD_B, sizeof(unsigned char), MAXRAWLEN - lenD_B, FBase);
		if (lenR_B < MAXRAWLEN - lenD_B)
			return -1;	// 解码失败

		lenD_B += lenR_B;

		// 解码基站数据
		if (DecodeNovOem7Dat(buff_D, lenD_B, &Raw->BasEpk, Raw->GpsEph, Raw->BdsEph) == 0)
		{
			// 计算流动站和基站的时间差
			dt = (Raw->RovEpk.Time.Week - Raw->BasEpk.Time.Week) * 604800.0
				+ Raw->RovEpk.Time.SecOfWeek - Raw->BasEpk.Time.SecOfWeek;

			// 若时间差在限差内，则同步成功
			if (fabs(dt) < 0.01) return 1;

			// 流动站数据滞后，无法解算
			else if (dt < 0) return 0;

			// 否则继续解算基站数据
		}
	}
	if (feof(FBase))
	{
		cout << "The file of Base comes to end!\n";
		return -1;
	}

	return 0;	// 数据同步失败
}


void GetSynGNSSObs() {

}
