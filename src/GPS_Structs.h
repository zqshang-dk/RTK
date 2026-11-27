#ifndef _GNSS_RTK_H_
#define _GNSS_RTK_H_

#include <stdio.h>
#include <Windows.h>
#include <cmath>
#include <iostream>
#include <iomanip>
#include <Eigen/Dense>

using namespace Eigen;

#define POSITION_MODE_SPP 1
#define POSITION_MODE_RTK 2
#define INPUT_MODE_NETWORK 0
#define INPUT_MODE_FILE 1

#define PI                3.1415926535898
#define CLIGHT            299792458.0
#define SQR(x)            ((x)*(x))
#define MAXIteration      15
#define Rad2Deg           (180.0/PI)
#define Deg2Rad           (PI/180.0)

#define R_WGS84           6378137.0          /* Radius Earth [m]; WGS-84  */
#define F_WGS84           1.0/298.257223563  /* Flattening; WGS-84   */
#define Omega_WGS         7.2921151467e-5   /*[rad/s], the earth rotation rate */
//#define GM_Earth         398600.5e+9      /* [m^3/s^2]; WGS-84 */
#define R_CGS2K           6378137.0          /* Radius Earth [m]; CGCS2000  */
#define F_CGS2K           1.0/298.257222101  /* Flattening; CGCS2000   */
#define Omega_BDS         7.2921150e-5      /*[rad/s], the earth rotation rate */
//#define GM_BDS           398600.4418e+9     /* [m^3/s^2]; CGCS2000  */
#define mu_GPS            3.986005e14
#define OMEGA_e_dot_GPS   7.2921151467e-5
#define mu_BDS            3.986004418e14
#define OMEGA_e_dot_BDS   7.2921150e-5

#define POLYCRC32         0xEDB88320u /* CRC32 polynomial */
#define MAXCHANNUM        148
#define MAXGPSNUM         32
#define MAXBDSNUM         63
#define MAXRAWLEN         40000               // 原始数据缓冲区最大长度

#define  FG1_GPS          1575.42E6                /* L1信号频率 */
#define  FG2_GPS          1227.60E6                /* L2信号频率*/
#define  WL1_GPS          (CLIGHT/FG1_GPS)
#define  WL2_GPS          (CLIGHT/FG2_GPS)

#define  FG1_BDS          1561.098E6               /* B1信号频率 */
#define  FG2_BDS          1207.140E6               /* B2信号频率 */
#define  FG3_BDS          1268.520E6               /* B3信号频率 */
#define  WL1_BDS          (CLIGHT/FG1_BDS)
#define  WL2_BDS          (CLIGHT/FG2_BDS)
#define  WL3_BDS          (CLIGHT/FG3_BDS)

/************************粗差探测MWGF阈值设置********************/
#define  GF_Threshold     0.05                  //GF阈值
#define  MW_Threshold     3.00                  //MW阈值

/************************对流层改正所需参数********************/
#define  H0               0                     //海平面高度
#define  T0               15 + 273.16           //标准温度
#define  P0               1013.25               //气压（单位mbar）
#define  RH0              0.50                  //相对湿度

//GNSS Systems
enum GNSSSYS { GPS = 0, BDS = 1, GLONASS = 2, GALILEO = 3, QZSS = 4, UNKNOWN = 5 };

struct COMMONTIME {
	short Year;
	unsigned short Month;
	unsigned short Day;
	unsigned short Hour;
	unsigned short Minute;
	double Second;

	COMMONTIME()
	{
		Year = 0;
		Month = 0;
		Day = 0;
		Hour = 0;
		Minute = 0;
		Second = 0.0;
	}
};

struct MJDTIME {
	int Days;
	double FracDay;

	MJDTIME() {
		Days = 0;
		FracDay = 0.0;
	}
};

struct GPSTIME {
	short Week;
	double SecOfWeek;

	GPSTIME() {
		Week = 0;
		SecOfWeek = 0.0;
	}

	GPSTIME(short w,double sow):Week(w),SecOfWeek(sow){}
};

union XYZ {
	double xyz[3];
	struct {
		double x;
		double y;
		double z;
	};
};

union BLH {
	double blh[3];
	struct {
		double b;
		double l;
		double h;
	};
};

union ENU {
	double enu[3];
	struct {
		double e;
		double n;
		double u;
	};
};

//GPS广播星历结构体
struct GPSEPH {
	short PRN;
	GNSSSYS System;
	GPSTIME TOC, TOE;    //钟差和星历拟合的参考时间
	double a0, a1, a2;   //钟差，钟漂，频漂
	double IODE, IODC;
	double sqrt_A, M0, e, OMEGA, i0, omega;
	double Crc, Crs, Cuc, Cus, Cic, Cis;
	double deltaN, OMEGA_dot, IDOT;
	int SVhealth;
	double tgd1, tgd2;
};

// 每颗卫星的观测值
struct SATOBSDATA {
	short Prn;
	GNSSSYS System;
	double P[2], L[2], D[2];   //双频
	double   cn0[2], LockTime[2];  //判断历元间相位数据的连续性，没有发生中断时，locktime(k+1)>=locktime(k)，数据正常，反之有周跳
	unsigned char half[2];
	bool valid;

	SATOBSDATA()
	{
		Prn = 0;
		System = UNKNOWN;
		for (int i = 0; i < 2; i++){
			P[i] = L[i] = D[i] = 0.0;
			cn0[i] = LockTime[i] = 0.0;
        	half[i] = 0;
		}
		valid = false;
	}
};

/* 每颗卫星位置、速度和钟差等的中间计算结果 */
struct SATMIDRES
{
	double SatPos[3], SatVel[3];
	double SatClkOft, SatClkSft;
	double Elevation, Azimuth;
	double TropCorr;
	double Tgd1, Tgd2;
	bool Valid;  //false=没有星历或星历过期,true-计算成功

	SATMIDRES()
	{
		SatPos[0] = SatPos[1] = SatPos[2] = 0.0;
		SatVel[0] = SatVel[1] = SatVel[2] = 0.0;
		Elevation = PI / 2.0;
		SatClkOft = SatClkSft = 0.0;
		Tgd1 = Tgd2 = TropCorr = 0.0;
		Valid = false;
	}
};

struct MWGF {
	short Prn;
	GNSSSYS System;
	double MW;
	double GF;
	double PIF;
	int n;    //平滑计数

	double BasLockTime[2]; //保存上一历元基站和流动站的锁定时间
    double RovLockTime[2]; 

    unsigned char BasParity[2]; // 保存上一历元的奇偶状态
    unsigned char RovParity[2];

	MWGF() {
		Prn = n = 0;
		System = UNKNOWN;
		MW = GF = PIF = 0.0;

		for (int i = 0; i < 2; i++)
        {
            BasLockTime[i] = 0.0;
            RovLockTime[i] = 0.0;
            BasParity[i] = 0;
            RovParity[i] = 0;
        }
	}
};

//每个历元的观测值
struct EPOCHOBSDATA
{
	GPSTIME      Time;     // 没有经过钟差改正的时间（接收机钟的表面时）
	short        SatNum;
	SATOBSDATA   SatObs[MAXCHANNUM];     //原始观测
	SATMIDRES    SatPVT[MAXCHANNUM];     //卫星状态
	MWGF         CombObs[MAXCHANNUM];    //组合观测

	EPOCHOBSDATA()
	{
		SatNum = 0;
	}
};

// 用户位置速度信息
struct SPPPOS {
	GPSTIME Time;
	double Pos[3], Vel[3];
	double RcvClk[2];
	double PDOP, SigmaPos, SigmaVel;
	MatrixXd Dxx;
	int SatNum;
};

struct ROVERCFGINFO   // 配置信息
{
	short  PositionMode;                 // 1=SPP, 2=RTK
	short  IsFileData;                   // 1=文件, 0=网络
	short  RTKProcMode;                  // 1=EKF, 2=LSQ，仅RTK模式使用
	int    RovPort, RovBaud;             // COM端口设置
	char   BasNetIP[20], RovNetIP[20];   // ip address
	unsigned short BasNetPort, RovNetPort; // port
	double CodeNoise, CarrierNoise;           // 伪距噪声
	double ElevThreshold;                // 高度角阈值
	double RatioThres;                   // Ratio检验阈值

	char  SPPObsDatFile[256];                         // 单点定位观测文件
	char  BasObsDatFile[256], RovObsDatFile[256];    // RTK基站和流动站观测文件
	char  ResFile[256];                              // 结果文件

	ROVERCFGINFO()
	{
		PositionMode = POSITION_MODE_SPP;
		IsFileData = INPUT_MODE_FILE;
		RTKProcMode = 2;
		RovPort = RovBaud = BasNetPort = RovNetPort = 0;
		CodeNoise = CarrierNoise = ElevThreshold = 0.0;
		RatioThres = 3.0;
		BasNetIP[0] = RovNetIP[0] = '\0';
		SPPObsDatFile[0] = BasObsDatFile[0] = RovObsDatFile[0] = '\0';
		ResFile[0] = '\0';
	}
};



/*  每颗卫星的单差观测数据定义  */
struct SDSATOBS
{
	short    Prn;
	GNSSSYS  System;
	short    Valid;
	double   dP[2], dL[2];   // m
	short    nBas, nRov;   // 存储单差观测值对应的基准和流动站的数值索引号

	SDSATOBS()
	{
		Prn = nBas = nRov = 0;
		System = UNKNOWN;
		dP[0] = dL[0] = dP[1] = dL[1] = 0.0;
		Valid = -1;
	}
};

/*  每个历元的单差观测数据定义  */
struct SDEPOCHOBS
{
	GPSTIME    Time;
	short      SatNum;
	SDSATOBS   SdSatObs[MAXCHANNUM];
	MWGF       SdCObs[MAXCHANNUM];

	SDEPOCHOBS()
	{
		SatNum = 0;
	}
};

/*  双差相关的数据定义  */
struct DDCOBS
{
	int RefPrn[2], RefPos[2];         // 参考星卫星号与存储位置，0=GPS; 1=BDS
	int Sats, DDSatNum[2];            // 待估的双差模糊度数量，0=GPS; 1=BDS
	double FixedAmb[MAXCHANNUM * 4];  // 包括双频最优解[0,AmbNum]和次优解[AmbNum,2*AmbNum]
	double ResAmb[2], Ratio;          // LAMBDA浮点解中的模糊度残差
	float  FixRMS[2];                 // 固定解定位中rms误差
	double dPos[3];                   // 基线向量
	bool bFixed;                      // true为固定，false为未固定
	int SdIndex[MAXCHANNUM];          // 记录双差中的这颗卫星的索引
	double ddP[MAXCHANNUM][2], ddL[MAXCHANNUM][2];     // 保存双频伪距和载波相位双差
	double FloatAmb[MAXCHANNUM * 4];  // 保存最小二乘求出的浮点模糊度
	MatrixXd QFloat;                  // 保存浮点参数的不确定程度及相互关系
    double Sigma0;                    // 判断残差与设定噪声是否相符
	double FloatRMS[3];               // 保存三个坐标分量的精度估计

	DDCOBS()
	{
		int i;
		for (i = 0; i < 2; i++) {
			DDSatNum[i] = 0;    // 各卫星系统的双差数量
			RefPos[i] = RefPrn[i] = -1;
		}
		Sats = 0;              // 双差卫星总数
		dPos[0] = dPos[1] = dPos[2] = 0.0;
		ResAmb[0] = ResAmb[1] = Ratio = 0.0;
		FixRMS[0] = FixRMS[1] = 0.0;
		bFixed = false;
		for (i = 0; i < MAXCHANNUM * 2; i++)
		{
			FixedAmb[2 * i + 0] = FixedAmb[2 * i + 1] = 0.0;
		}
	}
};
/*  RTK定位的数据定义  */
struct RAWDAT {
	EPOCHOBSDATA BasEpk;   //基站当前观测历元
	EPOCHOBSDATA RovEpk;   //流动站当前观测历元
	SPPPOS BaseBest;       // 基站的参考位置SPP结果
	SPPPOS RoverBest;      // 流动站的参考位置SPP结果
	SDEPOCHOBS SdObs;      // 站间单差观测
	DDCOBS DDObs;          // 双差和固定解结果
	GPSEPH GpsEph[MAXGPSNUM], BdsEph[MAXBDSNUM];  //广播星历
};
/* AngleChange.cpp */
double Degree2Rad(double theta);
double Rad2Degree(double theta);

/* MatrixModule.cpp */
Matrix3d BLH2ENUMat(double blh[3]);
Matrix3d Rx(double theta);
Matrix3d Rz(double theta);
Matrix3d Rz_dot(double theta);

/* TimeChange.cpp */
void CommonTimeToMJDTime(const COMMONTIME* ct, MJDTIME* mjd);
void MJDTimeToCommonTime(const MJDTIME* mjd, COMMONTIME* ct);
void MJDTimeToGPSTime(const MJDTIME* mjd, GPSTIME* gt);
void GPSTimeToMJDTime(const GPSTIME* gt, MJDTIME* mjd);
void CommonTimeToGPSTime(const COMMONTIME* ct, GPSTIME* gt);
void GPSTimeToCommonTime(const GPSTIME* gt, COMMONTIME* ct);


/* CoordinateSystem.cpp */
double Distance(double X1[3], double X2[3]);
void BLHToXYZ(const double blh[3], double xyz[3], const double R, const double F);
void XYZToBLH(const double xyz[3], double blh[3], const double R, const double F);
//void BLHToENUMat(const double blh[], double Mat[]);
void CompSatElAz(const double Xr[], const double Xs[], double R, double F, double* Elev, double* Azim); void CompSatElAz(const double Xr[], const double Xs[], double* Elev, double* Azim);
void CompEnudPos(const double X0[], const double Xr[], double dENU[], const double R, const double F);


/*Decode.cpp*/
int DecodeNovOem7Dat(unsigned char Buff[], int& Len, EPOCHOBSDATA*obs, GPSEPH geph[], GPSEPH beph[]);
int decode_rangeb_oem7(unsigned char* buff, EPOCHOBSDATA* obs);
int decode_gpsephem(unsigned char* buff, GPSEPH* geph);
int decode_bdsephem(unsigned char* buff, GPSEPH* beph);
//int decode_psrpos(unsigned char* buff, POSRES* pos);

/*decode*/
double D8(unsigned char* p);
float F4(unsigned char* p);
int I4(unsigned char* p);
unsigned int UI4(unsigned char* p);
short S2(unsigned char* p);
unsigned short US2(unsigned char* p);

/* SatPVTCalculation.cpp*/
double GetDiffTime(GPSTIME *t1, GPSTIME *t2);
bool CompSatClkOff(const int Prn, const GNSSSYS Sys, const GPSTIME* t,GPSEPH* GPSEph, GPSEPH* BDSEph, SATMIDRES* Mid);
int CompGPSSatPVT(const int Prn, const GPSTIME* t, const GPSEPH* Eph, SATMIDRES* Mid);
int CompBDSSatPVT(const int Prn, const GPSTIME* t, const GPSEPH* Eph, SATMIDRES* Mid);
void EarthRotationCorrection(SATMIDRES* Mid, double UserPos[3], double Omegae);
//void SatPVTCalculation(GPSEPH* geph, GPSEPH* beph, EPOCHOBSDATA* obs, double UserPos[3]);
void SatPVTCalculation(GPSEPH* geph, GPSEPH* beph, EPOCHOBSDATA* obs, SPPPOS* UserPos/*double UserPos[3]*/);
//void TestSatPvt(EPOCHOBSDATA* obs, GPSEPH* geph, GPSEPH* beph);

/* ErrorCorrect.cpp*/
double HopefieldTropCorr(double H, double E);
void DetectOutlier(EPOCHOBSDATA* obs);


/* Socket.cpp */
void CloseSocket(SOCKET& sock);
bool OpenSocket(SOCKET& sock, const char IP[], const unsigned short Port);


/* SPP.cpp */
//void SatPVTatSignal(GPSEPH* geph, GPSEPH* beph, double UserPos[3], EPOCHOBSDATA* obs);
bool SPP(GPSEPH* geph, GPSEPH* beph, SPPPOS* UserPos,
	EPOCHOBSDATA* obs, double elevationThreshold);
void SPV(EPOCHOBSDATA* obs, SPPPOS* UserPos, double elevationThreshold);


/* ReadCtrlFile.cpp */
bool ReadRTKConfigInfo(const char configFile[], ROVERCFGINFO* config);

/* RTK.cpp */
int ReadRTKEpoch(FILE *fp, SOCKET sock, int inputMode, unsigned char buff[], int &len, EPOCHOBSDATA *Obs, RAWDAT *Raw, const char station[]);
int GetSynObs(FILE* FBas, FILE* FRov, SOCKET& BasSock, SOCKET& RovSock, RAWDAT* Raw,int inputMode);
void FormSDEpochObs(const EPOCHOBSDATA *EpkA, const EPOCHOBSDATA *EpkB, SDEPOCHOBS *SDObs);
void DetectCycleSlip(const EPOCHOBSDATA* EpkA, const EPOCHOBSDATA* EpkB, SDEPOCHOBS* Obs);
void DetRefSat(const EPOCHOBSDATA* EpkA, const EPOCHOBSDATA* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs);
void FormDDEpochObs(const EPOCHOBSDATA *EpkA, const EPOCHOBSDATA *EpkB, SDEPOCHOBS *SDObs, DDCOBS *);
bool RTKFloat(RAWDAT* Raw, EPOCHOBSDATA* Base, EPOCHOBSDATA* Rov, double codeNoise, double phaseNoise);
bool RTKFix(RAWDAT* Raw, double ratioThreshold);

/* lambda */
static int LD(int n, const double *Q, double *L, double *D);
void gauss(int n, double *L, double *Z, int i, int j);
void perm(int n, double *L, double *D, int j, double del, double *Z);
void reduction(int n, double *L, double *D, double *Z);
int search(int n, int m, const double *L, const double *D, const double *zs, double *zn, double *s);
int lambda(int n, int m, const double *a, const double *Q, double *F, double *s);

/* EKFliter.cpp */

void EKFReset(bool resetBase);
bool RTKFloatEKF(RAWDAT* Raw,EPOCHOBSDATA* Base,EPOCHOBSDATA* Rov,double codeNoise,double phaseNoise,double positionNoise);

#endif
