#include "GPS_Structs.h"
using namespace std;

/********************************************************************************
	函数名称：GetSynObs

	函数目的：双站历元同步

    核心任务：从流动站和基站两个 OEM7 二进制文件中不断读取观测数据，找
            到时间相同或足够接近的两个观测历元，供后续 RTK 双差解算使用。


	输入参数：
		FBas    基站观测文件指针
		FRov    流动站观测文件指针
        BasSock 基站网络套接字
        RovSock 流动站网络套接字
        Raw     保存双站观测、星历、SPP结果和差分结果的结构体

	输出参数：
		1       基站与流动站历元同步成功
		0       本次没有成功，需要继续读取
        -1      文件读取结束或读取失败
********************************************************************************/
int GetSynObs(FILE* FBas, FILE* FRov, SOCKET& BasSock, SOCKET& RovSock, RAWDAT* Raw) {
    // 用于保存本次fread()实际读取到的字节数
    int n; 
    // 分别保存从基站和流动站文件中读出的二进制数据
    // static 意思是在函数执行结束后，缓冲区的内容不会被销毁，下一次调用该函数时，还保留上一次没有解码完的数据
    // 如果没有static，函数每次执行都会重新创建空缓冲区，上一轮遗留的半条OEM7消息就会丢失
    static unsigned char BasBuff[MAXRAWLEN], RovBuff[MAXRAWLEN]; 
    // 分别表示两个缓冲区里面当前保存了多少个有效字节
    static int BasLen = 0, RovLen = 0;

    // ①循环从流动站数据文件中读取数据，解码得到观测数据后就跳出循环，否则一直循环；如果遇到文件末尾，该函数返回。
    while (1) {
        n = (int)fread(RovBuff + RovLen, 1, MAXRAWLEN - RovLen, FRov);
        if (n < MAXRAWLEN - RovLen) {
            return -1;
        }
        RovLen += n;
        // RovBuff 待解码的流动站原始字节
        // RovLen 缓冲区当前长度，而且按照引用传递
        // &Raw->RovEpk 解码得到的流动站观测历元
        int v = DecodeNovOem7Dat(RovBuff, RovLen, &Raw->RovEpk, Raw->GpsEph, Raw->BdsEph);
        if (v == 1) break;
    }

    // ②将当前时刻的流动站观测时刻与基站观测时刻求差，得到时间差。如果流动站时刻早于基站时刻，返回同步不成功；
    // 如果时间差小于阈值，返回同步成功；如果晚于基站时间，进入第三步。
    double dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);
    if (dt<-0.5) {
        return 0;
    }
    else if(fabs(dt) < 0.5) {
        return 1;
    }

    else{
        // ③循环读取基站数据文件中读取数据，解码得到观测数据后，与流动站观测时刻求差，如果时间差小于阈值，返回同步成功；
        // 如果流动站时刻早于基站时刻，返回同步不成功；如果遇到基站文件末尾，该函数返回， RTK定位处理结束。
        while (1) {
            n = (int)fread(BasBuff + BasLen, 1, MAXRAWLEN - BasLen, FBas);
            if (n < MAXRAWLEN - BasLen) return -1;
            BasLen += n;
            int v = DecodeNovOem7Dat(BasBuff, BasLen, &Raw->BasEpk, Raw->GpsEph, Raw->BdsEph);
            if (v == 1){
                double dt = GetDiffTime(&Raw->RovEpk.Time, &Raw->BasEpk.Time);
                if(fabs(dt)<0.5) return 1;
                else if(dt<-0.5) return 0;
                else;
            }
        }
    }
}

/********************************************************************************
    函数名称：FormSDEpochObs

    函数目的：构造基站和流动站之间的站间单差观测值

    核心任务：
        对已经完成时间同步的基站和流动站观测历元进行共同卫星匹配。
        对于相同卫星系统、相同卫星号、相同频率和相同观测类型的数据，
        按照“流动站观测值减去基站观测值”的方向，计算伪距单差和
        载波相位单差，并保存两站原始观测值在数组中的索引号。

    输入参数：
        EpkA    基站观测历元，调用本函数之前必须已经完成历元同步
        EpkB    流动站观测历元，调用本函数之前必须已经完成历元同步

    输出参数：
        SDObs   站间单差观测历元，保存以下内容：
                1. 单差观测时刻
                2. 共同卫星数量
                3. 卫星系统和卫星号
                4. 双频伪距单差
                5. 双频载波相位单差
                6. 卫星在基站和流动站观测数组中的索引号
                7. 单差观测值的初始有效性标志

    返回值：
        无（void）

    单差方向：
        流动站减基站，即：
        dP = EpkB.P - EpkA.P
        dL = EpkB.L - EpkA.L

    注意事项：
        1. EpkA和EpkB必须是已经完成时间同步的观测历元。
        2. 卫星匹配必须同时比较卫星系统和卫星号。
        3. 当前版本只保留基站和流动站都具有完整双频伪距、载波
           相位数据的卫星，以便后续计算MW和GF组合。
        4. 本函数只负责构造站间单差，不负责周跳探测。
        5. 本函数不清空SDObs中的SdCObs，以保留上一历元的MW、
           GF和平滑计数，供DetectCycleSlip函数使用。
********************************************************************************/
void FormSDEpochObs(const EPOCHOBSDATA *EpkA, const EPOCHOBSDATA *EpkB, SDEPOCHOBS *SDObs){
    // 1.检查输入指针是否有效(检查空指针，防止访问无效内存)
    if (EpkA == nullptr || EpkB == nullptr || SDObs == nullptr)
    {
        return;
    }
    // 2.清空当前历元的单差卫星数据
    // 不清空SdCObs，因为其中需要保留上一历元的MW、GF历史数据
    SDObs->SatNum = 0;
    for (int i = 0; i < MAXCHANNUM; i++)
    {
        SDObs->SdSatObs[i] = SDSATOBS();
    }
    // 3.单差历元时间使用流动站观测时间
    // 此时基站和流动站应该已经完成历元同步
    SDObs->Time = EpkB->Time;
    // 4.遍历流动站的所有卫星
    for (int i = 0; i < EpkB->SatNum && i < MAXCHANNUM; i++){
        const SATOBSDATA& RovSat = EpkB->SatObs[i];
        // 5.检查流动站卫星系统和卫星号是否正常
        if (RovSat.System != GPS && RovSat.System != BDS){
            continue;
        }

        if (RovSat.Prn <= 0){
            continue;
        }

        // 6.在基站观测数据中寻找相同系统、相同PRN的卫星
        int BasIndex = -1;

        for (int j = 0; j < EpkA->SatNum && j < MAXCHANNUM; j++){
            if (EpkA->SatObs[j].System == RovSat.System && EpkA->SatObs[j].Prn == RovSat.Prn){
                BasIndex = j;
                break;
            }
        }

        // 基站没有找到这颗卫星，不能构造站间单差
        if (BasIndex == -1){
            continue;
        }

        const SATOBSDATA& BasSat = EpkA->SatObs[BasIndex];
        // 7.检查两站的双频伪距、载波相位是否完整
        bool DataComplete = true;

        for (int Freq = 0; Freq < 2; Freq++){
            if (fabs(RovSat.P[Freq]) < 1e-5 || fabs(RovSat.L[Freq]) < 1e-5 ||
                fabs(BasSat.P[Freq]) < 1e-5 || fabs(BasSat.L[Freq]) < 1e-5){
                DataComplete = false;
                break;
            }
        }

        if (DataComplete == false){
            continue;
        }

        // 防止单差卫星数量超过数组容量
        if (SDObs->SatNum >= MAXCHANNUM){
            break;
        }

        // 8.取得当前单差卫星在输出数组中的位置
        int SdIndex = SDObs->SatNum;
        SDSATOBS& SdSat = SDObs->SdSatObs[SdIndex];

        // 9.保存卫星基本信息
        SdSat.Prn = RovSat.Prn;
        SdSat.System = RovSat.System;

        // 保存该卫星在基站和流动站数组中的位置
        SdSat.nBas = BasIndex;
        SdSat.nRov = i;

        // 当前只表示数据完整，可以构造单差
        // 后续DetectCycleSlip可能会将其改为无效
        SdSat.Valid = 1;

        // 10.计算双频伪距和载波相位站间单差
        for (int Freq = 0; Freq < 2; Freq++){
            // 单差方向：流动站减基站
            SdSat.dP[Freq] = RovSat.P[Freq] - BasSat.P[Freq];

            SdSat.dL[Freq] = RovSat.L[Freq] - BasSat.L[Freq];
        }
        // 11.共同卫星数量加1
        SDObs->SatNum++;
    }
}


/********************************************************************************
    函数名称：DetectCycleSlip

    函数目的：利用站间单差的GF和MW组合探测载波相位周跳

    核心任务：
        对每颗站间单差卫星计算GF和MW组合，并与上一历元保存的组合值
        进行比较。如果GF或MW的变化超过阈值，则认为该卫星可能发生了
        周跳，并将该卫星标记为不可用；如果检测通过，则更新MW平滑值、
        GF历史值以及连续观测历元数。

    输入参数：
        Obs     当前历元的站间单差观测数据

    输出参数：
        Obs->SdSatObs[i].Valid   Obs->SdSatObs[i]表示当前历元的第i颗单差卫星
                1  表示当前卫星通过本函数的MW/GF检测
                0  表示当前卫星未通过检测
        Obs->SdCObs[i]
                保存卫星的MW、GF历史值和连续平滑历元数

    返回值：
        无（void）

    注意事项：
        1. 当前函数接口只有SDEPOCHOBS指针，无法访问两站原始观测中的
           LockTime和Parity字段，因此本版本只完成MW/GF检测。
        2. LockTime和Parity检测需要将基站、流动站观测数据作为参数传入，
           或者把相关字段复制到SDSATOBS结构体后再实现。
        3. MWGF历史按照“卫星系统 + 卫星号”匹配，不能按照数组下标匹配。
        4. MWGF::n表示已经连续通过检测的历史样本数，n为0表示没有可用历史。
********************************************************************************/
void DetectCycleSlip(const EPOCHOBSDATA* EpkA, const EPOCHOBSDATA* EpkB, SDEPOCHOBS* Obs)
{
    if (EpkA == nullptr || EpkB == nullptr || Obs == nullptr)
    {
        return;
    }

    // 这些数组保存上一历元的LockTime。
    // 下标和Obs->SdCObs[]中的历史下标一致。
    static double BaseLockTime[MAXCHANNUM][2] = {};
    static double RoverLockTime[MAXCHANNUM][2] = {};
    static bool LockHistoryValid[MAXCHANNUM] = {};

    // 记录本次处理已经使用的历史位置。
    bool HistoryUsed[MAXCHANNUM] = {};

    for (int i = 0; i < Obs->SatNum && i < MAXCHANNUM; i++){
        SDSATOBS& SdSat = Obs->SdSatObs[i];
        if (SdSat.Valid != 1){
            SdSat.Valid = 0;
            continue;
        }

        // 通过单差中的索引，找到两站的原始观测。
        int BasIndex = SdSat.nBas;
        int RovIndex = SdSat.nRov;

        const SATOBSDATA& BasSat = EpkA->SatObs[BasIndex];
        const SATOBSDATA& RovSat = EpkB->SatObs[RovIndex];

        // 找到这颗卫星原来保存的历史位置。
        int HistoryIndex = -1;

        for (int j = 0; j < MAXCHANNUM; j++){
            if (HistoryUsed[j]){
                continue;
            }
            if (Obs->SdCObs[j].System == SdSat.System && Obs->SdCObs[j].Prn == SdSat.Prn){
                HistoryIndex = j;
                break;
            }
        }

        // 第一次出现的新卫星，使用一个没有被本历元使用的位置。
        if (HistoryIndex == -1){
            for (int j = 0; j < MAXCHANNUM; j++){
                if (HistoryUsed[j] == false){
                    HistoryIndex = j;
                    break;
                }
            }
        }

        if (HistoryIndex == -1){
            SdSat.Valid = 0;
            continue;
        }

        HistoryUsed[HistoryIndex] = true;
        MWGF& History = Obs->SdCObs[HistoryIndex];

        // 只有同一颗卫星且n大于0时，才有上一历元历史。
        bool HasHistory =History.n > 0 && History.System == SdSat.System &&
            History.Prn == SdSat.Prn;

        // 检查LockTime和Parity。
        // 当前half[]保存的是OEM7的ParityFlag。
        bool LockOK = true;
        for (int Freq = 0; Freq < 2; Freq++){
            if (BasSat.half[Freq] == 0 || RovSat.half[Freq] == 0){
                LockOK = false;
            }

            if (HasHistory && LockHistoryValid[HistoryIndex]){
                if (BasSat.LockTime[Freq] < BaseLockTime[HistoryIndex][Freq] ||
                    RovSat.LockTime[Freq] < RoverLockTime[HistoryIndex][Freq]){
                    LockOK = false;
                }
            }
        }

        if (LockOK == false){
            SdSat.Valid = 0;
            History.n = 0;
            LockHistoryValid[HistoryIndex] = false;
            continue;
        }

        // 选择当前卫星所属系统的两个频率。
        double f1 = 0.0;
        double f2 = 0.0;
        if (SdSat.System == GPS){
            f1 = FG1_GPS;
            f2 = FG2_GPS;
        }
        else if (SdSat.System == BDS){
            f1 = FG1_BDS;
            f2 = FG3_BDS;
        }
        else{
            SdSat.Valid = 0;
            continue;
        }
        // GF组合：两个频率的载波相位单差之差。
        double GF = SdSat.dL[0] - SdSat.dL[1];
        // MW组合：载波部分减去伪距部分。
        double MW =(f1 * SdSat.dL[0] - f2 * SdSat.dL[1]) / (f1 - f2)
            - (f1 * SdSat.dP[0] + f2 * SdSat.dP[1]) / (f1 + f2);
        // 第一次看到这颗卫星，建立新的连续观测弧段。
        if (HasHistory == false){
            History.Prn = SdSat.Prn;
            History.System = SdSat.System;
            History.MW = MW;
            History.GF = GF;
            History.PIF = 0.0;
            History.n = 1;
            for (int Freq = 0; Freq < 2; Freq++){
                BaseLockTime[HistoryIndex][Freq] = BasSat.LockTime[Freq];
                RoverLockTime[HistoryIndex][Freq] = RovSat.LockTime[Freq];
            }
            LockHistoryValid[HistoryIndex] = true;
            SdSat.Valid = 1;
            continue;
        }

        // 和上一历元的GF、MW进行比较。
        double dGF = fabs(GF - History.GF);
        double dMW = fabs(MW - History.MW);

        if (dGF < GF_Threshold && dMW < MW_Threshold)
        {
            // 检测通过，更新MW平滑值。
            int OldCount = History.n;
            int NewCount = OldCount + 1;
            History.MW = (OldCount * History.MW + MW) / NewCount;
            History.GF = GF;
            History.n = NewCount;
            for (int Freq = 0; Freq < 2; Freq++)
            {
                BaseLockTime[HistoryIndex][Freq] = BasSat.LockTime[Freq];
                RoverLockTime[HistoryIndex][Freq] = RovSat.LockTime[Freq];
            }
            SdSat.Valid = 1;
        }
        else
        {
            // GF或MW变化过大，认为可能发生周跳。
            SdSat.Valid = 0;
            History.n = 0;
            LockHistoryValid[HistoryIndex] = false;
        }
    }
    // 当前历元没有出现的卫星，不再保留连续观测历史。
    for (int i = 0; i < MAXCHANNUM; i++)
    {
        if (HistoryUsed[i] == false)
        {
            Obs->SdCObs[i].n = 0;
            LockHistoryValid[i] = false;
        }
    }
}
void DetRefSat(const EPOCHOBSDATA* EpkA, const EPOCHOBSDATA* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs){

}
bool RTKFloat(RAWDAT* Raw, EPOCHOBSDATA* Base, EPOCHOBSDATA* Rov){

}
