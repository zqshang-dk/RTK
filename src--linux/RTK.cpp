#include "GPS_Structs.h"
#include <cerrno>
#include <cstring>
using namespace std;

/********************************************************************************
    函数名称：ReadRTKEpoch

    函数目的：从文件或网络中读取一个完整的OEM7观测历元

    输入参数：
        fp          文件指针，文件模式使用
        sock        网络套接字，网络模式使用
        inputMode   1=文件模式，0=网络模式
        buff        保存原始字节的缓冲区
        len         缓冲区中当前有效字节数
        Obs         保存解码得到的观测历元
        Raw         提供GPS、BDS星历存储空间
        station     用于提示信息，例如"Base"或"Rover"

    返回值：
        1           成功得到一个观测历元
        -1          文件结束、连接关闭或读取出错
********************************************************************************/
int ReadRTKEpoch(FILE* fp, SocketHandle sock, int inputMode, unsigned char buff[], int& len, EPOCHOBSDATA* Obs, RAWDAT* Raw, const char station[]){
    while (true){
        // 1. 先处理缓冲区里已经收到的数据
        int result = DecodeNovOem7Dat(buff, len, Obs, Raw->GpsEph, Raw->BdsEph);
        // 返回1：表示得到一个观测历元
        if (result == 1){
            return 1;
        }
        // 2. 没有完整历元，需要继续读取
        int freeLength = MAXRAWLEN - len;
        if (freeLength <= 0){
            cout << "Error: " << station << " input buffer is full." << endl;
            return -1;
        }
        int n = 0;
        if (inputMode == INPUT_MODE_FILE) {
            // 文件模式：从文件读取字节
            n = (int)fread(buff + len, 1, freeLength, fp);
            if (n == 0){
                if (ferror(fp)){
                    cout << "Error: Failed to read " << station << " file." << endl;
                }
                else{
                    cout << station << " file reached the end." << endl;
                }
                return -1;
            }
        }
        else {
            // 网络模式：从TCP连接接收字节
            n = recv(sock, (char*)buff + len, freeLength, 0);
            if (n == 0) {
                cout << station << " connection was closed." << endl;
                return -1;
            }
            if (n < 0) {
                cout << "Error: Failed to receive " << station << " data. Code="
                     << errno << " (" << strerror(errno) << ")" << endl;
                return -1;
            }
        }
        // 3. 只要收到正数个字节，就加入缓冲区
        len += n;
        // 回到while开头，再尝试解码
    }
}
/********************************************************************************
    函数名称：GetSynObs

    函数目的：读取基站和流动站数据，完成双站观测历元同步

    输入参数：
        FBas        基站文件指针
        FRov        流动站文件指针
        BasSock     基站网络套接字
        RovSock     流动站网络套接字
        Raw         保存观测数据和星历
        inputMode   1=文件模式，0=网络模式

    返回值：
        1           双站历元同步成功
        0           流动站历元较早，下次继续读取流动站
        -1          输入结束、连接关闭或读取失败
********************************************************************************/
int GetSynObs(FILE* FBas, FILE* FRov,SocketHandle& BasSock, SocketHandle& RovSock,RAWDAT* Raw,int inputMode) {
    // 保存两站尚未处理完的原始字节
    static unsigned char BasBuff[MAXRAWLEN]{};
    static unsigned char RovBuff[MAXRAWLEN]{};
    static int BasLen = 0;
    static int RovLen = 0;
    // 是否已有一个等待配对的基站历元
    static bool hasBaseEpoch = false;
    // 允许两站观测时间相差的秒数
    const double syncTolerance = 0.01;
    // 1. 每次调用，先读取一个新的流动站历元
    int result = ReadRTKEpoch(FRov,RovSock,inputMode,RovBuff,RovLen,&Raw->RovEpk,Raw,"Rover");
    if (result == -1) {
        return -1;
    }
    // 2. 查找与这个流动站历元匹配的基站历元
    while (true) {
        if (!hasBaseEpoch) {  //如果没有待匹配的基站，就读取一个
            result = ReadRTKEpoch(FBas,BasSock,inputMode,BasBuff,BasLen,&Raw->BasEpk,Raw,"Base");
            if (result == -1) {
                return -1;
            }
            hasBaseEpoch = true;
        }
        // 时间差 = 流动站观测时间 - 基站观测时间
        double dt = GetDiffTime(&Raw->RovEpk.Time,&Raw->BasEpk.Time);
        // 3. 两站时间足够接近，同步成功
        if (fabs(dt) < syncTolerance) {
            // 这个基站历元已经配对，下一次需要读取新的
            hasBaseEpoch = false;
            return 1;
        }
        // 4. 流动站较早：保留基站，下次读取新的流动站
        if (dt < 0.0) {
            return 0;
        }
        // 5. 基站较早：读取下一个基站历元继续比较
        hasBaseEpoch = false;
    }
}

/********************************************************************************
    函数名称：FormSDEpochObs

    函数目的：构造基站和流动站之间的站间单差观测值（同一颗卫星在Rover和Base的观测值相减）

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
    // 1.检查输入指针是否有效
    if (EpkA == nullptr || EpkB == nullptr || SDObs == nullptr) {
        return;
    }
    // 2.清空当前历元的单差卫星数据（不清空SdCObs，因为其中需要保留上一历元的MWGF历史数据）
    SDObs->SatNum = 0;
    for (int i = 0; i < MAXCHANNUM; i++) {
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

        // 6.在基站(EpkA)观测数据中寻找相同系统、相同PRN的卫星
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

        const SATOBSDATA& BasSat = EpkA->SatObs[BasIndex];  //取得对应的Base观测
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
void DetectCycleSlip(const EPOCHOBSDATA* EpkA, const EPOCHOBSDATA* EpkB, SDEPOCHOBS* Obs){
    if (EpkA == nullptr || EpkB == nullptr || Obs == nullptr) {
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
        bool HasHistory =History.n > 0 && History.System == SdSat.System && History.Prn == SdSat.Prn;

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
        double MW =(f1 * SdSat.dL[0] - f2 * SdSat.dL[1]) / (f1 - f2) - (f1 * SdSat.dP[0] + f2 * SdSat.dP[1]) / (f1 + f2);
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

        if (dGF < GF_Threshold && dMW < MW_Threshold) {
            // 检测通过，更新MW平滑值。
            int OldCount = History.n;
            int NewCount = OldCount + 1;
            History.MW = (OldCount * History.MW + MW) / NewCount;
            History.GF = GF;
            History.n = NewCount;
            for (int Freq = 0; Freq < 2; Freq++) {
                BaseLockTime[HistoryIndex][Freq] = BasSat.LockTime[Freq];
                RoverLockTime[HistoryIndex][Freq] = RovSat.LockTime[Freq];
            }
            SdSat.Valid = 1;
        }
        else {
            // GF或MW变化过大，认为可能发生周跳。
            SdSat.Valid = 0;
            History.n = 0;
            LockHistoryValid[HistoryIndex] = false;
        }
    }
    // 当前历元没有出现的卫星，不再保留连续观测历史。
    for (int i = 0; i < MAXCHANNUM; i++) {
        if (HistoryUsed[i] == false) {
            Obs->SdCObs[i].n = 0;
            LockHistoryValid[i] = false;
        }
    }
}
/********************************************************************************
    函数名称：DetRefSat

    函数目的：分别为GPS和BDS选取用于星间双差的基准星

    核心任务：
        遍历基站和流动站的共同卫星，先检查站间单差观测是否有效，
        以及两站对应卫星的位置是否计算成功。

        根据单差观测中保存的nBas和nRov索引，访问两站原始观测，
        检查双频载波的奇偶状态、载噪比和连续锁定时间等条件。

        在满足质量要求和高度角门限的候选卫星中，分别为GPS和BDS
        选择流动站高度角最大的卫星作为基准星，并保存其卫星号
        和在单差观测数组中的位置，供后续构造星间双差使用。

    输入参数：
        EpkA    基站观测历元，提供卫星原始观测和卫星位置计算结果

        EpkB    流动站观测历元，提供卫星原始观测、载噪比、
                连续锁定时间和高度角等信息

        SDObs   站间单差观测历元，调用前应已经完成共同卫星匹配、
                站间单差构造和周跳探测，提供：
                1. 共同卫星数量SatNum
                2. 卫星系统System和卫星号Prn
                3. 单差观测有效性标志Valid
                4. 对应基站观测数组的索引nBas
                5. 对应流动站观测数组的索引nRov

    输出参数：
        DDObs   在双差信息结构体中保存基准星选取结果：
                RefPrn[0]    GPS基准星的卫星号
                RefPos[0]    GPS基准星在SdSatObs[]中的下标
                RefPrn[1]    BDS基准星的卫星号
                RefPos[1]    BDS基准星在SdSatObs[]中的下标

                某个系统没有满足要求的基准星时，该系统对应的
                RefPrn和RefPos均设为-1。

    返回值：
        无（void），通过DDObs中的RefPrn和RefPos判断选取结果。

    选取原则：
        1. GPS和BDS分别选取基准星，同一系统内构造星间双差。
        2. 先进行质量检查，再比较高度角。
        3. 单差观测必须有效，两站卫星位置计算必须成功。
        4. 两站双频载波的奇偶状态必须可靠。
        5. 流动站双频载噪比和连续锁定时间必须满足设定门限。
        6. 在达到高度角门限的合格候选星中，选择流动站高度角
           最大的卫星。

    注意事项：
        1. 调用前，两站历元必须已经同步，并完成卫星位置和高度角计算，以及站间单差构造和周跳探测。
        2. 每个历元开始选取时，应先将两个系统的RefPrn和RefPos设为-1，避免沿用上一历元已经失效的选取结果。
        3. RefPos是SDObs->SdSatObs[]中的下标，不是原始观测数组SatObs[]中的下标。访问两站原始观测时，应通过该单差
           卫星记录中的nBas和nRov进行索引转换。
        4. Elevation的单位为弧度。若高度角门限使用度表示，必须先乘以Deg2Rad，再进行比较。
        5. 当前解码程序中，half[]保存的是Parity known标志：1表示奇偶状态已知，0表示未知，不能把half等于1
           直接理解为发生了半周问题。
        6. 本函数只负责选取基准星，不计算双差观测值，也不进行模糊度固定；不修改SDObs中的单差观测和周跳历史数据。
        7. 若某系统的RefPos为-1，后续不能使用该下标访问数组，也不能为该系统构造依赖此基准星的星间双差。
********************************************************************************/
void DetRefSat(const EPOCHOBSDATA* EpkA, const EPOCHOBSDATA* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs){
    double MinElevation = 40.0 * Deg2Rad;  //最低高度角

    double MaxElevation[2] = {0.0, 0.0};
    for (int i = 0; i < 2;i++){
        DDObs->RefPos[i] = -1;
        DDObs->RefPrn[i] = -1;
    }
    // 以站间单差卫星数量进行循环，先检查单差数据是否被标记为粗差
    // 以及基站与流动站的卫星位置计算是否成功，
    // 若不通过质量检查则continue
    for (int i = 0; i < SDObs->SatNum;i++){
        // 检查单差数据是否被标记为粗差
        if (SDObs->SdSatObs[i].Valid != 1){
            continue;
        }
        // 检查基站和流动站的卫星位置是否计算成功
        if (EpkA->SatPVT[SDObs->SdSatObs[i].nBas].Valid == false ||
                EpkB->SatPVT[SDObs->SdSatObs[i].nRov].Valid == false) {
            continue;
        }
        // 检查两站两个频率的奇偶状态（没有半周问题）
        if (EpkA->SatObs[SDObs->SdSatObs[i].nBas].half[0] == 0 ||
                EpkA->SatObs[SDObs->SdSatObs[i].nBas].half[1] == 0 ||
                EpkB->SatObs[SDObs->SdSatObs[i].nRov].half[0] == 0 ||
                EpkB->SatObs[SDObs->SdSatObs[i].nRov].half[1] == 0) {
            continue;
        }
        // 检查流动站的载噪比（大于35dBHz）
        if (EpkB->SatObs[SDObs->SdSatObs[i].nRov].cn0[0] <= 35.0 ||
                EpkB->SatObs[SDObs->SdSatObs[i].nRov].cn0[1] <= 35.0) {
            continue;
        }
        // 连续观测时间大于6s
        if (EpkB->SatObs[SDObs->SdSatObs[i].nRov].LockTime[0] < 6.0 ||
                EpkB->SatObs[SDObs->SdSatObs[i].nRov].LockTime[1] < 6.0) {
            continue;
        }
        //高度角
        if (EpkB->SatPVT[SDObs->SdSatObs[i].nRov].Elevation < MinElevation) {
            continue;
        }
        int sys;
        if (SDObs->SdSatObs[i].System == GPS) {
            sys = 0;
        }
        else if (SDObs->SdSatObs[i].System == BDS) {
            sys = 1;
        }
        else continue;

        // 通过质量检查后，选取高度角最大且流动站两个频率的CN0大于某个阈值（如35dBHz）的单差数据，
        // 将其卫星号和数组下标保存到RefPrn和RefPos数组。
        // 基准站常架设在观测环境良好的情况下，参考星主要以流动站数据的质量为参考。

        if (EpkB->SatPVT[SDObs->SdSatObs[i].nRov].Elevation > MaxElevation[sys]) {
            MaxElevation[sys] = EpkB->SatPVT[SDObs->SdSatObs[i].nRov].Elevation;
            DDObs->RefPrn[sys] = SDObs->SdSatObs[i].Prn;
            DDObs->RefPos[sys] = i;
        }
    }

}

// 站星双差观测值
void FormDDEpochObs(const EPOCHOBSDATA* EpkA, const EPOCHOBSDATA* EpkB, SDEPOCHOBS* SDObs, DDCOBS* DDObs){
    // 清空双差卫星计数和各系统计数
    DDObs->Sats = 0;
    DDObs->DDSatNum[0] = 0;
    DDObs->DDSatNum[1] = 0;
    DDObs->bFixed = false;

    // sys=0为GPS，sys=1为BDS
    for (int sys = 0; sys < 2; sys++) {
        int ref = DDObs->RefPos[sys];
        if (ref < 0 || ref >= SDObs->SatNum){ //该系统没有参考卫星就跳过
            continue;
        }
            
        const SDSATOBS& Ref = SDObs->SdSatObs[ref];
        if (Ref.System != sys || Ref.Valid != 1){
            continue;
        }

        int refBas = Ref.nBas;
        int refRov = Ref.nRov;

        if (!EpkA->SatPVT[refBas].Valid || !EpkB->SatPVT[refRov].Valid){
            continue;
        }
            
        for (int j = 0; j < SDObs->SatNum; j++) {
            const SDSATOBS& Sat = SDObs->SdSatObs[j];

            if (j == ref || Sat.System != sys || Sat.Valid != 1){
                continue;
            }
            int nBas = Sat.nBas;
            int nRov = Sat.nRov;

            if (!EpkA->SatPVT[nBas].Valid || !EpkB->SatPVT[nRov].Valid){
                continue;
            }

            // 当前half保存Parity known：0未知，1已知
            bool usable = true;
            for (int f = 0; f < 2; f++) {
                if (EpkA->SatObs[nBas].half[f] == 0 || EpkB->SatObs[nRov].half[f] == 0 ||
                    EpkA->SatObs[refBas].half[f] == 0 || EpkB->SatObs[refRov].half[f] == 0){
                        usable = false;
                    }
            }

            if (!usable){
                continue;
            }
            //记录该双差卫星对应的单差数组的下标
            int q = DDObs->Sats;
            DDObs->SdIndex[q] = j;

            // 普通星单差减本系统参考星单差，结果都是米
            for (int f = 0; f < 2; f++) {
                DDObs->ddP[q][f] = Sat.dP[f] - Ref.dP[f];
                DDObs->ddL[q][f] = Sat.dL[f] - Ref.dL[f];
            }
            DDObs->Sats++;
            DDObs->DDSatNum[sys]++;
        }
    }
}


// 相对定位浮点解
bool RTKFloat(RAWDAT* Raw, EPOCHOBSDATA* Base, EPOCHOBSDATA* Rov, double codeNoise, double phaseNoise){
    DDCOBS& DD = Raw->DDObs;
    SDEPOCHOBS& SD = Raw->SdObs;

    // 清除上次浮点结果；成功与否以返回值为准
    DD.bFixed = false;
    DD.QFloat.resize(0, 0);
    DD.Sigma0 = 0.0;
    for (int k = 0; k < 3; k++) {
        DD.dPos[k] = 0.0;
        DD.FloatRMS[k] = 0.0;
    }
    for (int k = 0; k < MAXCHANNUM * 2; k++){
        DD.FloatAmb[k] = 0.0;
    }
    int d = DD.Sats;           // 非参考双差卫星数，少于3颗卫星就不解算
    if (d < 3){
        return false;
    }

    int obsNum = 4 * d;        // 每星P1、P2、L1、L2
    int unknownNum = 3 + 2*d;  // 三维坐标、每星两个模糊度（2个频率）

    // 使用配置文件中的CodeNoise和CarrierNoise，单位为米。
    if (!isfinite(codeNoise) || !isfinite(phaseNoise) || codeNoise <= 0.0 || phaseNoise <= 0.0){
            return false;
        }

    double wl[2][2] = {
        { WL1_GPS, WL2_GPS },
        { WL1_BDS, WL3_BDS }
    };
    //取 Base、Rover 的初始坐标；它们来自前面计算的单点定位结果。
    double BasePos[3], RovPos[3];
    for (int k = 0; k < 3; k++) {
        BasePos[k] = Raw->BaseBest.Pos[k];
        RovPos[k] = Raw->RoverBest.Pos[k];
    }

    // 模糊度以“周”为单位，初始值不取整
    VectorXd Amb(2*d); 
    for (int q = 0; q < d; q++) {
        int sd = DD.SdIndex[q];
        int sys = SD.SdSatObs[sd].System;
        for (int f = 0; f < 2; f++){
            //Amb=(DD载波相位 - DD伪距) / lambda
            Amb(2*q + f) = (DD.ddL[q][f] - DD.ddP[q][f]) / wl[sys][f];
        }
    }

    // 构造相关权阵：同系统、同频、同类型之间保留相关项
    MatrixXd P = MatrixXd::Zero(obsNum, obsNum);
    for (int q = 0; q < d; q++) {
        int sys = SD.SdSatObs[DD.SdIndex[q]].System;
        int ds = DD.DDSatNum[sys];

        for (int r = 0; r < d; r++) {
            int otherSys = SD.SdSatObs[DD.SdIndex[r]].System;
            if (otherSys != sys){
                continue;
            }

            for (int t = 0; t < 4; t++) {
                double sigma = codeNoise;
                if (t >= 2){
                    sigma = phaseNoise;
                }
                double w = -1.0 / (2.0 * sigma * sigma * (ds + 1));

                if (q == r){
                    w = ds /(2.0 * sigma * sigma * (ds + 1));
                }
                P(4*q + t, 4*r + t) = w;
            }
        }
    }

    for (int iteration = 0; iteration < MAXIteration; iteration++) {
        MatrixXd B = MatrixXd::Zero(obsNum, unknownNum);
        VectorXd W = VectorXd::Zero(obsNum);

        for (int q = 0; q < d; q++) {
            const SDSATOBS& Sat = SD.SdSatObs[DD.SdIndex[q]];
            int sys = Sat.System;
            const SDSATOBS& Ref = SD.SdSatObs[DD.RefPos[sys]];

            int jBas = Sat.nBas, jRov = Sat.nRov;
            int iBas = Ref.nBas, iRov = Ref.nRov;

            // 两站分别使用各自已经计算好的卫星坐标
            double rhoBj = Distance(BasePos, Base->SatPVT[jBas].SatPos);
            double rhoRj = Distance(RovPos, Rov->SatPVT[jRov].SatPos);
            double rhoBi = Distance(BasePos, Base->SatPVT[iBas].SatPos);
            double rhoRi = Distance(RovPos, Rov->SatPVT[iRov].SatPos);

            double rhoDD = rhoRj - rhoBj - rhoRi + rhoBi;
            int row = 4*q;

            // 四行的前三列相同，都是几何距离的偏导
            for (int k = 0; k < 3; k++) {
                double g = (RovPos[k] - Rov->SatPVT[jRov].SatPos[k]) / rhoRj
                  - (RovPos[k] - Rov->SatPVT[iRov].SatPos[k]) / rhoRi;

                for (int t = 0; t < 4; t++){
                    B(row + t, k) = g;
                }
            }

            for (int f = 0; f < 2; f++) {
                W(row + f) = DD.ddP[q][f] - rhoDD;
                W(row + 2 + f) = DD.ddL[q][f] - rhoDD - wl[sys][f] * Amb(2*q + f);
                B(row + 2 + f, 3 + 2*q + f) = wl[sys][f];
            }
        }

        // 数量足够还不代表可解，必须检查几何秩
        if (B.fullPivLu().rank() < unknownNum){
            return false;
        }

        MatrixXd Normal = B.transpose() * P * B;
        VectorXd U = B.transpose() * P * W;

        // 仍然求解正规方程，用LDLT代替显式求逆
        LDLT<MatrixXd> solver(Normal);
        if (solver.info() != Success || !solver.isPositive()){
            return false;
        }

        VectorXd delta = solver.solve(U);
        if (solver.info() != Success || !delta.allFinite()){
            return false;
        }
        for (int k = 0; k < 3; k++){
            RovPos[k] += delta(k);
        }

        for (int k = 0; k < 2*d; k++){
            Amb(k) += delta(3 + k);
        }
        double correction = sqrt(delta(0)*delta(0) + delta(1)*delta(1) + delta(2)*delta(2));

        if (correction < 1E-4) {
            // 后验残差：不能直接用本轮求解前的W评定精度
            VectorXd v = W - B * delta;
            double variance = v.dot(P * v) / (obsNum - unknownNum);

            MatrixXd Q = solver.solve(MatrixXd::Identity(unknownNum, unknownNum));

            if (solver.info() != Success || !Q.allFinite() || !isfinite(variance) || variance < 0.0){
                return false;
            }

            for (int k = 0; k < 3; k++){
                if (Q(k, k) < 0.0){
                    return false;
                }
            }

            DD.QFloat = Q;
            DD.Sigma0 = sqrt(variance);

            for (int k = 0; k < 3; k++) {
                DD.dPos[k] = RovPos[k] - BasePos[k];
                DD.FloatRMS[k] = DD.Sigma0 * sqrt(Q(k, k));
            }

            for (int k = 0; k < 2*d; k++){
                DD.FloatAmb[k] = Amb(k);
            }

            return true;
        }
    }
    return false; // 超过迭代次数仍未收敛
}

/********************************************************************************
    函数名称：RTKFix

    函数目的：搜索整数模糊度，并在Ratio检验通过后计算固定基线

    输入参数：
        Raw             当前历元数据，调用前RTKFloat必须成功
        ratioThreshold  Ratio检验门限，来自配置文件

    输出参数：
        Raw->DDObs.FixedAmb  最优和次优整数模糊度候选
        Raw->DDObs.ResAmb    两组候选的目标函数值
        Raw->DDObs.Ratio     Ratio值
        Raw->DDObs.dPos      成功时更新为固定基线
        Raw->DDObs.bFixed    当前历元是否固定成功

    返回值：
        true    接受固定，并完成基线更新
        false   未接受固定，保留浮点基线
********************************************************************************/
bool RTKFix(RAWDAT* Raw, double ratioThreshold){
    DDCOBS& DD = Raw->DDObs;

    // 1. 清除上一历元的固定状态和检验结果
    DD.bFixed = false;
    DD.Ratio = 0.0;
    DD.ResAmb[0] = 0.0;
    DD.ResAmb[1] = 0.0;

    // 每颗普通双差卫星对应两个频率的模糊度
    int n = 2 * DD.Sats;

    // 当前lambda中的求逆函数最多处理250维
    if (n <= 0 || n > 250){
        return false;
    }
    if (DD.QFloat.rows() != 3 + n || DD.QFloat.cols() != 3 + n){
        return false;
    }

    // 2. 提取模糊度矩阵、坐标与模糊度的交叉矩阵
    MatrixXd Qaa = DD.QFloat.block(3, 3, n, n);
    MatrixXd Qba = DD.QFloat.block(0, 3, 3, n);

    // 3. 搜索最优和次优两组整数模糊度
    int info = lambda(n, 2, DD.FloatAmb, Qaa.data(), DD.FixedAmb, DD.ResAmb);

    if (info != 0){
        return false;
    }

    // 4. 检查候选代价，避免无效数值和除以零
    if (!isfinite(DD.ResAmb[0]) || !isfinite(DD.ResAmb[1]) ||
        DD.ResAmb[0] <= 0.0 || DD.ResAmb[1] < DD.ResAmb[0]){
            return false;
        }
    DD.Ratio = DD.ResAmb[1] / DD.ResAmb[0];

    if (!isfinite(DD.Ratio) || DD.Ratio < ratioThreshold){
        return false;
    }
    // 5. 计算“固定模糊度减浮点模糊度”
    VectorXd dAmb(n);

    for (int i = 0; i < n; i++) {
        dAmb(i) = DD.FixedAmb[i] - DD.FloatAmb[i];
    }

    // 6. 根据模糊度变化，计算固定基线改正
    LDLT<MatrixXd> solver(Qaa);

    if (solver.info() != Success || !solver.isPositive()){
        return false;
    }

    VectorXd u = solver.solve(dAmb);

    if (solver.info() != Success || !u.allFinite()){
        return false;
    }

    VectorXd dBaseline = Qba * u;

    if (!dBaseline.allFinite()){
        return false;
    }

    // 7. 所有计算成功后，再更新基线并标记固定
    for (int i = 0; i < 3; i++) {
        DD.dPos[i] += dBaseline(i);
    }
    DD.bFixed = true;
    return true;
}
