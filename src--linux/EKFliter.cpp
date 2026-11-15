#include "GPS_Structs.h"

using namespace std;
using namespace Eigen;

// 保存跨历元滤波状态。
static EKFSTATE Filter;

/********************************************************************************
    函数名称：EKFReset

    函数目的：
        清除滤波状态。

    输入参数：
        resetBase = true：连同基站坐标一起清除。
        resetBase = false：保留已经保存的基站坐标。
********************************************************************************/
void EKFReset(bool resetBase)
{
    EKFSTATE empty;
    if (!resetBase && Filter.BaseSaved) {
        empty.BaseSaved = true;

        for (int k = 0; k < 3; k++)
            empty.BasePos[k] = Filter.BasePos[k];
    }

    Filter = empty;
}


/********************************************************************************
    函数名称：InitializeFilter

    函数目的：
        初始化流动站坐标、双频双差模糊度和协方差。

    状态排列：
        X = [x, y, z, N1_1, N1_2, N2_1, N2_2, ...]。

        坐标单位：m。
        模糊度单位：周。

    注意：
        当前初始协方差采用对角近似。
        初始化历元不再执行测量更新。
********************************************************************************/
bool InitializeFilter(RAWDAT& Raw, EKFSTATE& Filter)
{
    const DDCOBS& DD = Raw.DDObs;
    const SDEPOCHOBS& SD = Raw.SdObs;

    if (DD.Sats < 3 || DD.Sats > MAXCHANNUM ||
        SD.SatNum < 1 || SD.SatNum > MAXCHANNUM)
        return false;

    const double wavelength[2][2] = {
        { WL1_GPS, WL2_GPS },
        { WL1_BDS, WL3_BDS }
    };

    const double positionSigma = 10.0;       // m
    const double ambiguitySigmaMeter = 5.0;  // m

    // 1. 使用临时对象，初始化成功后再提交。
    EKFSTATE initial;

    int n = 3 + 2 * DD.Sats;

    initial.XPred = VectorXd::Zero(n);
    initial.PPred = MatrixXd::Zero(n, n);

    // 2. 固定基站坐标，流动站初值取当前有效SPP结果。
    for (int k = 0; k < 3; k++) {
        initial.BasePos[k] = Filter.BaseSaved ? Filter.BasePos[k] : Raw.BaseBest.Pos[k];

        initial.XPred(k) = Raw.RoverBest.Pos[k];
        initial.PPred(k, k) = positionSigma * positionSigma;

        if (!isfinite(initial.BasePos[k])){
            return false;
        }
    }

    initial.BaseSaved = true;

    // 3. 初始化双频模糊度。
    // ddL、ddP均为米，除以波长后得到周。
    for (int q = 0; q < DD.Sats; q++) {
        int sd = DD.SdIndex[q];

        if (sd < 0 || sd >= SD.SatNum){
            return false;
        }

        const SDSATOBS& Sat = SD.SdSatObs[sd];
        int sys = Sat.System;

        if ((sys != GPS && sys != BDS) || Sat.Valid != 1){
            return false;
        }

        for (int f = 0; f < 2; f++) {
            int index = 3 + 2 * q + f;
            double wl = wavelength[sys][f];

            initial.XPred(index) =(DD.ddL[q][f] - DD.ddP[q][f]) / wl;

            double sigma = ambiguitySigmaMeter / wl;
            initial.PPred(index, index) = sigma * sigma;
        }
    }

    // 4. 仅提交初值和历史信息，本历元不输出浮点解。
    if (!EKFSaveResult(Raw, initial, false)){
        return false;
    }
    
    Filter = initial;
    return true;
}


/********************************************************************************
    函数名称：EKFTimePrediction

    函数目的：
        根据上一历元状态，得到当前历元预测状态。

    输入参数：
        dt：时间间隔，s。
        positionNoise：位置随机游走噪声，m/根号秒。

    输出参数：
        Filter.XPred、Filter.PPred：当前预测状态及协方差。

        obsIndex：
            实际参加本历元测量更新的普通双差卫星下标。
            数组容量必须为MAXCHANNUM。

        obsNum：
            obsIndex中的有效数量，不是标量观测行数。

    注意：
        参考星切换时使用Phi转换完整状态和协方差。
        不能继承的模糊度单独初始化。
********************************************************************************/
bool EKFTimePrediction(const RAWDAT& Raw,double dt,double positionNoise,EKFSTATE& Filter,int obsIndex[],int& obsNum){
    const DDCOBS& DD = Raw.DDObs;
    const SDEPOCHOBS& SD = Raw.SdObs;

    obsNum = 0;

    Filter.XPred.resize(0);
    Filter.PPred.resize(0, 0);
    Filter.XFloat.resize(0);
    Filter.PFloat.resize(0, 0);

    int s = DD.Sats;
    int oldS = Filter.SatNum;

    if (obsIndex == nullptr ||
        !Filter.Initialized || !Filter.BaseSaved ||
        s < 3 || s > MAXCHANNUM ||
        oldS < 3 || oldS > MAXCHANNUM ||
        SD.SatNum < 1 || SD.SatNum > MAXCHANNUM ||
        !isfinite(dt) || dt <= 0.0 ||
        !isfinite(positionNoise) || positionNoise < 0.0)
        return false;

    int n = 3 + 2 * s;
    int oldN = 3 + 2 * oldS;

    if (Filter.X.size() != oldN ||
        Filter.P.rows() != oldN || Filter.P.cols() != oldN ||
        !Filter.X.allFinite() || !Filter.P.allFinite())
        return false;

    const double wavelength[2][2] = {
        { WL1_GPS, WL2_GPS },
        { WL1_BDS, WL3_BDS }
    };

    // Phi为当前状态数×上一历元状态数，可以是长方形矩阵。
    MatrixXd Phi = MatrixXd::Zero(n, oldN);
    MatrixXd Q = MatrixXd::Zero(n, n);

    bool needInit[MAXCHANNUM] = {};
    bool resetSystem[2] = { false, false };

    // 1. 位置随机游走，positionNoise单位为m/根号秒。
    for (int k = 0; k < 3; k++) {
        Phi(k, k) = 1.0;
        Q(k, k) = positionNoise * positionNoise * dt;
    }

    // 2. 检查当前参考星在上一历元是否存在、弧段是否连续。
    int refOldIndex[2] = { -1, -1 };
    bool refContinuous[2] = { false, false };

    for (int sys = 0; sys < 2; sys++) {
        if (DD.DDSatNum[sys] == 0)
            continue;

        int ref = DD.RefPos[sys];

        if (ref < 0 || ref >= SD.SatNum)
            return false;

        const SDSATOBS& Ref = SD.SdSatObs[ref];

        if (Ref.System != sys || Ref.Valid != 1 ||
            Ref.Prn != DD.RefPrn[sys])
            return false;

        // 按系统和PRN查找当前参考星的连续观测计数。
        int count = 0;

        for (int j = 0; j < MAXCHANNUM; j++) {
            const MWGF& history = SD.SdCObs[j];

            if (history.System == sys &&
                history.Prn == Ref.Prn &&
                history.n > 0) {
                count = history.n;
                break;
            }
        }

        int oldCount = 0;

        if (Ref.Prn == Filter.RefPrn[sys]) {
            oldCount = Filter.RefCount[sys];
        }
        else {
            // 新参考星可能是上一历元的普通卫星。
            for (int j = 0; j < oldS; j++) {
                if (Filter.SatSystem[j] == sys &&
                    Filter.SatPrn[j] == Ref.Prn) {
                    refOldIndex[sys] = j;
                    oldCount = Filter.SatCount[j];
                    break;
                }
            }
        }

        // 按相邻处理历元的连续观测计数判断。
        refContinuous[sys] =
            oldCount > 0 &&
            static_cast<long long>(count) ==
            static_cast<long long>(oldCount) + 1;
    }

    // 3. 普通卫星匹配与参考星转换。
    for (int q = 0; q < s; q++) {
        int sd = DD.SdIndex[q];

        if (sd < 0 || sd >= SD.SatNum)
            return false;

        const SDSATOBS& Sat = SD.SdSatObs[sd];
        int sys = Sat.System;

        if ((sys != GPS && sys != BDS) ||
            Sat.Valid != 1 ||
            DD.DDSatNum[sys] <= 0 ||
            Sat.Prn == DD.RefPrn[sys])
            return false;

        int count = 0;

        for (int j = 0; j < MAXCHANNUM; j++) {
            const MWGF& history = SD.SdCObs[j];

            if (history.System == sys &&
                history.Prn == Sat.Prn &&
                history.n > 0) {
                count = history.n;
                break;
            }
        }

        int oldIndex = -1;
        int oldCount = 0;

        // 当前普通卫星可能是上一历元的参考星。
        bool wasReference = Sat.Prn == Filter.RefPrn[sys];

        if (wasReference) {
            oldCount = Filter.RefCount[sys];
        }
        else {
            for (int j = 0; j < oldS; j++) {
                if (Filter.SatSystem[j] == sys &&
                    Filter.SatPrn[j] == Sat.Prn) {
                    oldIndex = j;
                    oldCount = Filter.SatCount[j];
                    break;
                }
            }
        }

        bool continuous =
            oldCount > 0 &&
            static_cast<long long>(count) ==
            static_cast<long long>(oldCount) + 1;

        bool sameReference =
            DD.RefPrn[sys] == Filter.RefPrn[sys];

        bool canTransfer =
            continuous &&
            refContinuous[sys] &&
            (wasReference || oldIndex >= 0) &&
            (sameReference || refOldIndex[sys] >= 0);

        if (!canTransfer) {
            needInit[q] = true;
            resetSystem[sys] = true;
            continue;
        }

        for (int f = 0; f < 2; f++) {
            int row = 3 + 2 * q + f;

            // 旧参考星相对自身的双差为0。
            if (!wasReference)
                Phi(row, 3 + 2 * oldIndex + f) = 1.0;

            // N(j,newRef)=N(j,oldRef)-N(newRef,oldRef)。
            if (!sameReference)
                Phi(row, 3 + 2 * refOldIndex[sys] + f) = -1.0;

            Q(row, row) = 1E-10 * dt; // 周²
        }

        obsIndex[obsNum++] = q;
    }

    // 4. 出现新初始化模糊度的系统，本历元暂缓测量更新。
    // 同系统双差共用参考星，当前观测噪声存在相关性。
    // 原地整理索引，不需要额外容器。
    int used = 0;

    for (int a = 0; a < obsNum; a++) {
        int q = obsIndex[a];
        int sys = SD.SdSatObs[DD.SdIndex[q]].System;

        if (!resetSystem[sys])
            obsIndex[used++] = q;
    }

    obsNum = used;

    // 5. 传播状态和完整协方差。
    VectorXd XPred = Phi * Filter.X;

    MatrixXd PPred =
        Phi * Filter.P * Phi.transpose() + Q;

    // 6. 初始化无法继承的模糊度，采用对角协方差近似。
    // 对应Phi行保持为0，不继承无效的旧交叉协方差。
    const double ambiguitySigmaMeter = 5.0;

    for (int q = 0; q < s; q++) {
        if (!needInit[q])
            continue;

        int sys = SD.SdSatObs[DD.SdIndex[q]].System;

        for (int f = 0; f < 2; f++) {
            int index = 3 + 2 * q + f;
            double wl = wavelength[sys][f];

            XPred(index) =
                (DD.ddL[q][f] - DD.ddP[q][f]) / wl;

            double sigma = ambiguitySigmaMeter / wl;
            PPred(index, index) = sigma * sigma;
        }
    }

    // 7. 检查并保存预测结果。
    if (!XPred.allFinite() || !PPred.allFinite() ||
        (PPred.diagonal().array() < 0.0).any())
        return false;

    Filter.XPred = XPred;
    Filter.PPred =
        (0.5 * (PPred + PPred.transpose())).eval();

    return true;
}


/********************************************************************************
    函数名称：EKFMeasurementUpdate

    函数目的：
        构造H、W、R，完成浮点状态和协方差更新。

    输入参数：
        obsIndex、obsNum：
            实际参加测量更新的普通双差卫星下标及数量。

        codeNoise、phaseNoise：
            单站、单星原始观测标准差，单位m。

    返回值：
        EKF_FAILED：计算失败。
        EKF_INITIALIZED：没有可用于测量更新的观测，仅提交状态。
        EKF_FLOAT：完成浮点测量更新。

    注意：
        观测行按obsIndex排列，状态列仍按完整DD顺序排列。
********************************************************************************/
int EKFMeasurementUpdate(const RAWDAT& Raw,const int obsIndex[],int obsNum,double codeNoise,double phaseNoise,EKFSTATE& Filter)
{
    const DDCOBS& DD = Raw.DDObs;
    const SDEPOCHOBS& SD = Raw.SdObs;
    const EPOCHOBSDATA& Base = Raw.BasEpk;
    const EPOCHOBSDATA& Rov = Raw.RovEpk;

    Filter.XFloat.resize(0);
    Filter.PFloat.resize(0, 0);

    if (DD.Sats < 3 || DD.Sats > MAXCHANNUM ||
        SD.SatNum < 1 || SD.SatNum > MAXCHANNUM ||
        Base.SatNum < 1 || Base.SatNum > MAXCHANNUM ||
        Rov.SatNum < 1 || Rov.SatNum > MAXCHANNUM ||
        obsIndex == nullptr ||
        obsNum < 0 || obsNum > DD.Sats ||
        !Filter.BaseSaved ||
        !isfinite(codeNoise) || codeNoise <= 0.0 ||
        !isfinite(phaseNoise) || phaseNoise <= 0.0)
        return EKF_FAILED;

    int n = 3 + 2 * DD.Sats;
    int m = 4 * obsNum;

    if (Filter.XPred.size() != n ||
        Filter.PPred.rows() != n || Filter.PPred.cols() != n ||
        !Filter.XPred.allFinite() || !Filter.PPred.allFinite())
        return EKF_FAILED;

    if (obsNum == 0)
        return EKF_INITIALIZED;

    const double wavelength[2][2] = {
        { WL1_GPS, WL2_GPS },
        { WL1_BDS, WL3_BDS }
    };

    MatrixXd H = MatrixXd::Zero(m, n);
    VectorXd W = VectorXd::Zero(m);
    MatrixXd R = MatrixXd::Zero(m, m);

    int systems[MAXCHANNUM] = {};

    Vector3d rover = Filter.XPred.head<3>();

    Vector3d base(
        Filter.BasePos[0],
        Filter.BasePos[1],
        Filter.BasePos[2]);

    // 1. 构造H和创新W，每颗使用的卫星对应P1、P2、L1、L2。
    for (int a = 0; a < obsNum; a++) {
        int q = obsIndex[a];

        if (q < 0 || q >= DD.Sats)
            return EKF_FAILED;

        // 防止重复使用同一颗卫星。
        for (int b = 0; b < a; b++) {
            if (obsIndex[b] == q)
                return EKF_FAILED;
        }

        int sd = DD.SdIndex[q];

        if (sd < 0 || sd >= SD.SatNum)
            return EKF_FAILED;

        const SDSATOBS& Sat = SD.SdSatObs[sd];
        int sys = Sat.System;

        if ((sys != GPS && sys != BDS) || Sat.Valid != 1)
            return EKF_FAILED;

        systems[a] = sys;

        int ref = DD.RefPos[sys];

        if (ref < 0 || ref >= SD.SatNum)
            return EKF_FAILED;

        const SDSATOBS& Ref = SD.SdSatObs[ref];

        if (Ref.System != sys || Ref.Valid != 1 ||
            Ref.Prn != DD.RefPrn[sys] || Ref.Prn == Sat.Prn)
            return EKF_FAILED;

        int bj = Sat.nBas;
        int rj = Sat.nRov;
        int bi = Ref.nBas;
        int ri = Ref.nRov;

        if (bj < 0 || bj >= Base.SatNum ||
            bi < 0 || bi >= Base.SatNum ||
            rj < 0 || rj >= Rov.SatNum ||
            ri < 0 || ri >= Rov.SatNum)
            return EKF_FAILED;

        if (!Base.SatPVT[bj].Valid || !Base.SatPVT[bi].Valid ||
            !Rov.SatPVT[rj].Valid || !Rov.SatPVT[ri].Valid)
            return EKF_FAILED;

        // 两站到普通星、参考星的距离向量。
        Vector3d dBj, dBi, dRj, dRi;

        for (int k = 0; k < 3; k++) {
            dBj(k) = base(k) - Base.SatPVT[bj].SatPos[k];
            dBi(k) = base(k) - Base.SatPVT[bi].SatPos[k];

            dRj(k) = rover(k) - Rov.SatPVT[rj].SatPos[k];
            dRi(k) = rover(k) - Rov.SatPVT[ri].SatPos[k];
        }

        double rhoBj = dBj.norm();
        double rhoBi = dBi.norm();
        double rhoRj = dRj.norm();
        double rhoRi = dRi.norm();

        if (!isfinite(rhoBj) || !isfinite(rhoBi) ||
            !isfinite(rhoRj) || !isfinite(rhoRi) ||
            rhoBj <= 0.0 || rhoBi <= 0.0 ||
            rhoRj <= 0.0 || rhoRi <= 0.0)
            return EKF_FAILED;

        double rhoDD = rhoRj - rhoBj - rhoRi + rhoBi;
        Vector3d g = dRj / rhoRj - dRi / rhoRi;

        int row = 4 * a;

        for (int t = 0; t < 4; t++)
            H.block<1, 3>(row + t, 0) = g.transpose();

        for (int f = 0; f < 2; f++) {
            // 状态列按照完整双差卫星顺序确定。
            int index = 3 + 2 * q + f;
            double wl = wavelength[sys][f];

            W(row + f) = DD.ddP[q][f] - rhoDD;

            W(row + 2 + f) =
                DD.ddL[q][f] - rhoDD -
                wl * Filter.XPred(index);

            H(row + 2 + f, index) = wl;
        }
    }

    // 2. 构造双差观测协方差。
    // 同系统、同频、同类型：对角4σ²，非对角2σ²。
    for (int a = 0; a < obsNum; a++) {
        for (int b = 0; b < obsNum; b++) {
            if (systems[a] != systems[b])
                continue;

            for (int t = 0; t < 4; t++) {
                double sigma =
                    t < 2 ? codeNoise : phaseNoise;

                R(4 * a + t, 4 * b + t) =
                    (a == b ? 4.0 : 2.0) * sigma * sigma;
            }
        }
    }

    if (!H.allFinite() || !W.allFinite() || !R.allFinite())
        return EKF_FAILED;

    // 3. 求卡尔曼增益，使用分解求解而非显式求逆。
    MatrixXd PHt = Filter.PPred * H.transpose();
    MatrixXd S = H * PHt + R;

    if (!S.allFinite())
        return EKF_FAILED;

    LDLT<MatrixXd> solver(S);

    if (solver.info() != Success ||
        (solver.vectorD().array() <= 0.0).any())
        return EKF_FAILED;

    // S*K转置=PHt转置。
    MatrixXd Kt = solver.solve(PHt.transpose());

    if (solver.info() != Success || !Kt.allFinite())
        return EKF_FAILED;

    MatrixXd K = Kt.transpose();

    // 4. 更新状态，W已是创新，不再减H*XPred。
    VectorXd XFloat = Filter.XPred + K * W;

    // 5. 协方差采用Joseph形式。
    MatrixXd A = MatrixXd::Identity(n, n) - K * H;

    MatrixXd PFloat =
        A * Filter.PPred * A.transpose() +
        K * R * K.transpose();

    if (!XFloat.allFinite() || !PFloat.allFinite() ||
        (PFloat.diagonal().array() < 0.0).any())
        return EKF_FAILED;

    Filter.XFloat = XFloat;
    Filter.PFloat =
        (0.5 * (PFloat + PFloat.transpose())).eval();

    return EKF_FLOAT;
}


/********************************************************************************
    函数名称：EKFSaveResult

    函数目的：
        保存当前状态、协方差、时间及卫星历史；
        测量更新成功时，将浮点结果写入DDObs。

    输入参数：
        hasMeasurement = true：
            提交XFloat、PFloat，同时输出浮点结果。

        hasMeasurement = false：
            只提交XPred、PPred，不输出可供整数固定的浮点结果。

    注意：
        此方案持续递推浮点状态。
        外部RTKFix的整数固定结果不反馈到Filter。
********************************************************************************/
bool EKFSaveResult(RAWDAT& Raw,EKFSTATE& Filter,bool hasMeasurement)
{
    DDCOBS& DD = Raw.DDObs;
    const SDEPOCHOBS& SD = Raw.SdObs;

    const VectorXd& X =
        hasMeasurement ? Filter.XFloat : Filter.XPred;

    const MatrixXd& P =
        hasMeasurement ? Filter.PFloat : Filter.PPred;

    int s = DD.Sats;

    if (s < 3 || s > MAXCHANNUM ||
        SD.SatNum < 1 || SD.SatNum > MAXCHANNUM ||
        !Filter.BaseSaved ||
        !isfinite(Raw.RovEpk.Time.SecOfWeek))
        return false;

    int n = 3 + 2 * s;

    if (X.size() != n || P.rows() != n || P.cols() != n ||
        !X.allFinite() || !P.allFinite() ||
        (P.diagonal().array() < 0.0).any())
        return false;

    for (int k = 0; k < 3; k++) {
        if (!isfinite(Filter.BasePos[k]))
            return false;
    }

    // 1. 先检查历史信息，全部成功后再覆盖旧状态。
    int satSystem[MAXCHANNUM] = {};
    int satPrn[MAXCHANNUM] = {};
    int satCount[MAXCHANNUM] = {};

    int refPrn[2] = { -1, -1 };
    int refCount[2] = {};
    int systemCount[2] = {};

    // 前两项为GPS、BDS参考星，后面为普通卫星。
    for (int entry = 0; entry < s + 2; entry++) {
        int q = entry - 2;
        int sd;

        if (entry < 2) {
            if (DD.DDSatNum[entry] == 0)
                continue;

            sd = DD.RefPos[entry];
        }
        else {
            sd = DD.SdIndex[q];
        }

        if (sd < 0 || sd >= SD.SatNum)
            return false;

        const SDSATOBS& Sat = SD.SdSatObs[sd];
        int sys = Sat.System;

        if ((sys != GPS && sys != BDS) || Sat.Valid != 1)
            return false;

        if (entry < 2 &&
            (sys != entry || Sat.Prn != DD.RefPrn[sys]))
            return false;

        if (entry >= 2 && Sat.Prn == DD.RefPrn[sys])
            return false;

        // 按系统和PRN查找历史，不能直接沿用当前单差下标。
        int count = 0;

        for (int j = 0; j < MAXCHANNUM; j++) {
            const MWGF& history = SD.SdCObs[j];

            if (history.System == sys &&
                history.Prn == Sat.Prn &&
                history.n > 0) {
                count = history.n;
                break;
            }
        }

        if (count <= 0)
            return false;

        if (entry < 2) {
            refPrn[sys] = Sat.Prn;
            refCount[sys] = count;
        }
        else {
            // 普通双差卫星不能重复。
            for (int j = 0; j < q; j++) {
                if (satSystem[j] == sys &&
                    satPrn[j] == Sat.Prn)
                    return false;
            }

            satSystem[q] = sys;
            satPrn[q] = Sat.Prn;
            satCount[q] = count;

            systemCount[sys]++;
        }
    }

    for (int sys = 0; sys < 2; sys++) {
        if (systemCount[sys] != DD.DDSatNum[sys])
            return false;
    }

    // 2. 保存下一历元使用的状态和历史。
    Filter.X = X;
    Filter.P = P;

    Filter.SatNum = s;
    Filter.LastTime = Raw.RovEpk.Time;

    Filter.Initialized = true;
    Filter.IsFixed = false;
    Filter.Ratio = 0.0;

    for (int q = 0; q < MAXCHANNUM; q++) {
        Filter.SatSystem[q] = satSystem[q];
        Filter.SatPrn[q] = satPrn[q];
        Filter.SatCount[q] = satCount[q];
    }

    for (int sys = 0; sys < 2; sys++) {
        Filter.RefPrn[sys] = refPrn[sys];
        Filter.RefCount[sys] = refCount[sys];
    }

    // 3. 清理输出，只有完成测量更新才填入浮点结果。
    DD.bFixed = false;
    DD.Ratio = 0.0;

    DD.ResAmb[0] = DD.ResAmb[1] = 0.0;
    DD.FixRMS[0] = DD.FixRMS[1] = 0.0;

    DD.Sigma0 = 0.0;
    DD.QFloat.resize(0, 0);

    for (int k = 0; k < 3; k++) {
        DD.dPos[k] = 0.0;
        DD.FloatRMS[k] = 0.0;
    }

    for (int k = 0; k < MAXCHANNUM * 4; k++) {
        DD.FloatAmb[k] = 0.0;
        DD.FixedAmb[k] = 0.0;
    }

    if (!hasMeasurement)
        return true;

    // 4. 输出浮点解。
    DD.QFloat = Filter.PFloat;
    DD.Sigma0 = 1.0; // 不再乘LS后验方差因子。

    for (int k = 0; k < 3; k++) {
        DD.dPos[k] =
            Filter.XFloat(k) - Filter.BasePos[k];

        DD.FloatRMS[k] =
            sqrt(Filter.PFloat(k, k));
    }

    for (int k = 0; k < 2 * s; k++)
        DD.FloatAmb[k] = Filter.XFloat(3 + k);

    return true;
}


/********************************************************************************
    函数名称：RTKFloatEKF

    函数目的：
        处理一个历元的EKF浮点解，保持当前main.cpp的调用接口。

    调用前：
        已完成两站同步、SPP、单差构造、周跳探测、
        参考星选取和双差构造。

    参数约定：
        Base应为&Raw->BasEpk。
        Rov应为&Raw->RovEpk。

    返回值：
        true：完成测量更新，DDObs中有可供RTKFix使用的浮点结果。
        false：仅初始化、仅提交预测状态，或者处理失败。
********************************************************************************/
bool RTKFloatEKF(RAWDAT* Raw,EPOCHOBSDATA* Base,EPOCHOBSDATA* Rov,double codeNoise,double phaseNoise,double positionNoise)
{
    if (Raw == nullptr) {
        EKFReset(false);
        return false;
    }

    // 1. 清理旧输出；失败时不能复用上一历元结果。
    DDCOBS& DD = Raw->DDObs;

    DD.bFixed = false;
    DD.Ratio = 0.0;

    DD.ResAmb[0] = DD.ResAmb[1] = 0.0;
    DD.FixRMS[0] = DD.FixRMS[1] = 0.0;

    DD.Sigma0 = 0.0;
    DD.QFloat.resize(0, 0);

    for (int k = 0; k < 3; k++) {
        DD.dPos[k] = 0.0;
        DD.FloatRMS[k] = 0.0;
    }

    for (int k = 0; k < MAXCHANNUM * 4; k++) {
        DD.FloatAmb[k] = 0.0;
        DD.FixedAmb[k] = 0.0;
    }

    if (Base != &Raw->BasEpk || Rov != &Raw->RovEpk ||
        DD.Sats < 3 || DD.Sats > MAXCHANNUM ||
        !isfinite(codeNoise) || codeNoise <= 0.0 ||
        !isfinite(phaseNoise) || phaseNoise <= 0.0 ||
        !isfinite(positionNoise) || positionNoise < 0.0) {
        EKFReset(false);
        return false;
    }

    // 2. 首次初始化。
    if (!Filter.Initialized) {
        if (!InitializeFilter(*Raw, Filter))
            EKFReset(false);

        return false;
    }

    // 3. 时间间隔检查，GetDiffTime包含GPS周差。
    double dt = GetDiffTime(&Rov->Time, &Filter.LastTime);

    if (!isfinite(dt) || dt <= 0.0) {
        EKFReset(false);
        return false;
    }

    // 沿用现有中断门限，单位s。
    const double maxGap = 10.0;

    if (dt > maxGap) {
        if (!InitializeFilter(*Raw, Filter))
            EKFReset(false);

        return false;
    }

    // 4. 时间更新。
    int obsIndex[MAXCHANNUM] = {};
    int obsNum = 0;

    if (!EKFTimePrediction(
            *Raw, dt, positionNoise,
            Filter, obsIndex, obsNum)) {
        EKFReset(false);
        return false;
    }

    // 5. 测量更新。
    int status = EKFMeasurementUpdate(
        *Raw, obsIndex, obsNum,
        codeNoise, phaseNoise, Filter);

    if (status == EKF_FAILED) {
        EKFReset(false);
        return false;
    }

    // 6. 无观测更新时只提交状态，不进行整数固定。
    bool hasMeasurement = status == EKF_FLOAT;

    if (!EKFSaveResult(*Raw, Filter, hasMeasurement)) {
        EKFReset(false);
        return false;
    }

    return hasMeasurement;
}