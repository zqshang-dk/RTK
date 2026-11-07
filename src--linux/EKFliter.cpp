#include "GPS_Structs.h"
using namespace std;

/********************************************************************************
    结构体名称：EKFSTATE

    作用：
        保存上一历元的滤波结果，供下一历元继续使用。

    状态向量X的排列：
        X(0)、X(1)、X(2)：流动站ECEF坐标，单位为米。

        X(3)、X(4)：第0颗普通双差卫星的双频模糊度。
        X(5)、X(6)：第1颗普通双差卫星的双频模糊度。
        后面依此类推，模糊度单位为周。

    P：
        状态向量X的协方差矩阵。

    注意：
        BaseSaved与Initialized分开保存。
        重新初始化模糊度时，不应随意更换已经固定的基站坐标。
********************************************************************************/
struct EKFSTATE
{
    bool Initialized = false;
    bool BaseSaved = false;

    int SatNum = 0;

    // 记录各个模糊度对应的普通卫星。
    int SatSystem[MAXCHANNUM] = {};
    int SatPrn[MAXCHANNUM] = {};

    // 记录GPS和BDS参考星的卫星号。
    int RefPrn[2] = { -1, -1 };

    // 固定使用的基站坐标。
    double BasePos[3] = {};

    // 上一次初始化或更新对应的时刻。
    GPSTIME LastTime;

    VectorXd X;
    MatrixXd P;
};

// 保存在本文件中，函数返回后数据仍然存在。
static EKFSTATE Filter;

/********************************************************************************
    函数名称：EKFReset

    函数目的：通知滤波器重新初始化

    输入参数：
        resetBase = true：
            连同基站坐标一起重新初始化。
            开始处理一组新的数据时使用。

        resetBase = false：
            重新初始化滤波状态，但保留原来的基站坐标。
            历元中断、数据不满足解算条件时使用。

    返回值：
        无。
********************************************************************************/
void EKFReset(bool resetBase)
{
    Filter.Initialized = false;

    if (resetBase)
    {
        Filter.BaseSaved = false;
    }
}


/********************************************************************************
    函数名称：EKFGetWavelength

    函数目的：根据卫星系统和频率槽位取得波长

    输入参数：
        system：GPS或BDS。
        freq：0表示第一频率，1表示第二频率。

    注意：
        当前工程中，BDS第二频率使用B3，对应WL3_BDS。
********************************************************************************/
static double EKFGetWavelength(int system, int freq)
{
    if (system == GPS)
    {
        if (freq == 0)
            return WL1_GPS;

        return WL2_GPS;
    }

    if (system == BDS)
    {
        if (freq == 0)
            return WL1_BDS;

        return WL3_BDS;
    }

    return 0.0;
}


/********************************************************************************
    函数名称：EKFSameSatellites

    函数目的：
        检查当前双差模糊度的含义，是否与上一历元一致。

    检查内容：
        1. 普通双差卫星数量是否相同。
        2. 实际参与解算的系统，其参考星是否相同。
        3. 普通卫星的排列顺序、系统和卫星号是否相同。

    返回值：
        true：可以继续使用原来的滤波状态。
        false：需要重新初始化。

    注意：
        不直接比较SdIndex。
        单差数组下标变化，并不一定表示卫星发生变化。
********************************************************************************/
static bool EKFSameSatellites(const RAWDAT* Raw)
{
    const DDCOBS& DD = Raw->DDObs;
    const SDEPOCHOBS& SD = Raw->SdObs;

    if (DD.Sats != Filter.SatNum){
        return false;
    }
    for (int sys = 0; sys < 2; sys++)
    {
        if (DD.DDSatNum[sys] > 0 && DD.RefPrn[sys] != Filter.RefPrn[sys])
        {
            return false;
        }
    }
    for (int q = 0; q < DD.Sats; q++)
    {
        int sd = DD.SdIndex[q];
        const SDSATOBS& Sat = SD.SdSatObs[sd];
        if (Sat.System != Filter.SatSystem[q] || Sat.Prn != Filter.SatPrn[q])
        {
            return false;
        }
    }
    return true;
}

/********************************************************************************
    函数名称：EKFInitialize

    函数目的：初始化位置、模糊度及其协方差

    初始化方法：
        位置初值：当前历元成功的流动站SPP结果。
        模糊度初值：(双差相位 - 双差伪距) / 波长。

    初始标准差：
        位置：10米。
        模糊度：先在距离域取5米，再除以波长转换成周。

    注意：
        初始协方差采用对角阵，是便于学习的近似。
        本函数不进行测量更新。
********************************************************************************/
static void EKFInitialize(RAWDAT* Raw, EPOCHOBSDATA* Rov)
{
    const DDCOBS& DD = Raw->DDObs;
    const SDEPOCHOBS& SD = Raw->SdObs;
    int stateNum = 3 + 2 * DD.Sats;
    Filter.X = VectorXd::Zero(stateNum);
    Filter.P = MatrixXd::Zero(stateNum, stateNum);

    // 基站坐标只在首次使用，或明确要求重设基站时保存。
    if (!Filter.BaseSaved)
    {
        for (int k = 0; k < 3; k++)
        {
            Filter.BasePos[k] = Raw->BaseBest.Pos[k];
        }

        Filter.BaseSaved = true;
    }

    // 1. 初始化流动站坐标。
    const double positionSigma = 10.0;
    for (int k = 0; k < 3; k++)
    {
        Filter.X(k) = Raw->RoverBest.Pos[k];
        Filter.P(k, k) = positionSigma * positionSigma;
    }

    // 2. 初始化双频模糊度。
    const double ambiguitySigmaMeter = 5.0;
    for (int q = 0; q < DD.Sats; q++)
    {
        int sd = DD.SdIndex[q];
        const SDSATOBS& Sat = SD.SdSatObs[sd];
        Filter.SatSystem[q] = Sat.System;
        Filter.SatPrn[q] = Sat.Prn;
        for (int f = 0; f < 2; f++)
        {
            double wavelength = EKFGetWavelength(Sat.System, f);
            int index = 3 + 2 * q + f;
            Filter.X(index) = (DD.ddL[q][f] - DD.ddP[q][f]) / wavelength;
            // 距离标准差转换为模糊度标准差，单位为周。
            double ambiguitySigma = ambiguitySigmaMeter / wavelength;
            Filter.P(index, index) = ambiguitySigma * ambiguitySigma;
        }
    }
    // 3. 保存当前的卫星配置和时刻。
    Filter.SatNum = DD.Sats;
    for (int sys = 0; sys < 2; sys++)
    {
        Filter.RefPrn[sys] = DD.RefPrn[sys];
    }

    Filter.LastTime = Rov->Time;
    Filter.Initialized = Filter.X.allFinite() && Filter.P.allFinite();
}

/********************************************************************************
    函数名称：EKFPredict

    函数目的：时间更新

    模型：
        状态转移矩阵Phi = I。

        X预测 = X上次。
        P预测 = P上次 + Q。

    输入参数：
        dt：
            两次滤波时刻之差，单位为秒。

        positionNoise：
            位置随机游走噪声强度，单位为米/根号秒。
            位置过程方差 = positionNoise² * dt。

    注意：
        无周跳时，模糊度应基本保持不变。
********************************************************************************/
static void EKFPredict(double dt, double positionNoise)
{
    // Phi = I，所以状态均值保持不变，不需要修改Filter.X。
    double positionVariance = positionNoise * positionNoise * dt;
    // 模糊度过程噪声谱密度：1E-10 周²/秒。
    double ambiguityVariance = 1E-10 * dt;
    for (int k = 0; k < 3; k++)
    {
        Filter.P(k, k) += positionVariance;
    }
    int stateNum = static_cast<int>(Filter.X.size());
    for (int k = 3; k < stateNum; k++)
    {
        Filter.P(k, k) += ambiguityVariance;
    }
}

/********************************************************************************
    函数名称：EKFBuildObservation

    函数目的：构造观测矩阵H和创新W

    输出参数：
        H：观测函数对状态的偏导矩阵。
        W：实际观测值减预测观测值。

    每颗普通卫星对应四行：
        P1、P2、L1、L2。

    注意：
        W已经减去预测观测值。
        后面不能再对W减一次H * X。
********************************************************************************/
static bool EKFBuildObservation(RAWDAT* Raw,EPOCHOBSDATA* Base,EPOCHOBSDATA* Rov,MatrixXd& H,VectorXd& W)
{
    const DDCOBS& DD = Raw->DDObs;
    const SDEPOCHOBS& SD = Raw->SdObs;
    int obsNum = 4 * DD.Sats;
    int stateNum = 3 + 2 * DD.Sats;
    H = MatrixXd::Zero(obsNum, stateNum);
    W = VectorXd::Zero(obsNum);
    double roverPos[3];
    for (int k = 0; k < 3; k++)
    {
        roverPos[k] = Filter.X(k);
    }
    for (int q = 0; q < DD.Sats; q++)
    {
        const SDSATOBS& Sat = SD.SdSatObs[DD.SdIndex[q]];
        int sys = Sat.System;
        const SDSATOBS& Ref = SD.SdSatObs[DD.RefPos[sys]];
        int jBas = Sat.nBas;
        int jRov = Sat.nRov;
        int iBas = Ref.nBas;
        int iRov = Ref.nRov;
        // 1. 计算两站到普通星、参考星的四个距离。
        double rhoBj = Distance(Filter.BasePos, Base->SatPVT[jBas].SatPos);
        double rhoRj = Distance(roverPos, Rov->SatPVT[jRov].SatPos);
        double rhoBi = Distance(Filter.BasePos, Base->SatPVT[iBas].SatPos);
        double rhoRi = Distance(roverPos, Rov->SatPVT[iRov].SatPos);

        if (rhoRj <= 0.0 || rhoRi <= 0.0)
            return false;

        // 2. 双差几何距离：普通星单差减参考星单差。
        double rhoDD = rhoRj - rhoBj - rhoRi + rhoBi;
        int row = 4 * q;
        // 3. 填写四行观测对位置的偏导。
        for (int k = 0; k < 3; k++)
        {
            double g = (roverPos[k] - Rov->SatPVT[jRov].SatPos[k]) / rhoRj
              - (roverPos[k] - Rov->SatPVT[iRov].SatPos[k]) / rhoRi;

            for (int t = 0; t < 4; t++)
            {
                H(row + t, k) = g;
            }
        }
        // 4. 填写双频创新，以及相位对模糊度的偏导。
        for (int f = 0; f < 2; f++)
        {
            double wavelength = EKFGetWavelength(sys, f);
            int ambIndex = 3 + 2 * q + f;
            // 伪距预测值是双差几何距离。
            W(row + f) = DD.ddP[q][f] - rhoDD;
            // 相位预测值是双差几何距离加lambda*N。
            W(row + 2 + f) = DD.ddL[q][f] - rhoDD - wavelength * Filter.X(ambIndex);
            H(row + 2 + f, ambIndex) = wavelength;
        }
    }
    return H.allFinite() && W.allFinite();
}

/********************************************************************************
    函数名称：EKFBuildR

    函数目的：构造双差观测协方差矩阵R

    输入参数：
        codeNoise：
            单站、单星原始伪距的标准差，单位为米。

        phaseNoise：
            单站、单星原始载波相位的标准差，单位为米。

    随机模型：
        同系统、同频、同类型：

        同一条双差的方差：4 * sigma²。
        不同双差的协方差：2 * sigma²。

        不同系统、不同频率、不同类型之间暂设为不相关。

    注意：
        这里的R是协方差矩阵，不是最小二乘中的权阵。
********************************************************************************/
static MatrixXd EKFBuildR(const RAWDAT* Raw,double codeNoise,double phaseNoise)
{
    const DDCOBS& DD = Raw->DDObs;
    const SDEPOCHOBS& SD = Raw->SdObs;
    int obsNum = 4 * DD.Sats;
    MatrixXd R = MatrixXd::Zero(obsNum, obsNum);
    for (int q = 0; q < DD.Sats; q++)
    {
        int sysQ = SD.SdSatObs[DD.SdIndex[q]].System;
        for (int r = 0; r < DD.Sats; r++)
        {
            int sysR = SD.SdSatObs[DD.SdIndex[r]].System;
            if (sysQ != sysR){
                continue;
            }
            for (int t = 0; t < 4; t++)
            {
                double sigma = codeNoise;
                if (t >= 2)
                {
                    sigma = phaseNoise;
                }
                double covariance = 2.0 * sigma * sigma;
                if (q == r)
                {
                    covariance = 4.0 * sigma * sigma;
                }
                R(4 * q + t, 4 * r + t) = covariance;
            }
        }
    }
    return R;
}

/********************************************************************************
    函数名称：EKFUpdate

    函数目的：利用当前历元观测，更新状态和协方差

    主要公式：
        S = H * P * H转置 + R
        K = P * H转置 * S逆
        X更新 = X预测 + K * W

        A = I - K * H
        P更新 = A * P预测 * A转置 + K * R * K转置

    注意：
        使用Joseph形式更新协方差。
        用Eigen的LDLT求解矩阵方程，避免手写矩阵求逆。
********************************************************************************/
static bool EKFUpdate(const MatrixXd& H,const VectorXd& W,const MatrixXd& R)
{
    // 1. 创新协方差。
    MatrixXd PHt = Filter.P * H.transpose();
    MatrixXd S = H * PHt + R;
    LDLT<MatrixXd> solver(S);
    if (solver.info() != Success || !solver.isPositive()){
        return false;
    }
    // 2. 求卡尔曼增益。
    // K = PHt * S逆。
    // 因为S对称，所以可以先解：
    // S * K转置 = PHt转置。
    MatrixXd KTranspose = solver.solve(PHt.transpose());
    if (solver.info() != Success){
        return false;
    }
    MatrixXd K = KTranspose.transpose();
    // 3. 更新状态。
    VectorXd newX = Filter.X + K * W;
    // 4. Joseph形式更新协方差。
    int stateNum = static_cast<int>(Filter.X.size());
    MatrixXd I = MatrixXd::Identity(stateNum, stateNum);
    MatrixXd A = I - K * H;
    MatrixXd newP = A * Filter.P * A.transpose()+ K * R * K.transpose();
    if (!newX.allFinite() || !newP.allFinite()){
        return false;
    }
    // 方差不能为负数。
    for (int k = 0; k < stateNum; k++)
    {
        if (newP(k, k) < 0.0){
            return false;
        }
    }
    // 5. 全部计算成功后，保存新状态。
    Filter.X = newX;
    // 减少浮点运算造成的微小不对称。
    Filter.P = 0.5 * (newP + newP.transpose());
    return true;
}


/********************************************************************************
    函数名称：EKFSaveResult

    函数目的：把滤波结果保存到现有的DDObs结构体中

    输出内容：
        dPos：流动站减基站的ECEF基线，单位为米。
        FloatAmb：双频浮点模糊度，单位为周。
        QFloat：EKF状态协方差矩阵。
        FloatRMS：坐标分量的形式标准差，单位为米。

    注意：
        这里的P已经是状态协方差，不再乘LS的后验方差因子。

        Sigma0设为1，只表示不进行额外缩放，
        不是计算出了一个等于1的LS后验标准差因子。
********************************************************************************/
static void EKFSaveResult(RAWDAT* Raw)
{
    DDCOBS& DD = Raw->DDObs;
    DD.QFloat = Filter.P;
    DD.Sigma0 = 1.0;
    for (int k = 0; k < 3; k++)
    {
        DD.dPos[k] = Filter.X(k) - Filter.BasePos[k];
        DD.FloatRMS[k] = std::sqrt(Filter.P(k, k));
    }
    for (int k = 0; k < 2 * DD.Sats; k++)
    {
        DD.FloatAmb[k] = Filter.X(3 + k);
    }
    DD.bFixed = false;
}

/********************************************************************************
    函数名称：RTKFloatEKF

    函数目的：处理一个历元的EKF相对定位浮点解

    调用前必须完成：
        1. 双站历元同步。
        2. 当期两站SPP和卫星状态计算。
        3. 站间单差构造、周跳探测。
        4. GPS和BDS参考星选取。
        5. 双差观测构造。

    输入参数：
        Raw             当前RTK数据。
        Base            基站观测历元。
        Rov             流动站观测历元。
        codeNoise       原始伪距标准差，单位为米。
        phaseNoise      原始载波相位标准差，单位为米。
        positionNoise   位置随机游走噪声，单位为米/根号秒。

    返回值：
        true：
            本历元完成滤波更新，DDObs中有可使用的浮点结果。

        false：
            本历元没有输出滤波结果。
            可能是刚完成初始化，也可能是数据或计算不满足要求。

    教学版本的简化：
        1. 至少使用3颗普通双差卫星。
        2. 卫星列表或参考星变化时，整体重新初始化。
        3. 时间间隔超过10秒时，整体重新初始化。
        4. 不在这里执行整数模糊度固定。
********************************************************************************/
bool RTKFloatEKF(RAWDAT* Raw,EPOCHOBSDATA* Base,EPOCHOBSDATA* Rov,double codeNoise,double phaseNoise,double positionNoise)
{
    if (Raw == nullptr || Base == nullptr || Rov == nullptr)
    {
        EKFReset(false);
        return false;
    }
    DDCOBS& DD = Raw->DDObs;
    // 当前是否有有效结果，以函数返回值为准。
    DD.bFixed = false;
    DD.Ratio = 0.0;
    DD.ResAmb[0] = DD.ResAmb[1] = 0.0;
    DD.Sigma0 = 0.0;
    DD.QFloat.resize(0, 0);

    for (int k = 0; k < 3; k++)
    {
        DD.dPos[k] = 0.0;
        DD.FloatRMS[k] = 0.0;
    }

    if (DD.Sats < 3 || DD.Sats > MAXCHANNUM ||
        !isfinite(codeNoise) || codeNoise <= 0.0 ||
        !isfinite(phaseNoise) || phaseNoise <= 0.0 ||
        !isfinite(positionNoise) || positionNoise < 0.0)
    {
        EKFReset(false);
        return false;
    }

    // 1. 首次使用，或者双差模糊度的对应关系发生变化。
    if (!Filter.Initialized || !EKFSameSatellites(Raw))
    {
        EKFInitialize(Raw, Rov);
        // 当前观测已用于初始化，不再作为独立观测更新一次。
        return false;
    }
    // 2. 计算时间间隔。
    double dt = GetDiffTime(&Rov->Time, &Filter.LastTime);
    if (!isfinite(dt) || dt <= 0.0)
    {
        EKFReset(false);
        return false;
    }
    // 10秒是本示例的中断处理门限，不是课件规定值。
    const double maxGap = 10.0;
    if (dt > maxGap)
    {
        EKFInitialize(Raw, Rov);
        return false;
    }
    // 3. 时间更新。
    EKFPredict(dt, positionNoise);
    // 4. 构造观测矩阵和创新。
    MatrixXd H;
    VectorXd W;
    if (!EKFBuildObservation(Raw, Base, Rov, H, W))
    {
        EKFReset(false);
        return false;
    }
    // 5. 构造观测协方差。
    MatrixXd R = EKFBuildR(Raw, codeNoise, phaseNoise);
    // 6. 测量更新。
    if (!EKFUpdate(H, W, R))
    {
        EKFReset(false);
        return false;
    }
    // 7. 保存本历元结果，供输出和下一历元使用。
    Filter.LastTime = Rov->Time;
    EKFSaveResult(Raw);
    return true;
}