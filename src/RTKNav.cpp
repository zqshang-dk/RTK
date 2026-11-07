// RTKNav.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。

#include "GPS_Structs.h"
#include <iostream>
#include <cmath>
#include <Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <iomanip>

using namespace Eigen;
using namespace std;

int main()
{
    ROVERCFGINFO config{};

    if (!ReadRTKConfigInfo("RTKConfig.ini", &config))
    {
        // 配置读取失败
        cout << "Error: Failed to read RTK configuration." << endl;
        return 1;
    }

    if (config.PositionMode == POSITION_MODE_RTK)
    {
        cout << "Positioning mode: RTK" << endl;

        if (config.RTKProcMode != 1 && config.RTKProcMode != 2)
        {
            cout << "Error: Unsupported RTK processing mode." << endl;
            return 1;
        }
        // 文件模式使用这两个文件指针
        FILE* FBas = nullptr;
        FILE* FRov = nullptr;

        // 网络模式使用这两个套接字
        SOCKET BasSock = INVALID_SOCKET;
        SOCKET RovSock = INVALID_SOCKET;

        if (config.IsFileData == INPUT_MODE_FILE)
        {
            cout << "Input mode: FILE" << endl;
            cout << "Base observation file: "<< config.BasObsDatFile << endl;
            cout << "Rover observation file: "<< config.RovObsDatFile << endl;

            FBas = fopen(config.BasObsDatFile, "rb");
            FRov = fopen(config.RovObsDatFile, "rb");

            if (FBas == nullptr || FRov == nullptr)
            {
                cout << "Error: Cannot open observation files." << endl;

                if (FBas != nullptr)
                {
                    fclose(FBas);
                }

                if (FRov != nullptr)
                {
                    fclose(FRov);
                }

                return 1;
            }
        }
        else
        {
            cout << "Input mode: NETWORK" << endl;

            // 连接基站
            cout << "Connecting to base: "<< config.BasNetIP << ":"
                << config.BasNetPort << endl;

            if (!OpenSocket(BasSock,config.BasNetIP,config.BasNetPort))
            {
                cout << "Error: Cannot connect to base." << endl;
                return 1;
            }
            cout << "Base connected." << endl;
            // 连接流动站
            cout << "Connecting to rover: "
                << config.RovNetIP << ":"
                << config.RovNetPort << endl;

            if (!OpenSocket(RovSock,config.RovNetIP,config.RovNetPort))
            {
                cout << "Error: Cannot connect to rover." << endl;
                // 基站已经连接成功，退出前关闭
                CloseSocket(BasSock);
                return 1;
            }

            cout << "Rover connected." << endl;
            cout << "Waiting for synchronized OEM7 observations..." << endl;
        }

        // 打开结果文件
        ofstream rtkLog(config.ResFile, ios::out);

        if (!rtkLog.is_open())
        {
            cout << "Error: Cannot create RTK result file: "
                << config.ResFile << endl;

            if (FBas != nullptr)
            {
                fclose(FBas);
            }

            if (FRov != nullptr)
            {
                fclose(FRov);
            }

            CloseSocket(BasSock);
            CloseSocket(RovSock);

            return 1;
        }

        // 保存两站观测、星历以及定位结果
        RAWDAT Raw{};
        // 开始处理一组数据，允许重新保存基站坐标。
        EKFReset(true);

        // 教学用起始值，单位为米/根号秒。
        // 应结合静态或运动情况调整。
        const double positionNoise = 1.0;
        const double elevationThreshold = config.ElevThreshold * Deg2Rad;

        int totalEpochCount = 0;
        int floatSuccessCount = 0;

        while (true)
        {
            int syncResult = GetSynObs(FBas, FRov, BasSock, RovSock, &Raw,config.IsFileData);

            // -1表示文件结束或读取失败
            if (syncResult == -1)
            {
                break;
            }

            // 0表示当前两个历元没有同步成功
            if (syncResult == 0)
            {
                continue;
            }

            // 只有返回1时，两个历元才是同步的
            totalEpochCount++;
            if (totalEpochCount == 1)
            {
                cout << "First synchronized epoch received. WEEK="
                    << Raw.RovEpk.Time.Week
                    << " SOW=" << Raw.RovEpk.Time.SecOfWeek
                    << endl;
            }

            // 先对基站和流动站分别进行粗差探测
            DetectOutlier(&Raw.BasEpk);
            DetectOutlier(&Raw.RovEpk);

            // 构造站间单差
            FormSDEpochObs(&Raw.BasEpk, &Raw.RovEpk, &Raw.SdObs);

            // 探测站间单差中的周跳
            DetectCycleSlip(&Raw.BasEpk, &Raw.RovEpk, &Raw.SdObs);

            // 分别进行基站和流动站SPP
            bool baseOK = SPP(Raw.GpsEph, Raw.BdsEph, &Raw.BaseBest, &Raw.BasEpk, elevationThreshold);

            bool roverOK = SPP(Raw.GpsEph, Raw.BdsEph, &Raw.RoverBest, &Raw.RovEpk, elevationThreshold);

            if (!baseOK || !roverOK)
            {
                EKFReset(false);
                continue;
            }

            // 选择GPS和BDS基准星
            DetRefSat(&Raw.BasEpk, &Raw.RovEpk, &Raw.SdObs, &Raw.DDObs);

            // 构造星间双差
            FormDDEpochObs(&Raw.BasEpk, &Raw.RovEpk, &Raw.SdObs, &Raw.DDObs);

            // 每颗双差卫星有4个观测值，
            // 至少需要3颗普通双差卫星才能估计3个坐标和模糊度
            if (Raw.DDObs.Sats < 3)
            {
                EKFReset(false);
                continue;
            }

            // 检查每个实际参与双差的系统是否有参考星
            bool referenceOK = true;

            for (int sys = 0; sys < 2; sys++)
            {
                if (Raw.DDObs.DDSatNum[sys] > 0 && Raw.DDObs.RefPos[sys] < 0)
                {
                    referenceOK = false;
                }
            }

            if (!referenceOK)
            {
                EKFReset(false);
                continue;
            }

            // 当前RTKFloat计算的是浮点解
            bool floatOK = false;

            if (config.RTKProcMode == 1){
                floatOK = RTKFloatEKF(&Raw,&Raw.BasEpk,&Raw.RovEpk,config.CodeNoise,config.CPNoise,positionNoise);
            }
            else{
                floatOK = RTKFloat(&Raw,&Raw.BasEpk,&Raw.RovEpk);
            }
            if (!floatOK){
                continue;
            }

            floatSuccessCount++;
            
            // 浮点解成功后，尝试模糊度固定
            bool fixedOK = RTKFix(&Raw, config.RatioThres);

            if (fixedOK)
            {
                cout << "Epoch " << totalEpochCount
                    << ": RTK fixed solution succeeded." << endl;
            }
            else
            {
                cout << "Epoch " << totalEpochCount
                    << ": RTK float solution retained." << endl;
            }

            if (config.RTKProcMode == 2)
            {
                cout << "Ratio: " << Raw.DDObs.Ratio << endl;
            }

            cout << "Baseline ECEF [m]: "
                << "dX=" << Raw.DDObs.dPos[0] << ", "
                << "dY=" << Raw.DDObs.dPos[1] << ", "
                << "dZ=" << Raw.DDObs.dPos[2] << endl;

            rtkLog << fixed << setprecision(4);
            rtkLog << "WEEK=" << Raw.RovEpk.Time.Week
                << " SOW=" << Raw.RovEpk.Time.SecOfWeek
                << " BASELINE_X=" << Raw.DDObs.dPos[0]
                << " BASELINE_Y=" << Raw.DDObs.dPos[1]
                << " BASELINE_Z=" << Raw.DDObs.dPos[2]
                << " RMS_X=" << Raw.DDObs.FloatRMS[0]
                << " RMS_Y=" << Raw.DDObs.FloatRMS[1]
                << " RMS_Z=" << Raw.DDObs.FloatRMS[2]
                << " SIGMA0=" << Raw.DDObs.Sigma0
                << " DD_SATS=" << Raw.DDObs.Sats
                << endl;
        }
        
        if (FBas != nullptr)
        {
            fclose(FBas);
        }

        if (FRov != nullptr)
        {
            fclose(FRov);
        }

        CloseSocket(BasSock);
        CloseSocket(RovSock);

        rtkLog.close();

        cout << "RTK processing finished." << endl;
        cout << "Synchronized epochs: " << totalEpochCount << endl;
        cout << "Successful float solutions: " << floatSuccessCount << endl;
        return 0;
    }

    cout << "Positioning mode: SPP" << endl;
    const double elevationThreshold = config.ElevThreshold * Deg2Rad;

    FILE* fp = nullptr;
    SOCKET NetGPS = INVALID_SOCKET;

    unsigned char Buff[MAXRAWLEN]{};
    int Len = 0;
    int LenRead = 0;
    int val = 0;

    EPOCHOBSDATA Obs;
    GPSEPH GpsEph[MAXGPSNUM]{};
    GPSEPH BdsEph[MAXBDSNUM]{};

    SPPPOS UserPos{};

    int totalEpochCount = 0;
    int successfulSPPCount = 0;
    bool inputError = false;

    ofstream flog(config.ResFile, ios::out);
    if (!flog.is_open())
    {
        cout << "错误：无法创建结果文件：" << config.ResFile << endl;
        return 1;
    }

    if (config.IsFileData == INPUT_MODE_FILE)
    {
        if ((fp = fopen(config.SPPObsDatFile, "rb")) == NULL)
        {
            cout << "错误：无法打开单点定位观测文件："
                << config.SPPObsDatFile << endl;
            flog.close();
            return 1;
        }
    }
    else
    {
        if (OpenSocket(NetGPS, config.RovNetIP, config.RovNetPort) == false)
        {
            cout << "错误：无法打开网络连接" << endl;
            flog.close();
            return 1;
        }
    }

    cout << "开始处理..." << endl;

    do
    {
        if (config.IsFileData == INPUT_MODE_FILE)
        {
            if (Len >= MAXRAWLEN)
            {
                cout << "错误：输入缓冲区已满，无法继续读取文件。" << endl;
                inputError = true;
                break;
            }

            LenRead = fread(Buff + Len, sizeof(unsigned char), MAXRAWLEN - Len, fp);

            if (LenRead == 0)
            {
                if (feof(fp) != 0)
                {
                    break;
                }

                cout << "错误：读取观测文件失败。" << endl;
                inputError = true;
                break;
            }

            Len += LenRead;
        }
        else
        {
            Sleep(980);

            if (Len >= MAXRAWLEN)
            {
                cout << "错误：输入缓冲区已满，无法继续接收网络数据。" << endl;
                inputError = true;
                break;
            }

            LenRead = recv(NetGPS, (char*)Buff + Len, MAXRAWLEN - Len, 0);

            if (LenRead > 0)
            {
                Len += LenRead;
            }
            else if (LenRead == 0)
            {
                break;
            }
            else
            {
                cout << "错误：接收网络数据失败，错误码："
                    << WSAGetLastError() << endl;
                inputError = true;
                break;
            }
        }

        val = DecodeNovOem7Dat(Buff, Len, &Obs, GpsEph, BdsEph);

        if (val == 1)
        {
            totalEpochCount++;

            DetectOutlier(&Obs);

            if (totalEpochCount == 1)
            {
                UserPos.Pos[0] = 1000.0;
                UserPos.Pos[1] = 1000.0;
                UserPos.Pos[2] = 1000.0;
                UserPos.RcvClk[0] = 0.0;
                UserPos.RcvClk[1] = 0.0;
            }

            SatPVTCalculation(GpsEph, BdsEph, &Obs, &UserPos);

            //for (int i = 0; i < Obs.SatNum; i++)
            //{
            //    if (!Obs.SatObs[i].valid || !Obs.SatPVT[i].Valid)
            //    {
            //        continue;
            //    }

            //    char sysChar = 'U';
            //    if (Obs.SatObs[i].System == GPS)
            //    {
            //        sysChar = 'G';
            //    }
            //    else if (Obs.SatObs[i].System == BDS)
            //    {
            //        sysChar = 'C';
            //    }

            //    cout
            //        << sysChar << setw(2) << setfill('0') << Obs.SatObs[i].Prn << setfill(' ')
            //        << " X=" << fixed << setw(13) << setprecision(3) << Obs.SatPVT[i].SatPos[0]
            //        << " Y=" << fixed << setw(13) << setprecision(3) << Obs.SatPVT[i].SatPos[1]
            //        << " Z=" << fixed << setw(13) << setprecision(3) << Obs.SatPVT[i].SatPos[2]
            //        << " Clk=" << scientific << uppercase << setw(15) << setprecision(6) << Obs.SatPVT[i].SatClkOft
            //        << " Vx=" << fixed << nouppercase << setw(13) << setprecision(4) << Obs.SatPVT[i].SatVel[0]
            //        << " Vy=" << fixed << setw(13) << setprecision(4) << Obs.SatPVT[i].SatVel[1]
            //        << " Vz=" << fixed << setw(13) << setprecision(4) << Obs.SatPVT[i].SatVel[2]
            //        << " Clkd=" << scientific << uppercase << setw(14) << setprecision(5) << Obs.SatPVT[i].SatClkSft
            //        << " PIF=" << fixed << nouppercase << setw(15) << setprecision(4) << Obs.CombObs[i].PIF
            //        << " Trop=" << fixed << setw(8) << setprecision(3) << Obs.SatPVT[i].TropCorr
            //        << " E=" << fixed << setw(8) << setprecision(3) << Obs.SatPVT[i].Elevation * Rad2Deg
            //        << "deg"
            //        << endl;

            //    if (config.IsFileData == 1)
            //    {
            //        flog
            //            << sysChar << setw(2) << setfill('0') << Obs.SatObs[i].Prn << setfill(' ')
            //            << " X=" << fixed << setw(13) << setprecision(3) << Obs.SatPVT[i].SatPos[0]
            //            << " Y=" << fixed << setw(13) << setprecision(3) << Obs.SatPVT[i].SatPos[1]
            //            << " Z=" << fixed << setw(13) << setprecision(3) << Obs.SatPVT[i].SatPos[2]
            //            << " Clk=" << scientific << uppercase << setw(15) << setprecision(6) << Obs.SatPVT[i].SatClkOft
            //            << " Vx=" << fixed << nouppercase << setw(13) << setprecision(4) << Obs.SatPVT[i].SatVel[0]
            //            << " Vy=" << fixed << setw(13) << setprecision(4) << Obs.SatPVT[i].SatVel[1]
            //            << " Vz=" << fixed << setw(13) << setprecision(4) << Obs.SatPVT[i].SatVel[2]
            //            << " Clkd=" << scientific << uppercase << setw(14) << setprecision(5) << Obs.SatPVT[i].SatClkSft
            //            << " PIF=" << fixed << nouppercase << setw(15) << setprecision(4) << Obs.CombObs[i].PIF
            //            << " Trop=" << fixed << setw(8) << setprecision(3) << Obs.SatPVT[i].TropCorr
            //            << " E=" << fixed << setw(8) << setprecision(3) << Obs.SatPVT[i].Elevation * Rad2Deg
            //            << "deg"
            //            << endl;
            //    }
            //}
            if (flog.is_open())
            {
                for (int i = 0; i < Obs.SatNum; i++)
                {
                    if (!Obs.SatObs[i].valid || !Obs.SatPVT[i].Valid ||
                        Obs.SatPVT[i].Elevation < elevationThreshold)
                    {
                        continue;
                    }

                    char sysChar = 'U';
                    if (Obs.SatObs[i].System == GPS)
                    {
                        sysChar = 'G';
                    }
                    else if (Obs.SatObs[i].System == BDS)
                    {
                        sysChar = 'C';
                    }

                    flog
                        << sysChar << setw(2) << setfill('0') << Obs.SatObs[i].Prn << setfill(' ')
                        << " X=" << fixed << setw(13) << setprecision(3) << Obs.SatPVT[i].SatPos[0]
                        << " Y=" << fixed << setw(13) << setprecision(3) << Obs.SatPVT[i].SatPos[1]
                        << " Z=" << fixed << setw(13) << setprecision(3) << Obs.SatPVT[i].SatPos[2]
                        << " Clk=" << scientific << uppercase << setw(15) << setprecision(6) << Obs.SatPVT[i].SatClkOft
                        << " Vx=" << fixed << nouppercase << setw(13) << setprecision(4) << Obs.SatPVT[i].SatVel[0]
                        << " Vy=" << fixed << setw(13) << setprecision(4) << Obs.SatPVT[i].SatVel[1]
                        << " Vz=" << fixed << setw(13) << setprecision(4) << Obs.SatPVT[i].SatVel[2]
                        << " Clkd=" << scientific << uppercase << setw(14) << setprecision(5) << Obs.SatPVT[i].SatClkSft
                        << " PIF=" << fixed << nouppercase << setw(15) << setprecision(4) << Obs.CombObs[i].PIF
                        << " Trop=" << fixed << setw(8) << setprecision(3) << Obs.SatPVT[i].TropCorr
                        << " E=" << fixed << setw(8) << setprecision(3) << Obs.SatPVT[i].Elevation * Rad2Deg
                        << "deg"
                        << endl;
                }
            }
 
            UserPos.Time = Obs.Time;

            if (SPP(GpsEph, BdsEph, &UserPos, &Obs, elevationThreshold))
            {
                successfulSPPCount++;

                SPV(&Obs, &UserPos, elevationThreshold);

                double blh[3] = { 0.0, 0.0, 0.0 };
                XYZToBLH(UserPos.Pos, blh, R_WGS84, F_WGS84);

                int gpsSatNum = 0;
                int bdsSatNum = 0;

                for (int i = 0; i < Obs.SatNum; i++)
                {
                    if (!Obs.SatObs[i].valid || !Obs.SatPVT[i].Valid)
                    {
                        continue;
                    }

                    if (Obs.SatObs[i].System == GPS)
                    {
                        gpsSatNum++;
                    }
                    else if (Obs.SatObs[i].System == BDS)
                    {
                        bdsSatNum++;
                    }
                }

                cout
                    << "SPP: "
                    << UserPos.Time.Week << " "
                    << fixed << setw(10) << setprecision(3) << UserPos.Time.SecOfWeek
                    << " X:" << fixed << setprecision(4) << UserPos.Pos[0]
                    << " Y:" << fixed << setprecision(4) << UserPos.Pos[1]
                    << " Z:" << fixed << setprecision(4) << UserPos.Pos[2]

                    << " Vx:" << fixed << setprecision(4) << UserPos.Vel[0]
                    << " Vy:" << fixed << setprecision(4) << UserPos.Vel[1]
                    << " Vz:" << fixed << setprecision(4) << UserPos.Vel[2]

                    << " B:" << fixed << setw(12) << setprecision(8) << blh[0] * Rad2Deg
                    << " L:" << fixed << setw(12) << setprecision(8) << blh[1] * Rad2Deg
                    << " H:" << fixed << setw(8) << setprecision(3) << blh[2]
                    << " GPS Clk:" << fixed << setw(8) << setprecision(3) << UserPos.RcvClk[0]*1E9<<"ns"
                    << " BDS Clk:" << fixed << setw(8) << setprecision(3) << UserPos.RcvClk[1]*1E9<<"ns"
                    << " PDOP:" << fixed << setw(8) << setprecision(3) << UserPos.PDOP
                    << " Sigma:" << fixed << setw(8) << setprecision(3) << UserPos.SigmaPos
                    << " GPSSats:" << setw(3) << gpsSatNum
                    << " BDSSats:" << setw(3) << bdsSatNum
                    << " Sats:" << setw(3) << Obs.SatNum
                    << endl;

                if (flog.is_open())
                {
                    flog
                        << "SPP: "
                        << UserPos.Time.Week << " "
                        << fixed << setw(10) << setprecision(3) << UserPos.Time.SecOfWeek
                        << " X:" << fixed << setprecision(4) << UserPos.Pos[0]
                        << " Y:" << fixed << setprecision(4) << UserPos.Pos[1]
                        << " Z:" << fixed << setprecision(4) << UserPos.Pos[2]
                        << " Vx:" << fixed << setprecision(4) << UserPos.Vel[0]
                        << " Vy:" << fixed << setprecision(4) << UserPos.Vel[1]
                        << " Vz:" << fixed << setprecision(4) << UserPos.Vel[2]
                        << " B:" << fixed << setw(12) << setprecision(8) << blh[0] * Rad2Deg
                        << " L:" << fixed << setw(12) << setprecision(8) << blh[1] * Rad2Deg
                        << " H:" << fixed << setw(8) << setprecision(3) << blh[2]
                        << " GPS Clk:" << fixed << setw(8) << setprecision(3) << UserPos.RcvClk[0] * 1E9 << "ns"
                        << " BDS Clk:" << fixed << setw(8) << setprecision(3) << UserPos.RcvClk[1] * 1E9 << "ns"
                        << " PDOP:" << fixed << setw(8) << setprecision(3) << UserPos.PDOP
                        << " Sigma:" << fixed << setw(8) << setprecision(3) << UserPos.SigmaPos
                        << " GPSSats:" << setw(3) << gpsSatNum
                        << " BDSSats:" << setw(3) << bdsSatNum
                        << " Sats:" << setw(3) << Obs.SatNum
                        << endl
                        << endl;
                }

                cout << endl;
            }
            else
            {
                cout
                    << "SPP: "
                    << Obs.Time.Week << " "
                    << fixed << setprecision(3) << Obs.Time.SecOfWeek
                    << " FAILED"
                    << endl
                    << endl;

                if (flog.is_open())
                {
                    flog
                        << "SPP: "
                        << Obs.Time.Week << " "
                        << fixed << setprecision(3) << Obs.Time.SecOfWeek
                        << " FAILED"
                        << endl
                        << endl;
                }
            }
        }

    } while (1);

    if (config.IsFileData == INPUT_MODE_FILE)
    {
        if (fp)
        {
            fclose(fp);
        }
    }
    else
    {
        CloseSocket(NetGPS);
    }

    flog << "TOTAL_EPOCH=" << totalEpochCount
        << " SUCCESS_SPP=" << successfulSPPCount
        << endl;
    flog.close();

    cout << "处理结束。总历元数: " << totalEpochCount
        << "，SPP成功数: " << successfulSPPCount << endl;

    return inputError ? 1 : 0;
}
