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
        cout << "错误：无法读取RTK配置文件" << endl;
        return 1;
    }

    if (config.PositionMode == POSITION_MODE_RTK)
    {
        cout << "当前定位模式：RTK" << endl;
        if (config.IsFileData == INPUT_MODE_FILE)
        {
            cout << "基站文件：" << config.BasObsDatFile << endl;
            cout << "流动站文件：" << config.RovObsDatFile << endl;
        }
        else
        {
            cout << "基站网络：" << config.BasNetIP << ":" << config.BasNetPort << endl;
            cout << "流动站网络：" << config.RovNetIP << ":" << config.RovNetPort << endl;
        }

        cout << "RTK解算流程尚未实现，请先将PositionMode设置为1运行单点定位。" << endl;
        return 1;
    }

    cout << "当前定位模式：单点定位SPP" << endl;
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
