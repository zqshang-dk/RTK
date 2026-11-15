// #include "GPS_Structs.h"

// #include <cmath>
// #include <cstdio>
// #include <cerrno>
// #include <cstring>
// #include <fstream>
// #include <iomanip>
// #include <iostream>

// using namespace std;
#include "GPS_Structs.h"

#include <cerrno>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <iostream>

using namespace std;

// /* 输出RTK结果 */
// void WriteRTKResult(ofstream& log,const RAWDAT& Raw,bool hasSolution,const char reason[],int gpsUsed,int bdsUsed){
//     log << fixed << setprecision(4);

//     log << Raw.RovEpk.Time.Week << " " << Raw.RovEpk.Time.SecOfWeek;

//     if (hasSolution) {
//         log << " " << Raw.DDObs.dPos[0]
//             << " " << Raw.DDObs.dPos[1]
//             << " " << Raw.DDObs.dPos[2]
//             << " " << Raw.DDObs.FloatRMS[0]
//             << " " << Raw.DDObs.FloatRMS[1]
//             << " " << Raw.DDObs.FloatRMS[2]
//             << " " << Raw.DDObs.Sigma0;
//     }
//     else {
//         log << " nan nan nan nan nan nan nan";
//     }

//     if (gpsUsed >= 0 && bdsUsed >= 0){
//         log << " " << Raw.DDObs.Sats;
//     }
//     else{
//         log << " nan";
//     }
//     if (hasSolution) {
//         if (Raw.DDObs.bFixed){
//             log << " FIX";
//         }
//         else{
//             log << " FLOAT";
//         }
//         log << " " << Raw.DDObs.Ratio;
//     }
//     else {
//         log << " INVALID nan";
//     }
//     if (gpsUsed >= 0){
//         log << " " << gpsUsed;
//     }
//     else{
//         log << " nan";
//     }
//     if (bdsUsed >= 0){
//         log << " " << bdsUsed;
//     }
//     else{
//         log << " nan";
//     }
//     log << " " << reason << endl;
// }


// int main(int argc, char* argv[])
// {
//     // 1. 读取配置文件路径。
//     if (argc != 2) {
//         cout << "Usage: " << argv[0] << " config.txt" << endl;
//         return 1;
//     }

//     ROVERCFGINFO config;

//     if (!ReadRTKConfigInfo(argv[1], &config)) {
//         cout << "Error: read config failed." << endl;
//         return 1;
//     }

//     const double elevationThreshold = config.ElevThreshold * Deg2Rad;

//     /*
//         当前配置结构中还没有PositionNoise。
//         后续可以将它加入ROVERCFGINFO和配置文件。
//     */
//     const double positionNoise = 1.0;

//     /*************************************************************************
//         RTK模式
//     *************************************************************************/
//     if (config.PositionMode == POSITION_MODE_RTK){
//         FILE* FBas = nullptr;
//         FILE* FRov = nullptr;

//         SocketHandle BasSock = InvalidSocket;
//         SocketHandle RovSock = InvalidSocket;

//         // 2. 打开RTK输入。
//         if (config.IsFileData == INPUT_MODE_FILE) {
//             FBas = fopen(config.BasObsDatFile, "rb");
//             FRov = fopen(config.RovObsDatFile, "rb");

//             if (FBas == nullptr || FRov == nullptr) {
//                 cout << "Error: open RTK observation file failed." << endl;

//                 if (FBas != nullptr){
//                     fclose(FBas);
//                 }
//                 if (FRov != nullptr){
//                     fclose(FRov);
//                 }
//                 return 1;
//             }
//         }
//         else {
//             if (!OpenSocket(BasSock,
//                             config.BasNetIP,
//                             config.BasNetPort)) {
//                 cout << "Error: connect base failed." << endl;
//                 return 1;
//             }

//             if (!OpenSocket(RovSock,
//                             config.RovNetIP,
//                             config.RovNetPort)) {
//                 cout << "Error: connect rover failed." << endl;
//                 CloseSocket(BasSock);
//                 return 1;
//             }
//         }

//         // 3. 打开RTK结果文件。
//         ofstream log(config.ResFile, ios::out);

//         if (!log.is_open()) {
//             cout << "Error: open result file failed: "
//                 << config.ResFile << endl;

//             if (FBas != nullptr)
//                 fclose(FBas);

//             if (FRov != nullptr)
//                 fclose(FRov);

//             CloseSocket(BasSock);
//             CloseSocket(RovSock);

//             return 1;
//         }

//         log << "WEEK SOW BASELINE_X BASELINE_Y BASELINE_Z "
//             << "RMS_X RMS_Y RMS_Z SIGMA0 DD_SATS STATUS RATIO "
//             << "GPS_USED BDS_USED REASON" << endl;

//         // 4. 初始化RTK数据和滤波器。
//         RAWDAT Raw{};
//         EKFReset(true);

//         int totalEpoch = 0;
//         int successEpoch = 0;
//         int fixedEpoch = 0;

//         while (true)
//         {
//             // 5. 读取并同步基站、流动站历元。
//             int result = GetSynObs(
//                 FBas,
//                 FRov,
//                 BasSock,
//                 RovSock,
//                 &Raw,
//                 config.IsFileData);

//             if (result == -1)
//                 break;

//             if (result == 0) {
//                 WriteRTKResult(
//                     log,
//                     Raw,
//                     false,
//                     "NOT_SYNCHRONIZED",
//                     -1,
//                     -1);

//                 continue;
//             }

//             totalEpoch++;

//             // 6. 观测预处理。
//             DetectOutlier(&Raw.BasEpk);
//             DetectOutlier(&Raw.RovEpk);

//             FormSDEpochObs(
//                 &Raw.BasEpk,
//                 &Raw.RovEpk,
//                 &Raw.SdObs);

//             DetectCycleSlip(
//                 &Raw.BasEpk,
//                 &Raw.RovEpk,
//                 &Raw.SdObs);

//             // 7. 基站和流动站SPP。
//             bool baseOK = SPP(
//                 Raw.GpsEph,
//                 Raw.BdsEph,
//                 &Raw.BaseBest,
//                 &Raw.BasEpk,
//                 elevationThreshold);

//             bool roverOK = SPP(
//                 Raw.GpsEph,
//                 Raw.BdsEph,
//                 &Raw.RoverBest,
//                 &Raw.RovEpk,
//                 elevationThreshold);

//             if (!baseOK || !roverOK) {
//                 const char* reason =
//                     "BASE_AND_ROVER_SPP_FAILED";

//                 if (baseOK)
//                     reason = "ROVER_SPP_FAILED";
//                 else if (roverOK)
//                     reason = "BASE_SPP_FAILED";

//                 WriteRTKResult(
//                     log,
//                     Raw,
//                     false,
//                     reason,
//                     -1,
//                     -1);

//                 EKFReset(false);
//                 continue;
//             }

//             // 8. 选择参考星，构造双差。
//             DetRefSat(
//                 &Raw.BasEpk,
//                 &Raw.RovEpk,
//                 &Raw.SdObs,
//                 &Raw.DDObs);

//             FormDDEpochObs(
//                 &Raw.BasEpk,
//                 &Raw.RovEpk,
//                 &Raw.SdObs,
//                 &Raw.DDObs);

//             int gpsUsed = Raw.DDObs.DDSatNum[GPS];
//             int bdsUsed = Raw.DDObs.DDSatNum[BDS];

//             if (gpsUsed > 0 &&
//                 Raw.DDObs.RefPos[GPS] >= 0)
//                 gpsUsed++;

//             if (bdsUsed > 0 &&
//                 Raw.DDObs.RefPos[BDS] >= 0)
//                 bdsUsed++;

//             if (Raw.DDObs.Sats < 3) {
//                 WriteRTKResult(
//                     log,
//                     Raw,
//                     false,
//                     "TOO_FEW_DD_SATS",
//                     gpsUsed,
//                     bdsUsed);

//                 EKFReset(false);
//                 continue;
//             }

//             // 9. 检查参考星。
//             bool referenceOK = true;

//             for (int sys = 0; sys < 2; sys++) {
//                 if (Raw.DDObs.DDSatNum[sys] > 0 &&
//                     Raw.DDObs.RefPos[sys] < 0) {
//                     referenceOK = false;
//                 }
//             }

//             if (!referenceOK) {
//                 WriteRTKResult(
//                     log,
//                     Raw,
//                     false,
//                     "NO_REFERENCE_SAT",
//                     gpsUsed,
//                     bdsUsed);

//                 EKFReset(false);
//                 continue;
//             }

//             // 10. 浮点解。
//             bool floatOK = false;

//             if (config.RTKProcMode == 1) {
//                 floatOK = RTKFloatEKF(
//                     &Raw,
//                     &Raw.BasEpk,
//                     &Raw.RovEpk,
//                     config.CodeNoise,
//                     config.CPNoise,
//                     positionNoise);
//             }
//             else if (config.RTKProcMode == 2) {
//                 floatOK = RTKFloat(
//                     &Raw,
//                     &Raw.BasEpk,
//                     &Raw.RovEpk,
//                     config.CodeNoise,
//                     config.CPNoise);
//             }

//             if (!floatOK) {
//                 WriteRTKResult(
//                     log,
//                     Raw,
//                     false,
//                     "FLOAT_FAILED",
//                     gpsUsed,
//                     bdsUsed);

//                 continue;
//             }

//             successEpoch++;

//             // 11. 整数模糊度固定。
//             bool fixedOK = RTKFix(
//                 &Raw,
//                 config.RatioThres);

//             if (fixedOK)
//                 fixedEpoch++;

//             if (fixedOK) {
//                 WriteRTKResult(
//                     log,
//                     Raw,
//                     true,
//                     "OK",
//                     gpsUsed,
//                     bdsUsed);
//             }
//             else {
//                 WriteRTKResult(
//                     log,
//                     Raw,
//                     true,
//                     "FIX_NOT_ACCEPTED",
//                     gpsUsed,
//                     bdsUsed);
//             }

//             cout << "Epoch " << totalEpoch
//                 << " dX=" << Raw.DDObs.dPos[0]
//                 << " dY=" << Raw.DDObs.dPos[1]
//                 << " dZ=" << Raw.DDObs.dPos[2]
//                 << endl;
//         }

//         // 12. 关闭RTK输入。
//         if (FBas != nullptr)
//             fclose(FBas);

//         if (FRov != nullptr)
//             fclose(FRov);

//         CloseSocket(BasSock);
//         CloseSocket(RovSock);

//         log << "# TOTAL_EPOCH="
//             << totalEpoch
//             << " SUCCESS_EPOCH="
//             << successEpoch
//             << " FIXED_EPOCH="
//             << fixedEpoch
//             << endl;

//         log.close();

//         cout << "RTK finished." << endl;
//         cout << "Total epoch: " << totalEpoch << endl;
//         cout << "Float epoch: " << successEpoch << endl;
//         cout << "Fixed epoch: " << fixedEpoch << endl;

//         return 0;
//     }


//     /*************************************************************************
//         SPP模式
//     *************************************************************************/
//     if (config.PositionMode == POSITION_MODE_SPP)
//     {
//         FILE* fp = nullptr;
//         SocketHandle sock = InvalidSocket;

//         // 2. 打开SPP输入。
//         if (config.IsFileData == INPUT_MODE_FILE) {
//             fp = fopen(config.SPPObsDatFile, "rb");

//             if (fp == nullptr) {
//                 cout << "Error: open SPP observation file failed."
//                     << endl;
//                 return 1;
//             }
//         }
//         else {
//             if (!OpenSocket(sock,
//                             config.RovNetIP,
//                             config.RovNetPort)) {
//                 cout << "Error: connect rover failed." << endl;
//                 return 1;
//             }
//         }

//         // 3. 打开SPP结果文件。
//         ofstream log(config.ResFile, ios::out);

//         if (!log.is_open()) {
//             cout << "Error: open result file failed: "
//                 << config.ResFile << endl;

//             if (fp != nullptr)
//                 fclose(fp);

//             CloseSocket(sock);
//             return 1;
//         }

//         log << "WEEK SOW X Y Z PDOP SIGMA GPS_SATS BDS_SATS"
//             << endl;

//         unsigned char buffer[MAXRAWLEN] = {};
//         int length = 0;

//         EPOCHOBSDATA Obs;
//         GPSEPH GpsEph[MAXGPSNUM] = {};
//         GPSEPH BdsEph[MAXBDSNUM] = {};
//         SPPPOS UserPos{};

//         UserPos.Pos[0] = 1000.0;
//         UserPos.Pos[1] = 1000.0;
//         UserPos.Pos[2] = 1000.0;

//         int totalEpoch = 0;
//         int successEpoch = 0;
//         bool inputError = false;

//         while (true)
//         {
//             int readLength = 0;

//             // 4. 读取原始数据。
//             if (config.IsFileData == INPUT_MODE_FILE) {
//                 if (length >= MAXRAWLEN) {
//                     cout << "Error: input buffer is full."
//                         << endl;
//                     inputError = true;
//                     break;
//                 }

//                 readLength = static_cast<int>(
//                     fread(
//                         buffer + length,
//                         1,
//                         MAXRAWLEN - length,
//                         fp));

//                 if (readLength == 0) {
//                     if (ferror(fp) != 0)
//                         inputError = true;

//                     break;
//                 }
//             }
//             else {
//                 if (length >= MAXRAWLEN) {
//                     cout << "Error: input buffer is full."
//                         << endl;
//                     inputError = true;
//                     break;
//                 }

//                 readLength = recv(
//                     sock,
//                     reinterpret_cast<char*>(buffer) + length,
//                     MAXRAWLEN - length,
//                     0);

//                 if (readLength == 0)
//                     break;

//                 if (readLength < 0) {
//                     cout << "Error: receive data failed. "
//                         << errno << " "
//                         << strerror(errno) << endl;

//                     inputError = true;
//                     break;
//                 }
//             }

//             length += readLength;

//             // 5. 解码一个完整历元。
//             int result = DecodeNovOem7Dat(
//                 buffer,
//                 length,
//                 &Obs,
//                 GpsEph,
//                 BdsEph);

//             if (result != 1)
//                 continue;

//             totalEpoch++;

//             // 6. 观测预处理和卫星状态计算。
//             DetectOutlier(&Obs);

//             UserPos.Time = Obs.Time;

//             SatPVTCalculation(
//                 GpsEph,
//                 BdsEph,
//                 &Obs,
//                 &UserPos);

//             // 7. SPP定位。
//             bool sppOK = SPP(
//                 GpsEph,
//                 BdsEph,
//                 &UserPos,
//                 &Obs,
//                 elevationThreshold);

//             if (!sppOK) {
//                 cout << "SPP failed: "
//                     << Obs.Time.Week << " "
//                     << Obs.Time.SecOfWeek
//                     << endl;
//                 continue;
//             }

//             successEpoch++;

//             // 8. SPP速度计算。
//             SPV(
//                 &Obs,
//                 &UserPos,
//                 elevationThreshold);

//             int gpsSatNum = 0;
//             int bdsSatNum = 0;

//             for (int i = 0; i < Obs.SatNum; i++) {
//                 if (!Obs.SatObs[i].valid ||
//                     !Obs.SatPVT[i].Valid)
//                     continue;

//                 if (Obs.SatObs[i].System == GPS)
//                     gpsSatNum++;
//                 else if (Obs.SatObs[i].System == BDS)
//                     bdsSatNum++;
//             }

//             log << fixed << setprecision(4)
//                 << Obs.Time.Week << " "
//                 << Obs.Time.SecOfWeek << " "
//                 << UserPos.Pos[0] << " "
//                 << UserPos.Pos[1] << " "
//                 << UserPos.Pos[2] << " "
//                 << UserPos.PDOP << " "
//                 << UserPos.SigmaPos << " "
//                 << gpsSatNum << " "
//                 << bdsSatNum << endl;

//             cout << "SPP epoch "
//                 << Obs.Time.Week << " "
//                 << Obs.Time.SecOfWeek
//                 << " X=" << UserPos.Pos[0]
//                 << " Y=" << UserPos.Pos[1]
//                 << " Z=" << UserPos.Pos[2]
//                 << endl;
//         }

//         // 9. 关闭SPP输入。
//         if (fp != nullptr)
//             fclose(fp);

//         CloseSocket(sock);

//         log << "# TOTAL_EPOCH="
//             << totalEpoch
//             << " SUCCESS_EPOCH="
//             << successEpoch
//             << endl;

//         log.close();

//         cout << "SPP finished." << endl;
//         cout << "Total epoch: " << totalEpoch << endl;
//         cout << "Success epoch: " << successEpoch << endl;

//         return inputError ? 1 : 0;
//     }


//     cout << "Error: unsupported PositionMode." << endl;
//     return 1;
// }




/* 输出RTK结果 */
void WriteRTKResult(
    ofstream& log,
    const RAWDAT& Raw,
    bool hasSolution,
    const char reason[],
    int gpsUsed,
    int bdsUsed)
{
    log << fixed << setprecision(4);

    log << Raw.RovEpk.Time.Week
        << " " << Raw.RovEpk.Time.SecOfWeek;

    if (hasSolution) {
        log << " " << Raw.DDObs.dPos[0]
            << " " << Raw.DDObs.dPos[1]
            << " " << Raw.DDObs.dPos[2]
            << " " << Raw.DDObs.FloatRMS[0]
            << " " << Raw.DDObs.FloatRMS[1]
            << " " << Raw.DDObs.FloatRMS[2]
            << " " << Raw.DDObs.Sigma0;
    }
    else {
        log << " nan nan nan nan nan nan nan";
    }

    if (gpsUsed >= 0 && bdsUsed >= 0)
        log << " " << Raw.DDObs.Sats;
    else
        log << " nan";

    if (hasSolution) {
        if (Raw.DDObs.bFixed)
            log << " FIX";
        else
            log << " FLOAT";

        log << " " << Raw.DDObs.Ratio;
    }
    else {
        log << " INVALID nan";
    }

    if (gpsUsed >= 0)
        log << " " << gpsUsed;
    else
        log << " nan";

    if (bdsUsed >= 0)
        log << " " << bdsUsed;
    else
        log << " nan";

    log << " " << reason << endl;
}


int main(int argc, char* argv[])
{
    // 1. 读取配置文件
    if (argc != 2) {
        cout << "Usage: " << argv[0] << " config.ini" << endl;
        return 1;
    }

    ROVERCFGINFO config;

    if (!ReadRTKConfigInfo(argv[1], &config)) {
        cout << "Error: read config failed." << endl;
        return 1;
    }

    const double elevationThreshold =
        config.ElevThreshold * Deg2Rad;

    // 当前配置结构中没有PositionNoise，暂时使用固定值。
    const double positionNoise = 1.0;

    switch (config.PositionMode) {
    case POSITION_MODE_RTK:
    {
        FILE* FBas = nullptr;
        FILE* FRov = nullptr;

        SocketHandle BasSock = InvalidSocket;
        SocketHandle RovSock = InvalidSocket;

        // 2. 打开RTK输入
        if (config.IsFileData == INPUT_MODE_FILE) {
            FBas = fopen(config.BasObsDatFile, "rb");
            FRov = fopen(config.RovObsDatFile, "rb");

            if (FBas == nullptr || FRov == nullptr) {
                cout << "Error: open RTK observation file failed."
                    << endl;

                if (FBas != nullptr)
                    fclose(FBas);

                if (FRov != nullptr)
                    fclose(FRov);

                return 1;
            }
        }
        else {
            if (!OpenSocket(
                    BasSock,
                    config.BasNetIP,
                    config.BasNetPort)) {
                cout << "Error: connect base failed." << endl;
                return 1;
            }

            if (!OpenSocket(
                    RovSock,
                    config.RovNetIP,
                    config.RovNetPort)) {
                cout << "Error: connect rover failed." << endl;
                CloseSocket(BasSock);
                return 1;
            }
        }

        // 3. 打开结果文件
        ofstream log(config.ResFile, ios::out);

        if (!log.is_open()) {
            cout << "Error: open result file failed: "
                << config.ResFile << endl;

            if (FBas != nullptr)
                fclose(FBas);

            if (FRov != nullptr)
                fclose(FRov);

            CloseSocket(BasSock);
            CloseSocket(RovSock);
            return 1;
        }

        log << "WEEK SOW BASELINE_X BASELINE_Y BASELINE_Z "
            << "RMS_X RMS_Y RMS_Z SIGMA0 DD_SATS STATUS RATIO "
            << "GPS_USED BDS_USED REASON" << endl;

        // 4. 初始化RTK数据和滤波器
        RAWDAT Raw{};

        if (config.RTKProcMode == 1)
            EKFReset(true);

        int totalEpoch = 0;
        int successEpoch = 0;
        int fixedEpoch = 0;
        bool inputError = false;

        while (true) {
            // 5. 读取并同步基站、流动站历元
            int result = GetSynObs(
                FBas,
                FRov,
                BasSock,
                RovSock,
                &Raw,
                config.IsFileData);

            if (result == -1) {
                // 约定：-1表示正常结束。
                break;
            }

            if (result == -2) {
                // 需要同步修改RTK.cpp，让读取错误返回-2。
                cout << "Error: read or synchronize RTK input failed."
                    << endl;
                inputError = true;
                break;
            }

            if (result == 0) {
                WriteRTKResult(
                    log,
                    Raw,
                    false,
                    "NOT_SYNCHRONIZED",
                    -1,
                    -1);

                continue;
            }

            if (result != 1) {
                cout << "Error: unexpected GetSynObs return value: "
                    << result << endl;
                inputError = true;
                break;
            }

            totalEpoch++;

            // 6. 观测预处理
            DetectOutlier(&Raw.BasEpk);
            DetectOutlier(&Raw.RovEpk);

            FormSDEpochObs(
                &Raw.BasEpk,
                &Raw.RovEpk,
                &Raw.SdObs);

            DetectCycleSlip(
                &Raw.BasEpk,
                &Raw.RovEpk,
                &Raw.SdObs);

            // 7. 基站和流动站SPP
            bool baseOK = SPP(
                Raw.GpsEph,
                Raw.BdsEph,
                &Raw.BaseBest,
                &Raw.BasEpk,
                elevationThreshold);

            bool roverOK = SPP(
                Raw.GpsEph,
                Raw.BdsEph,
                &Raw.RoverBest,
                &Raw.RovEpk,
                elevationThreshold);

            if (!baseOK || !roverOK) {
                const char* reason = "BASE_AND_ROVER_SPP_FAILED";

                if (baseOK)
                    reason = "ROVER_SPP_FAILED";
                else if (roverOK)
                    reason = "BASE_SPP_FAILED";

                WriteRTKResult(
                    log,
                    Raw,
                    false,
                    reason,
                    -1,
                    -1);

                if (config.RTKProcMode == 1)
                    EKFReset(false);

                continue;
            }

            // 8. 选择参考星并构造双差
            DetRefSat(
                &Raw.BasEpk,
                &Raw.RovEpk,
                &Raw.SdObs,
                &Raw.DDObs);

            FormDDEpochObs(
                &Raw.BasEpk,
                &Raw.RovEpk,
                &Raw.SdObs,
                &Raw.DDObs);

            int gpsUsed = Raw.DDObs.DDSatNum[GPS];
            int bdsUsed = Raw.DDObs.DDSatNum[BDS];

            if (gpsUsed > 0 && Raw.DDObs.RefPos[GPS] >= 0)
                gpsUsed++;

            if (bdsUsed > 0 && Raw.DDObs.RefPos[BDS] >= 0)
                bdsUsed++;

            if (Raw.DDObs.Sats < 3) {
                WriteRTKResult(
                    log,
                    Raw,
                    false,
                    "TOO_FEW_DD_SATS",
                    gpsUsed,
                    bdsUsed);

                if (config.RTKProcMode == 1)
                    EKFReset(false);

                continue;
            }

            // 9. 检查各系统参考星
            bool referenceOK = true;

            for (int sys = 0; sys < 2; sys++) {
                if (Raw.DDObs.DDSatNum[sys] > 0 &&
                    Raw.DDObs.RefPos[sys] < 0) {
                    referenceOK = false;
                }
            }

            if (!referenceOK) {
                WriteRTKResult(
                    log,
                    Raw,
                    false,
                    "NO_REFERENCE_SAT",
                    gpsUsed,
                    bdsUsed);

                if (config.RTKProcMode == 1)
                    EKFReset(false);

                continue;
            }

            // 10. 浮点解
            bool floatOK = false;

            if (config.RTKProcMode == 1) {
                floatOK = RTKFloatEKF(
                    &Raw,
                    &Raw.BasEpk,
                    &Raw.RovEpk,
                    config.CodeNoise,
                    config.CPNoise,
                    positionNoise);
            }
            else if (config.RTKProcMode == 2) {
                floatOK = RTKFloat(
                    &Raw,
                    &Raw.BasEpk,
                    &Raw.RovEpk,
                    config.CodeNoise,
                    config.CPNoise);
            }

            if (!floatOK) {
                WriteRTKResult(
                    log,
                    Raw,
                    false,
                    "FLOAT_FAILED",
                    gpsUsed,
                    bdsUsed);

                continue;
            }

            successEpoch++;

            // 11. 整数模糊度固定
            bool fixedOK = RTKFix(
                &Raw,
                config.RatioThres);

            if (fixedOK)
                fixedEpoch++;

            if (fixedOK) {
                WriteRTKResult(
                    log,
                    Raw,
                    true,
                    "OK",
                    gpsUsed,
                    bdsUsed);
            }
            else {
                WriteRTKResult(
                    log,
                    Raw,
                    true,
                    "FIX_NOT_ACCEPTED",
                    gpsUsed,
                    bdsUsed);
            }

            cout << "Epoch " << totalEpoch
                << " dX=" << Raw.DDObs.dPos[0]
                << " dY=" << Raw.DDObs.dPos[1]
                << " dZ=" << Raw.DDObs.dPos[2]
                << endl;
        }

        // 12. 关闭输入并检查结果文件
        if (FBas != nullptr)
            fclose(FBas);

        if (FRov != nullptr)
            fclose(FRov);

        CloseSocket(BasSock);
        CloseSocket(RovSock);

        log << "# TOTAL_EPOCH=" << totalEpoch
            << " SUCCESS_EPOCH=" << successEpoch
            << " FIXED_EPOCH=" << fixedEpoch
            << endl;

        log.close();

        if (!log) {
            cout << "Error: write result file failed: "
                << config.ResFile << endl;
            return 1;
        }

        if (inputError)
            return 1;

        cout << "RTK finished." << endl;
        cout << "Total epoch: " << totalEpoch << endl;
        cout << "Float epoch: " << successEpoch << endl;
        cout << "Fixed epoch: " << fixedEpoch << endl;

        return 0;
    }

    case POSITION_MODE_SPP:
    {
        FILE* fp = nullptr;
        SocketHandle sock = InvalidSocket;

        // 2. 打开SPP输入
        if (config.IsFileData == INPUT_MODE_FILE) {
            fp = fopen(config.SPPObsDatFile, "rb");

            if (fp == nullptr) {
                cout << "Error: open SPP observation file failed: "
                    << config.SPPObsDatFile << endl;
                return 1;
            }
        }
        else {
            if (!OpenSocket(
                    sock,
                    config.RovNetIP,
                    config.RovNetPort)) {
                cout << "Error: connect rover failed." << endl;
                return 1;
            }
        }

        // 3. 打开结果文件
        ofstream log(config.ResFile, ios::out);

        if (!log.is_open()) {
            cout << "Error: open result file failed: "
                << config.ResFile << endl;

            if (fp != nullptr)
                fclose(fp);

            CloseSocket(sock);
            return 1;
        }

        log << "WEEK SOW X Y Z PDOP SIGMA GPS_SATS BDS_SATS"
            << endl;

        unsigned char buffer[MAXRAWLEN] = {};
        int length = 0;

        EPOCHOBSDATA Obs;
        GPSEPH GpsEph[MAXGPSNUM] = {};
        GPSEPH BdsEph[MAXBDSNUM] = {};
        SPPPOS UserPos{};

        UserPos.Pos[0] = 1000.0;
        UserPos.Pos[1] = 1000.0;
        UserPos.Pos[2] = 1000.0;

        int totalEpoch = 0;
        int successEpoch = 0;
        bool inputError = false;

        while (true) {
            // 4. 先处理缓冲区里的数据；没有完整观测历元时再读取新数据
            int result = DecodeNovOem7Dat(
                buffer,
                length,
                &Obs,
                GpsEph,
                BdsEph);

            if (result == 1) {
                totalEpoch++;

                // 5. 观测预处理和卫星状态计算
                DetectOutlier(&Obs);

                UserPos.Time = Obs.Time;

                SatPVTCalculation(
                    GpsEph,
                    BdsEph,
                    &Obs,
                    &UserPos);

                // 6. SPP定位
                bool sppOK = SPP(
                    GpsEph,
                    BdsEph,
                    &UserPos,
                    &Obs,
                    elevationThreshold);

                if (!sppOK) {
                    cout << "SPP failed: "
                        << Obs.Time.Week << " "
                        << Obs.Time.SecOfWeek << endl;

                    continue;
                }

                successEpoch++;

                // 7. SPP测速
                SPV(
                    &Obs,
                    &UserPos,
                    elevationThreshold);

                int gpsSatNum = 0;
                int bdsSatNum = 0;

                for (int i = 0; i < Obs.SatNum; i++) {
                    if (!Obs.SatObs[i].valid ||
                        !Obs.SatPVT[i].Valid) {
                        continue;
                    }

                    if (Obs.SatObs[i].System == GPS)
                        gpsSatNum++;
                    else if (Obs.SatObs[i].System == BDS)
                        bdsSatNum++;
                }

                log << fixed << setprecision(4)
                    << Obs.Time.Week << " "
                    << Obs.Time.SecOfWeek << " "
                    << UserPos.Pos[0] << " "
                    << UserPos.Pos[1] << " "
                    << UserPos.Pos[2] << " "
                    << UserPos.PDOP << " "
                    << UserPos.SigmaPos << " "
                    << gpsSatNum << " "
                    << bdsSatNum << endl;

                cout << "SPP epoch "
                    << Obs.Time.Week << " "
                    << Obs.Time.SecOfWeek
                    << " X=" << UserPos.Pos[0]
                    << " Y=" << UserPos.Pos[1]
                    << " Z=" << UserPos.Pos[2]
                    << endl;

                // 回到循环开头，继续解码缓冲区中可能存在的下一个历元
                continue;
            }

            // 8. 当前缓冲区中没有完整历元，追加读取数据
            if (length >= MAXRAWLEN) {
                cout << "Error: input buffer is full." << endl;
                inputError = true;
                break;
            }

            int readLength = 0;

            if (config.IsFileData == INPUT_MODE_FILE) {
                readLength = static_cast<int>(
                    fread(
                        buffer + length,
                        1,
                        MAXRAWLEN - length,
                        fp));

                if (readLength == 0) {
                    if (ferror(fp) != 0) {
                        cout << "Error: read SPP observation file failed."
                            << endl;
                        inputError = true;
                    }
                    else if (length > 0) {
                        cout << "Error: incomplete OEM7 message at end of file."
                            << endl;
                        inputError = true;
                    }

                    break;
                }
            }
            else {
                readLength = recv(
                    sock,
                    reinterpret_cast<char*>(buffer) + length,
                    MAXRAWLEN - length,
                    0);

                if (readLength == 0) {
                    if (length > 0) {
                        cout << "Error: incomplete OEM7 message at end of stream."
                            << endl;
                        inputError = true;
                    }

                    break;
                }

                if (readLength < 0) {
                    cout << "Error: receive SPP data failed. "
                        << errno << " " << strerror(errno) << endl;
                    inputError = true;
                    break;
                }
            }

            length += readLength;
        }

        // 9. 关闭输入并检查结果文件
        if (fp != nullptr)
            fclose(fp);

        CloseSocket(sock);

        log << "# TOTAL_EPOCH=" << totalEpoch
            << " SUCCESS_EPOCH=" << successEpoch
            << endl;

        log.close();

        if (!log) {
            cout << "Error: write result file failed: "
                << config.ResFile << endl;
            return 1;
        }

        cout << "SPP finished." << endl;
        cout << "Total epoch: " << totalEpoch << endl;
        cout << "Success epoch: " << successEpoch << endl;

        return inputError ? 1 : 0;
    }

    default:
        cout << "Error: unsupported PositionMode: "
            << config.PositionMode << endl;
        return 1;
    }
}