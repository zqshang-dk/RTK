#include "GPS_Structs.h"
#include <cstdio>
#include <cstring>
#include <cctype>

// #include "GPS_Structs.h"
// #include <fstream>
// #include <string>
// #include <cstring>

// using namespace std;

// // Remove leading and trailing spaces, tabs, and line breaks.
// string TrimString(const string& text){
// 	size_t first = text.find_first_not_of(" \t\r\n");
// 	if (first == string::npos){
// 		return "";
// 	}

// 	size_t last = text.find_last_not_of(" \t\r\n");
// 	return text.substr(first, last - first + 1);
// }

// // Copy a string into a fixed-size character array.
// bool CopyConfigString(char target[], size_t targetSize, const string& value){
// 	if (value.size() >= targetSize){
// 		return false;
// 	}

// 	strcpy(target, value.c_str());
// 	return true;
// }

// // Read the configuration file supplied by the user. Each setting uses key=value.
// bool ReadRTKConfigInfo(const char configFile[], ROVERCFGINFO* config){
// 	if (configFile == nullptr || config == nullptr){
// 		cout << "Error: configFile or config is null." << endl;
// 		return false;
// 	}

// 	ifstream inputFile(configFile);
// 	if (!inputFile.is_open()){
// 		cout << "Error: Cannot open configuration file: " << configFile << endl;
// 		return false;
// 	}

// 	config->BasNetIP[0] = '\0';
// 	config->RovNetIP[0] = '\0';
// 	config->SPPObsDatFile[0] = '\0';
// 	config->BasObsDatFile[0] = '\0';
// 	config->RovObsDatFile[0] = '\0';
// 	strcpy(config->ResFile, "position_result.txt");

// 	config->CodeNoise = 1.0;
// 	config->CarrierNoise = 0.01;
// 	config->ElevThreshold = 15.0;
// 	config->RatioThres = 3.0;

// 	string line;
// 	int lineNumber = 0;

// 	while (getline(inputFile, line)){
// 		lineNumber++;
// 		line = TrimString(line);

// 		if (line.empty() || line[0] == '#' || line[0] == ';'){
// 			continue;
// 		}

// 		size_t equalPosition = line.find('=');
// 		if (equalPosition == string::npos){
// 			cout << "Warning: Configuration line " << lineNumber
// 				<< " has no '=' sign and was skipped." << endl;
// 			continue;
// 		}

// 		string key = TrimString(line.substr(0, equalPosition));
// 		string value = TrimString(line.substr(equalPosition + 1));

// 		if (value.size() >= 2 && value.front() == '"' && value.back() == '"'){
// 			value = value.substr(1, value.size() - 2);
// 		}

// 		try{
// 			if (key == "PositionMode"){
// 				config->PositionMode = static_cast<short>(stoi(value));
// 			}
// 			else if (key == "IsFileData"){
// 				config->IsFileData = static_cast<short>(stoi(value));
// 			}
// 			else if (key == "RTKProcMode"){
// 				config->RTKProcMode = static_cast<short>(stoi(value));
// 			}
// 			else if (key == "RovPort"){
// 				config->RovPort = stoi(value);
// 			}
// 			else if (key == "RovBaud"){
// 				config->RovBaud = stoi(value);
// 			}
// 			else if (key == "BasNetIP"){
// 				if (!CopyConfigString(config->BasNetIP, sizeof(config->BasNetIP), value)){
// 					cout << "Error: BasNetIP is too long." << endl;
// 					return false;
// 				}
// 			}
// 			else if (key == "RovNetIP"){
// 				if (!CopyConfigString(config->RovNetIP, sizeof(config->RovNetIP), value)){
// 					cout << "Error: RovNetIP is too long." << endl;
// 					return false;
// 				}
// 			}
// 			else if (key == "BasNetPort" || key == "RovNetPort"){
// 				int port = stoi(value);
// 				if (port < 0 || port > 65535){
// 					cout << "Error: Port on configuration line " << lineNumber << " must be between 0 and 65535." << endl;
// 					return false;
// 				}
// 				if (key == "BasNetPort"){
// 					config->BasNetPort = static_cast<unsigned short>(port);
// 				}
// 				else{
// 					config->RovNetPort = static_cast<unsigned short>(port);
// 				}
// 			}
// 			else if (key == "CodeNoise"){
// 				config->CodeNoise = stod(value);
// 			}
// 			else if (key == "CarrierNoise"){
// 				config->CarrierNoise = stod(value);
// 			}
// 			else if (key == "ElevThreshold"){
// 				config->ElevThreshold = stod(value);
// 			}
// 			else if (key == "RatioThres"){
// 				config->RatioThres = stod(value);
// 			}
// 			else if (key == "SPPObsDatFile"){
// 				if (!CopyConfigString(config->SPPObsDatFile, sizeof(config->SPPObsDatFile), value)){
// 					cout << "Error: SPPObsDatFile path is too long." << endl;
// 					return false;
// 				}
// 			}
// 			else if (key == "BasObsDatFile"){
// 				if (!CopyConfigString(config->BasObsDatFile, sizeof(config->BasObsDatFile), value)){
// 					cout << "Error: BasObsDatFile path is too long." << endl;
// 					return false;
// 				}
// 			}
// 			else if (key == "RovObsDatFile"){
// 				if (!CopyConfigString(config->RovObsDatFile, sizeof(config->RovObsDatFile), value)){
// 					cout << "Error: RovObsDatFile path is too long." << endl;
// 					return false;
// 				}
// 			}
// 			else if (key == "ResFile"){
// 				if (!CopyConfigString(config->ResFile, sizeof(config->ResFile), value)){
// 					cout << "Error: ResFile path is too long." << endl;
// 					return false;
// 				}
// 			}
// 			else{
// 				cout << "Warning: Unknown configuration key: " << key << "." << endl;
// 			}
// 		}
// 		catch (...){
// 			cout << "Error: Invalid parameter value on configuration line " << lineNumber << "." << endl;
// 			return false;
// 		}
// 	}

// 	inputFile.close();

// 	if (config->PositionMode != POSITION_MODE_SPP &&
// 		config->PositionMode != POSITION_MODE_RTK){
// 		cout << "Error: PositionMode must be 1 or 2." << endl;
// 		return false;
// 	}

// 	if (config->IsFileData != INPUT_MODE_NETWORK &&
// 		config->IsFileData != INPUT_MODE_FILE){
// 		cout << "Error: IsFileData must be 0 or 1." << endl;
// 		return false;
// 	}

// 	if (config->RTKProcMode != 1 && config->RTKProcMode != 2){
// 		cout << "Error: RTKProcMode must be 1 or 2." << endl;
// 		return false;
// 	}

// 	if (config->CodeNoise <= 0.0 || config->CarrierNoise <= 0.0){
// 		cout << "Error: CodeNoise and CarrierNoise must be greater than 0." << endl;
// 		return false;
// 	}

// 	if (config->ElevThreshold < 0.0 || config->ElevThreshold >= 90.0){
// 		cout << "Error: ElevThreshold must satisfy 0 <= value < 90 degrees." << endl;
// 		return false;
// 	}

// 	if (config->RatioThres <= 0.0){
// 		cout << "Error: RatioThres must be greater than 0." << endl;
// 		return false;
// 	}

// 	if (config->ResFile[0] == '\0'){
// 		cout << "Error: ResFile is required." << endl;
// 		return false;
// 	}

// 	if (config->IsFileData == INPUT_MODE_FILE){
// 		if (config->PositionMode == POSITION_MODE_SPP &&
// 			config->SPPObsDatFile[0] == '\0'){
// 			cout << "Error: SPPObsDatFile is required in SPP file mode." << endl;
// 			return false;
// 		}

// 		if (config->PositionMode == POSITION_MODE_RTK &&
// 			(config->BasObsDatFile[0] == '\0' || config->RovObsDatFile[0] == '\0')){
// 			cout << "Error: BasObsDatFile and RovObsDatFile are required in RTK file mode." << endl;
// 			return false;
// 		}
// 	}
// 	else{
// 		if (config->RovNetIP[0] == '\0' || config->RovNetPort == 0){
// 			cout << "Error: RovNetIP and a nonzero RovNetPort are required in network mode." << endl;
// 			return false;
// 		}

// 		if (config->PositionMode == POSITION_MODE_RTK &&
// 			(config->BasNetIP[0] == '\0' || config->BasNetPort == 0)){
// 			cout << "Error: BasNetIP and a nonzero BasNetPort are required in RTK network mode." << endl;
// 			return false;
// 		}
// 	}


// 	// 配置中的相对路径以ini文件所在目录为起点，而不是程序启动目录。
// 	string configPath = configFile;
// #ifdef _WIN32
// 	size_t slash = configPath.find_last_of("/\\");
// #else
// 	size_t slash = configPath.find_last_of('/');
// #endif
// 	string configDirectory;
// 	if (slash != string::npos){
// 		configDirectory = configPath.substr(0, slash + 1);
// 	}

// 	char* paths[] = { config->SPPObsDatFile, config->BasObsDatFile,
// 	                  config->RovObsDatFile, config->ResFile };
// 	size_t sizes[] = { sizeof(config->SPPObsDatFile), sizeof(config->BasObsDatFile),
// 	                   sizeof(config->RovObsDatFile), sizeof(config->ResFile) };
// 	for (int i = 0; i < 4; i++){
// 		string path = paths[i];
// 		if (path.empty()){
// 			continue;
// 		}
// 		bool absolute = path[0] == '/';
// #ifdef _WIN32
// 		absolute = absolute || path[0] == '\\' || (path.size() > 1 && path[1] == ':');
// #endif
// 		if (!absolute && !CopyConfigString(paths[i], sizes[i], configDirectory + path)){
// 			cout << "Error: Path relative to configuration file is too long: " << path << endl;
// 			return false;
// 		}
// 	}

// 	return true;
// }




using namespace std;

#define CFGITEMS 17

static const char CfgStr[CFGITEMS][32] =
{
    "PositionMode",
    "IsFileData",
    "RTKProcMode",
    "RovPort",
    "RovBaud",
    "BasNetIP",
    "RovNetIP",
    "BasNetPort",
    "RovNetPort",
    "CodeNoise",
    "CarrierNoise",
    "ElevThreshold",
    "RatioThres",
    "SPPObsDatFile",
    "BasObsDatFile",
    "RovObsDatFile",
    "ResFile"
};

static void TrimText(char text[])
{
    char* first = text;
    while (*first != '\0' && isspace(static_cast<unsigned char>(*first))) {
        first++;
    }
    if (first != text) {
        memmove(text, first, strlen(first) + 1);
    }
    int len = static_cast<int>(strlen(text));
    while (len > 0 && isspace(static_cast<unsigned char>(text[len - 1]))) {
        text[len - 1] = '\0';
        len--;
    }
}

static bool CopyConfigText(char target[], size_t targetSize, const char source[]){
    size_t length = strlen(source);
    if (length >= targetSize) {
        return false;
    }
    strcpy(target, source);
    return true;
}

bool ReadRTKConfigInfo(const char configFile[], ROVERCFGINFO* config){
    if (configFile == nullptr || config == nullptr) {
        printf("ERROR: invalid configuration input.\n");
        return false;
    }
    FILE* Fin = fopen(configFile, "r");
    if (Fin == nullptr) {
        printf("ERROR: cannot open configuration file: %s\n", configFile);
        return false;
    }

    // 使用结构体构造函数初始化默认值 
    *config = ROVERCFGINFO();
    strcpy(config->ResFile, "position_result.txt");
    char Line[512];
    char Key[32];
    char Value[256];
    int lineNumber = 0;
    while (fgets(Line, sizeof(Line), Fin) != nullptr) {
        lineNumber++;
        // 删除行尾注释
        char* comment = strchr(Line, '#');
        if (comment != nullptr) {
            *comment = '\0';
        }
        TrimText(Line);

        // 跳过空行和分号注释
        if (Line[0] == '\0' || Line[0] == ';') {
            continue;
        }

        Key[0] = '\0';
        Value[0] = '\0';

        // 读取：参数名 参数值
        if (sscanf(Line, "%31s %255[^\n]", Key, Value) != 2) {
            printf("WARNING: invalid line %d.\n", lineNumber);
            continue;
        }

        TrimText(Value);

        // 去掉参数值两端的双引号
        size_t valueLength = strlen(Value);
        if (valueLength >= 2 &&
            Value[0] == '"' &&
            Value[valueLength - 1] == '"') {
            Value[valueLength - 1] = '\0';
            memmove(Value, Value + 1, strlen(Value));
        }

        int item = -1;

        for (int i = 0; i < CFGITEMS; i++) {
            if (strcmp(Key, CfgStr[i]) == 0) {
                item = i;
                break;
            }
        }

        if (item < 0) {
            printf("WARNING: unknown parameter on line %d: %s\n",
                   lineNumber, Key);
            continue;
        }

        bool ok = true;

        switch (item) {
        case 0:
            ok = sscanf(Value, "%hd", &config->PositionMode) == 1;
            break;

        case 1:
            ok = sscanf(Value, "%hd", &config->IsFileData) == 1;
            break;

        case 2:
            ok = sscanf(Value, "%hd", &config->RTKProcMode) == 1;
            break;

        case 3:
            ok = sscanf(Value, "%d", &config->RovPort) == 1;
            break;

        case 4:
            ok = sscanf(Value, "%d", &config->RovBaud) == 1;
            break;

        case 5:
            ok = CopyConfigText(config->BasNetIP,
                                sizeof(config->BasNetIP),
                                Value);
            break;

        case 6:
            ok = CopyConfigText(config->RovNetIP,
                                sizeof(config->RovNetIP),
                                Value);
            break;

        case 7:
        {
            int port = 0;
            ok = sscanf(Value, "%d", &port) == 1;

            if (ok && port >= 0 && port <= 65535) {
                config->BasNetPort =
                    static_cast<unsigned short>(port);
            }
            else {
                ok = false;
            }
            break;
        }

        case 8:
        {
            int port = 0;
            ok = sscanf(Value, "%d", &port) == 1;

            if (ok && port >= 0 && port <= 65535) {
                config->RovNetPort =
                    static_cast<unsigned short>(port);
            }
            else {
                ok = false;
            }
            break;
        }

        case 9:
            ok = sscanf(Value, "%lf", &config->CodeNoise) == 1;
            break;

        case 10:
            ok = sscanf(Value, "%lf", &config->CarrierNoise) == 1;
            break;

        case 11:
            ok = sscanf(Value, "%lf", &config->ElevThreshold) == 1;
            break;

        case 12:
            ok = sscanf(Value, "%lf", &config->RatioThres) == 1;
            break;

        case 13:
            ok = CopyConfigText(config->SPPObsDatFile,
                                sizeof(config->SPPObsDatFile),
                                Value);
            break;

        case 14:
            ok = CopyConfigText(config->BasObsDatFile,
                                sizeof(config->BasObsDatFile),
                                Value);
            break;

        case 15:
            ok = CopyConfigText(config->RovObsDatFile,
                                sizeof(config->RovObsDatFile),
                                Value);
            break;

        case 16:
            ok = CopyConfigText(config->ResFile,
                                sizeof(config->ResFile),
                                Value);
            break;

        default:
            ok = false;
            break;
        }

        if (!ok) {
            printf("ERROR: invalid value on line %d: %s\n",
                   lineNumber, Line);
            fclose(Fin);
            return false;
        }
    }

    fclose(Fin);

    // 基本参数检查
    if (config->PositionMode != POSITION_MODE_SPP &&
        config->PositionMode != POSITION_MODE_RTK) {
        printf("ERROR: PositionMode must be 1 or 2.\n");
        return false;
    }

    if (config->IsFileData != INPUT_MODE_NETWORK &&
        config->IsFileData != INPUT_MODE_FILE) {
        printf("ERROR: IsFileData must be 0 or 1.\n");
        return false;
    }

    if (config->RTKProcMode != 1 &&
        config->RTKProcMode != 2) {
        printf("ERROR: RTKProcMode must be 1 or 2.\n");
        return false;
    }

    if (config->CodeNoise <= 0.0 ||
        config->CarrierNoise <= 0.0) {
        printf("ERROR: noise must be greater than zero.\n");
        return false;
    }

    if (config->ElevThreshold < 0.0 ||
        config->ElevThreshold >= 90.0) {
        printf("ERROR: invalid elevation threshold.\n");
        return false;
    }

    if (config->RatioThres <= 0.0) {
        printf("ERROR: RatioThres must be greater than zero.\n");
        return false;
    }

    return true;
}