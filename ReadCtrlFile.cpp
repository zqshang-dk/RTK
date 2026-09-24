#include "GPS_Structs.h"
#include <fstream>
#include <string>
#include <cstring>

using namespace std;

// 删除字符串首尾的空格、制表符和换行符
string TrimString(const string& text)
{
	size_t first = text.find_first_not_of(" \t\r\n");
	if (first == string::npos)
	{
		return "";
	}

	size_t last = text.find_last_not_of(" \t\r\n");
	return text.substr(first, last - first + 1);
}

// 将string复制到固定长度的char数组中
bool CopyConfigString(char target[], size_t targetSize, const string& value)
{
	if (value.size() >= targetSize)
	{
		return false;
	}

	strcpy(target, value.c_str());
	return true;
}

// 读取RTKConfig.ini，每一行的格式为“参数名=参数值”
bool ReadRTKConfigInfo(const char configFile[], ROVERCFGINFO* config)
{
	if (configFile == nullptr || config == nullptr)
	{
		cout << "错误：配置文件名或配置结构体为空。" << endl;
		return false;
	}

	ifstream inputFile(configFile);
	if (!inputFile.is_open())
	{
		cout << "错误：无法打开配置文件：" << configFile << endl;
		return false;
	}

	config->BasNetIP[0] = '\0';
	config->RovNetIP[0] = '\0';
	config->SPPObsDatFile[0] = '\0';
	config->BasObsDatFile[0] = '\0';
	config->RovObsDatFile[0] = '\0';
	strcpy(config->ResFile, "position_result.txt");

	config->CodeNoise = 1.0;
	config->CPNoise = 0.01;
	config->ElevThreshold = 15.0;
	config->RatioThres = 3.0;

	string line;
	int lineNumber = 0;

	while (getline(inputFile, line))
	{
		lineNumber++;
		line = TrimString(line);

		if (line.empty() || line[0] == '#' || line[0] == ';')
		{
			continue;
		}

		size_t equalPosition = line.find('=');
		if (equalPosition == string::npos)
		{
			cout << "警告：配置文件第 " << lineNumber
				<< " 行缺少等号，已经跳过。" << endl;
			continue;
		}

		string key = TrimString(line.substr(0, equalPosition));
		string value = TrimString(line.substr(equalPosition + 1));

		if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
		{
			value = value.substr(1, value.size() - 2);
		}

		try
		{
			if (key == "PositionMode")
			{
				config->PositionMode = static_cast<short>(stoi(value));
			}
			else if (key == "IsFileData")
			{
				config->IsFileData = static_cast<short>(stoi(value));
			}
			else if (key == "RTKProcMode")
			{
				config->RTKProcMode = static_cast<short>(stoi(value));
			}
			else if (key == "RovPort")
			{
				config->RovPort = stoi(value);
			}
			else if (key == "RovBaud")
			{
				config->RovBaud = stoi(value);
			}
			else if (key == "BasNetIP")
			{
				if (!CopyConfigString(config->BasNetIP, sizeof(config->BasNetIP), value))
				{
					cout << "错误：BasNetIP过长。" << endl;
					return false;
				}
			}
			else if (key == "RovNetIP")
			{
				if (!CopyConfigString(config->RovNetIP, sizeof(config->RovNetIP), value))
				{
					cout << "错误：RovNetIP过长。" << endl;
					return false;
				}
			}
			else if (key == "BasNetPort" || key == "RovNetPort")
			{
				int port = stoi(value);
				if (port < 0 || port > 65535)
				{
					cout << "错误：配置文件第 " << lineNumber
						<< " 行的端口必须在0到65535之间。" << endl;
					return false;
				}

				if (key == "BasNetPort")
				{
					config->BasNetPort = static_cast<unsigned short>(port);
				}
				else
				{
					config->RovNetPort = static_cast<unsigned short>(port);
				}
			}
			else if (key == "CodeNoise")
			{
				config->CodeNoise = stod(value);
			}
			else if (key == "CPNoise")
			{
				config->CPNoise = stod(value);
			}
			else if (key == "ElevThreshold")
			{
				config->ElevThreshold = stod(value);
			}
			else if (key == "RatioThres")
			{
				config->RatioThres = stod(value);
			}
			else if (key == "SPPObsDatFile")
			{
				if (!CopyConfigString(config->SPPObsDatFile, sizeof(config->SPPObsDatFile), value))
				{
					cout << "错误：SPPObsDatFile路径过长。" << endl;
					return false;
				}
			}
			else if (key == "BasObsDatFile")
			{
				if (!CopyConfigString(config->BasObsDatFile, sizeof(config->BasObsDatFile), value))
				{
					cout << "错误：BasObsDatFile路径过长。" << endl;
					return false;
				}
			}
			else if (key == "RovObsDatFile")
			{
				if (!CopyConfigString(config->RovObsDatFile, sizeof(config->RovObsDatFile), value))
				{
					cout << "错误：RovObsDatFile路径过长。" << endl;
					return false;
				}
			}
			else if (key == "ResFile")
			{
				if (!CopyConfigString(config->ResFile, sizeof(config->ResFile), value))
				{
					cout << "错误：ResFile路径过长。" << endl;
					return false;
				}
			}
			else
			{
				cout << "警告：无法识别配置项 " << key << "。" << endl;
			}
		}
		catch (...)
		{
			cout << "错误：配置文件第 " << lineNumber
				<< " 行的参数值格式不正确。" << endl;
			return false;
		}
	}

	inputFile.close();

	if (config->PositionMode != POSITION_MODE_SPP &&
		config->PositionMode != POSITION_MODE_RTK)
	{
		cout << "错误：PositionMode只能填写1或2。" << endl;
		return false;
	}

	if (config->IsFileData != INPUT_MODE_NETWORK &&
		config->IsFileData != INPUT_MODE_FILE)
	{
		cout << "错误：IsFileData只能填写0或1。" << endl;
		return false;
	}

	if (config->RTKProcMode != 1 && config->RTKProcMode != 2)
	{
		cout << "错误：RTKProcMode只能填写1或2。" << endl;
		return false;
	}

	if (config->CodeNoise <= 0.0 || config->CPNoise <= 0.0)
	{
		cout << "错误：CodeNoise和CPNoise必须大于0。" << endl;
		return false;
	}

	if (config->ElevThreshold < 0.0 || config->ElevThreshold >= 90.0)
	{
		cout << "错误：ElevThreshold必须在0到90度之间。" << endl;
		return false;
	}

	if (config->RatioThres <= 0.0)
	{
		cout << "错误：RatioThres必须大于0。" << endl;
		return false;
	}

	if (config->ResFile[0] == '\0')
	{
		cout << "错误：必须填写ResFile。" << endl;
		return false;
	}

	if (config->IsFileData == INPUT_MODE_FILE)
	{
		if (config->PositionMode == POSITION_MODE_SPP &&
			config->SPPObsDatFile[0] == '\0')
		{
			cout << "错误：单点定位文件模式下必须填写SPPObsDatFile。" << endl;
			return false;
		}

		if (config->PositionMode == POSITION_MODE_RTK &&
			(config->BasObsDatFile[0] == '\0' || config->RovObsDatFile[0] == '\0'))
		{
			cout << "错误：RTK文件模式下必须填写基站和流动站文件。" << endl;
			return false;
		}
	}
	else
	{
		if (config->RovNetIP[0] == '\0' || config->RovNetPort == 0)
		{
			cout << "错误：网络模式下必须填写流动站IP和端口。" << endl;
			return false;
		}

		if (config->PositionMode == POSITION_MODE_RTK &&
			(config->BasNetIP[0] == '\0' || config->BasNetPort == 0))
		{
			cout << "错误：RTK网络模式下必须填写基站IP和端口。" << endl;
			return false;
		}
	}

	return true;
}
