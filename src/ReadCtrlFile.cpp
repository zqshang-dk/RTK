#include "GPS_Structs.h"
#include <fstream>
#include <string>
#include <cstring>

using namespace std;

// Remove leading and trailing spaces, tabs, and line breaks.
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

// Copy a string into a fixed-size character array.
bool CopyConfigString(char target[], size_t targetSize, const string& value)
{
	if (value.size() >= targetSize)
	{
		return false;
	}

	strcpy(target, value.c_str());
	return true;
}

// Read RTKConfig.ini. Each setting uses the key=value format.
bool ReadRTKConfigInfo(const char configFile[], ROVERCFGINFO* config)
{
	if (configFile == nullptr || config == nullptr)
	{
		cout << "Error: configFile or config is null." << endl;
		return false;
	}

	ifstream inputFile(configFile);
	if (!inputFile.is_open())
	{
		cout << "Error: Cannot open configuration file: " << configFile << endl;
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
			cout << "Warning: Configuration line " << lineNumber
				<< " has no '=' sign and was skipped." << endl;
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
					cout << "Error: BasNetIP is too long." << endl;
					return false;
				}
			}
			else if (key == "RovNetIP")
			{
				if (!CopyConfigString(config->RovNetIP, sizeof(config->RovNetIP), value))
				{
					cout << "Error: RovNetIP is too long." << endl;
					return false;
				}
			}
			else if (key == "BasNetPort" || key == "RovNetPort")
			{
				int port = stoi(value);
				if (port < 0 || port > 65535)
				{
					cout << "Error: Port on configuration line " << lineNumber << " must be between 0 and 65535." << endl;
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
					cout << "Error: SPPObsDatFile path is too long." << endl;
					return false;
				}
			}
			else if (key == "BasObsDatFile")
			{
				if (!CopyConfigString(config->BasObsDatFile, sizeof(config->BasObsDatFile), value))
				{
					cout << "Error: BasObsDatFile path is too long." << endl;
					return false;
				}
			}
			else if (key == "RovObsDatFile")
			{
				if (!CopyConfigString(config->RovObsDatFile, sizeof(config->RovObsDatFile), value))
				{
					cout << "Error: RovObsDatFile path is too long." << endl;
					return false;
				}
			}
			else if (key == "ResFile")
			{
				if (!CopyConfigString(config->ResFile, sizeof(config->ResFile), value))
				{
					cout << "Error: ResFile path is too long." << endl;
					return false;
				}
			}
			else
			{
				cout << "Warning: Unknown configuration key: " << key << "." << endl;
			}
		}
		catch (...)
		{
			cout << "Error: Invalid parameter value on configuration line " << lineNumber
				<< "." << endl;
			return false;
		}
	}

	inputFile.close();

	if (config->PositionMode != POSITION_MODE_SPP &&
		config->PositionMode != POSITION_MODE_RTK)
	{
		cout << "Error: PositionMode must be 1 or 2." << endl;
		return false;
	}

	if (config->IsFileData != INPUT_MODE_NETWORK &&
		config->IsFileData != INPUT_MODE_FILE)
	{
		cout << "Error: IsFileData must be 0 or 1." << endl;
		return false;
	}

	if (config->RTKProcMode != 1 && config->RTKProcMode != 2)
	{
		cout << "Error: RTKProcMode must be 1 or 2." << endl;
		return false;
	}

	if (config->CodeNoise <= 0.0 || config->CPNoise <= 0.0)
	{
		cout << "Error: CodeNoise and CPNoise must be greater than 0." << endl;
		return false;
	}

	if (config->ElevThreshold < 0.0 || config->ElevThreshold >= 90.0)
	{
		cout << "Error: ElevThreshold must satisfy 0 <= value < 90 degrees." << endl;
		return false;
	}

	if (config->RatioThres <= 0.0)
	{
		cout << "Error: RatioThres must be greater than 0." << endl;
		return false;
	}

	if (config->ResFile[0] == '\0')
	{
		cout << "Error: ResFile is required." << endl;
		return false;
	}

	if (config->IsFileData == INPUT_MODE_FILE)
	{
		if (config->PositionMode == POSITION_MODE_SPP &&
			config->SPPObsDatFile[0] == '\0')
		{
			cout << "Error: SPPObsDatFile is required in SPP file mode." << endl;
			return false;
		}

		if (config->PositionMode == POSITION_MODE_RTK &&
			(config->BasObsDatFile[0] == '\0' || config->RovObsDatFile[0] == '\0'))
		{
			cout << "Error: BasObsDatFile and RovObsDatFile are required in RTK file mode." << endl;
			return false;
		}
	}
	else
	{
		if (config->RovNetIP[0] == '\0' || config->RovNetPort == 0)
		{
			cout << "Error: RovNetIP and a nonzero RovNetPort are required in network mode." << endl;
			return false;
		}

		if (config->PositionMode == POSITION_MODE_RTK &&
			(config->BasNetIP[0] == '\0' || config->BasNetPort == 0))
		{
			cout << "Error: BasNetIP and a nonzero BasNetPort are required in RTK network mode." << endl;
			return false;
		}
	}

	return true;
}
