#include "GPS_Structs.h"
#include <arpa/inet.h>
#include <cerrno>
#include <cstring>
#include <unistd.h>

bool OpenSocket(SocketHandle& sock, const char IP[], const unsigned short Port)
{
	sock = InvalidSocket;
	sockaddr_in addrSrv{};
	addrSrv.sin_family = AF_INET;
	addrSrv.sin_port = htons(Port);

	if (inet_pton(AF_INET, IP, &addrSrv.sin_addr) != 1)
	{
		return false;
	}

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock == InvalidSocket)
	{
		return false;
	}

	if (connect(sock, reinterpret_cast<sockaddr*>(&addrSrv), sizeof(addrSrv)) == -1)
	{
		int connectError = errno;
		CloseSocket(sock);
		errno = connectError;
		return false;
	}
	return true;
}

void CloseSocket(SocketHandle& sock)
{
    if (sock != InvalidSocket)
    {
        close(sock);
        sock = InvalidSocket;
    }
}
