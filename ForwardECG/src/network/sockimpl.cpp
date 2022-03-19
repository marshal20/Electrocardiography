#include "sockimpl.h"

// platform specific code
#ifdef _WIN32

////////////////////
// Windows
////////////////////

#include <exception>

#include <stdio.h>
#define WIN32_LEAN_AND_MEAN
#include <windows.h>

// https://docs.microsoft.com/en-us/windows/desktop/WinSock/complete-client-code
bool initialize_sockets_api()
{
	WSADATA wsaData;
	int iResult = WSAStartup(MAKEWORD(2, 2), &wsaData);
	if (iResult != NO_ERROR) {
		return false;
	}
	return true;
}

int close_impl(int sock)
{
	return closesocket(sock);
}

int getaddrinfo_impl(const char *node, const char *service, const struct addrinfo *hints, struct addrinfo **res)
{
	return getaddrinfo(node, service, hints, res);
}

int socket_impl(int af, int type, int protocol)
{
	return socket(af, type, protocol);
}


#else

////////////////////
// UNIX
////////////////////

bool initialize_sockets_api()
{
	return true;
}

int close_impl_impl(int sock)
{
	return close(sock);
}

int getaddrinfo_impl(const char *node, const char *service, const struct addrinfo *hints, struct addrinfo **res)
{
	return getaddrinfo(node, service, hints, res);
}

int socket_impl(int af, int type, int protocol)
{
	return socket(af, type, protocol);
}

#endif


