#pragma once

// platform specific include files
#ifdef _WIN32

// Windows

#include <sys/types.h>
#include <WinSock2.h>
//#include <ws2def.h>
#include <ws2tcpip.h>

#define SHUT_RD SD_RECEIVE
#define SHUT_WR SD_SEND
#define SHUT_RDWR SD_BOTH

typedef int socklen_t;

#else

// UNIX

#include <sys/types.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <unistd.h>

#endif


bool initialize_sockets_api();

int close_impl(int sock);

int getaddrinfo_impl(const char *node, const char *service, const struct addrinfo *hints, struct addrinfo **res);
int socket_impl(int af, int type, int protocol);
