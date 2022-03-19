#pragma once
#include "socket.h"
#include "sockimpl.h"

/*
	Note:
	- All utility functions expects (and returns) values to be in network byte order (Big endian).
*/

void create_sockaddr_from_address(const Address& address, const Port port, sockaddr_storage* sockaddr);
void create_address_from_sockaddr(Address& address, Port& port, const sockaddr_storage* sockaddr);
Address create_address_from_ipv4addr(uint32_t inaddr);

