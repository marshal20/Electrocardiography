#include "utils.h"


void create_sockaddr_from_address(const Address& address, const Port port, sockaddr_storage* sockaddr)
{
	sockaddr->ss_family = PF_INET;
	sockaddr_in* temp = (sockaddr_in*)sockaddr;
	unsigned char v4_interm[4] = { address.a, address.b, address.c, address.d };
	memset(&temp->sin_addr, 0, sizeof(sockaddr_in));
	memcpy(&temp->sin_addr, v4_interm, sizeof(v4_interm));
	temp->sin_port = htons(port);
}

void create_address_from_sockaddr(Address& address, Port& port, const sockaddr_storage* sockaddr)
{
	// PF_INET only
	sockaddr_in* temp = (sockaddr_in*)sockaddr;
	address = create_address_from_ipv4addr(temp->sin_addr.s_addr);
	port = ntohs(temp->sin_port);
}

// Works with value in network byte order.
Address create_address_from_ipv4addr(uint32_t inaddr)
{
	const uint8_t* vals = (const uint8_t*)&inaddr;

	return Address{ vals[0], vals[1], vals[2], vals[3] };
}

