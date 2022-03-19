#include "socket.h"
#include "sockimpl.h"
#include "utils.h"


static void addrinfo_to_sockaddrstorage(const addrinfo* src, sockaddr_storage* dst)
{
	memset(dst, 0, sizeof(sockaddr_storage));
	int struct_size = (src->ai_family == AF_INET) ? sizeof(sockaddr_in) : sizeof(sockaddr_in6);
	memcpy(dst, src->ai_addr, struct_size);
}



// IPv4 Address

std::string Address::to_string() const
{
	return std::to_string(a) + "." + std::to_string(b) + "." + std::to_string(c) + "." + std::to_string(d);
}



// Utitlity functions

bool initialize_socket()
{
	return initialize_sockets_api();
}

std::string address_to_representation(const Address& address)
{
	sockaddr_storage temp_addr;
	create_sockaddr_from_address(address, 0, &temp_addr);

	static const int temp_len = INET6_ADDRSTRLEN;
	static char temp[temp_len + 1];
	void* addr;

	if (temp_addr.ss_family == AF_INET)
	{
		struct sockaddr_in* ipv4 = (struct sockaddr_in*)&temp_addr;
		addr = &(ipv4->sin_addr);
	}
	else
	{
		struct sockaddr_in6* ipv6 = (struct sockaddr_in6*)&temp_addr;
		addr = &(ipv6->sin6_addr);
	}

	inet_ntop(temp_addr.ss_family, addr, temp, temp_len);

	temp[temp_len] = '\0';

	return std::string(temp);
}

std::vector<Address> address_from_representation_all(const std::string& rep)
{
	std::vector<Address> addrs;

	struct addrinfo hints, *res, *it;
	const char* domainname = rep.c_str();

	memset(&hints, 0, sizeof(struct addrinfo));
	hints.ai_family = AF_UNSPEC;
	hints.ai_socktype = SOCK_STREAM;

	int code = getaddrinfo_impl(domainname, NULL, &hints, &res);
	if (code != 0)
	{
		return {};
	}

	for (it = res; it != NULL; it = it->ai_next)
	{
		Address temp;
		unsigned short dummy_port = 0;
		sockaddr_storage temp_sockaddr_storage;
		addrinfo_to_sockaddrstorage(it, &temp_sockaddr_storage);
		create_address_from_sockaddr(temp, dummy_port, &temp_sockaddr_storage);
		addrs.push_back(temp);
	}

	freeaddrinfo(res);

	return addrs;
}

Address address_from_representation(const std::string& rep)
{
	std::vector<Address> all_addresses = address_from_representation_all(rep);
	if (all_addresses.size())
	{
		return all_addresses[0];
	}

	return { 0, 0, 0, 0 };
}



// Socket

Socket::Socket()
	: m_sock(-1)
{

}

Socket::Socket(Type type) 
	: m_type(type)
{
	int t = (type == Type::Stream) ? SOCK_STREAM : SOCK_DGRAM;

	m_sock = socket_impl(PF_INET, t, 0);
}

Socket::~Socket()
{

}

bool Socket::is_valid_const() const
{
	return (m_sock == -1) ? false : true;
}

Socket::operator bool() const
{
	return is_valid_const();
}

bool Socket::shutdown(How how)
{
	if (m_sock == -1) return;

	int how_pram = 0;
	switch (how)
	{
	case How::Read: how_pram = SHUT_RD; break;
	case How::Write: how_pram = SHUT_WR; break;
	case How::ReadWrite: how_pram = SHUT_RDWR; break;
	default: how_pram = 0;
	}

	if (::shutdown(m_sock, how_pram) == -1)
	{
		return false;
	}

	return true;
}

bool Socket::close()
{
	if (m_sock == -1) return;
	if (::close_impl(m_sock) == -1)
	{
		return false;
	}

	return true;
}

bool Socket::be_broadcast()
{
	if (m_type != Type::Dgram)
	{
		return false;
	}

	int broadcast = 1;
	if (setsockopt(m_sock, SOL_SOCKET, SO_BROADCAST, (const char*)&broadcast, sizeof(broadcast)) == -1)
	{
		return false;
	}

	return true;
}


bool Socket::connect(const Address& address, const Port port)
{
	sockaddr_storage temp_sockaddr_storage;
	create_sockaddr_from_address(address, port, &temp_sockaddr_storage);
	if (::connect(m_sock, (const sockaddr*)&temp_sockaddr_storage, sizeof(sockaddr_in)) == -1)
	{
		return false;
	}

	return true;
}

bool Socket::bind(const Address& address, const Port port)
{
	sockaddr_storage temp_sockaddr_storage;
	create_sockaddr_from_address(address, port, &temp_sockaddr_storage);
	if (::bind(m_sock, (const sockaddr*)&temp_sockaddr_storage, sizeof(sockaddr_in)) == -1)
	{
		return false;
	}

	return true;
}

bool Socket::listen(int prelog = 10)
{
	if (::listen(m_sock, prelog) == -1)
	{
		return false;
	}

	return true;
}

Socket Socket::accept(Address& connected_addr, Port& connected_port)
{
	Socket sock;

	int len = sizeof(sockaddr_in);
	sockaddr_storage temp_sockaddr_storage;
	if ((sock.m_sock = ::accept(m_sock, (sockaddr*)&temp_sockaddr_storage, (socklen_t*)&len)) == -1)
	{
		return Socket();
	}

	create_address_from_sockaddr(connected_addr, connected_port, &temp_sockaddr_storage);

	return sock;
}


int Socket::recv(char* buff, int len)
{
	int recieved;
	if ((recieved = ::recv(m_sock, (char*)buff, len, 0)) == -1)
	{
		return -1;
	}

	m_monitor.recv += recieved;
	return recieved;
}

int Socket::send(const char* buff, int len)
{
	int sent;
	if ((sent = ::send(m_sock, (const char*)buff, len, 0)) == -1)
	{
		return -1;
	}

	m_monitor.sent += sent;
	return sent;
}

void Socket::send_all(const char* buff, int len)
{
	int sent = 0;
	while (sent < len)
	{
		sent += send((buff + sent), len - sent);
	}
}


void Socket::get_peer(Address& peer_addr, Port& peer_port) const
{
	sockaddr peer_sockaddr;
	socklen_t peer_sockaddr_len;

	if (::getpeername(m_sock, &peer_sockaddr, &peer_sockaddr_len) == -1)
	{
		peer_addr = { 0, 0, 0, 0 };
		peer_port = 0;
	}

	create_address_from_sockaddr(peer_addr, peer_port, (const sockaddr_storage*)&peer_sockaddr);
}

int Socket::get_total_recv() const
{
	return m_stats.recv;
}

int Socket::get_total_sent() const
{
	return m_stats.sent;
}

