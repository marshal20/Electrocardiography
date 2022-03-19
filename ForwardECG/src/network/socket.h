#pragma once
#include <stdint.h>
#include <string>
#include <vector>


struct sockaddr_storage;


typedef uint16_t Port;

// IPv4 Address

struct Address 
{
	uint8_t a, b, c, d;

	std::string to_string() const;
};

#define ADDRESS_LOCALHOST Address{127, 0, 0, 1}
#define ADDRESS_THISHOST  Address{0, 0, 0, 0}
#define ADDRESS_BROADCAST Address{255, 255, 255, 255}


// Utility functions

bool initialize_socket();
std::string address_to_representation(const Address& address);
std::vector<Address> address_from_representation_all(const std::string& rep);
Address address_from_representation(const std::string& rep);


// Socket

class Socket
{
public:
	enum class Type
	{
		Stream = 1,
		Dgram
	};

	enum class How
	{
		Read = 1,
		Write,
		ReadWrite
	};

	Socket();
	Socket(Type type);
	~Socket();

	bool is_valid_const() const;
	operator bool() const;

	bool shutdown(How how);
	bool close();
	bool be_broadcast();

	bool connect(const Address& address, const Port port);
	bool bind(const Address& address, const Port port);
	bool listen(int prelog = 10);
	Socket accept(Address& connected_addr, Port& connected_port);

	int recv(char* buff, int len);
	int send(const char* buff, int len);
	void send_all(const char* buff, int len);

	void get_peer(Address& connected_addr, Port& connected_port) const;
	int get_total_recv() const;
	int get_total_sent() const;

private:
	int m_sock = -1;
	Type m_type = Type::Stream;
	struct Stats
	{
		int recv;
		int sent;
	} m_stats = { 0, 0 };
};

