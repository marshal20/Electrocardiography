#pragma once
#include <vector>
#include <mutex>
#include <thread>
#include "socket.h"
#include "semaphore.h"


class Server
{
public:
	Server();
	~Server();

	bool start(const Address addr, const Port port);
	bool stop();

	bool poll_request(std::vector<uint8_t>& request_bytes, Address* req_address = NULL, Port* req_port = NULL);
	bool push_response(const std::vector<uint8_t>& response_bytes);

private:
	void server_thread_routine();

private:
	Socket m_sock;
	std::thread m_server_thread;
	Semaphore m_exit_thread_semaphore;
	Semaphore m_response_semaphore;
	bool m_got_request = false;
	Address m_request_addr; Port m_request_port;
	std::vector<uint8_t> m_request_bytes;
	std::vector<uint8_t> m_response_bytes;

};
