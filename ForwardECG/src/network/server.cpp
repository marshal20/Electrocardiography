#include "server.h"
#include "sockimpl.h"
#include <functional>


Server::Server()
{

}

Server::~Server()
{

}

bool Server::start(const Address addr, const Port port)
{
	// check if socket is running
	if (m_sock.is_valid())
	{
		return false;
	}

	// create socket
	m_sock = Socket(Socket::Type::Stream);
	if (!m_sock.is_valid())
	{
		return false;
	}

	if (!m_sock.bind(addr, port))
	{
		m_sock.close();
		m_sock = Socket();
		return false;
	}
	
	m_sock.listen();


	// start server thread
	m_server_thread = std::thread(std::bind(&Server::server_thread_routine, this));
	m_server_thread.detach();
	m_is_running = true;

	return true;
}

bool Server::stop()
{
	if (!m_sock.is_valid())
	{
		return false;
	}

	m_sock.close();
	m_sock = Socket();
	if (m_server_thread.joinable())
	{
		m_server_thread.join();
	}
	return true;
}

bool Server::is_running() const
{
	return m_is_running;
}


bool Server::poll_request(std::vector<uint8_t>& request_bytes, Address* req_address, Port* req_port)
{
	if (m_got_request)
	{
		request_bytes = m_request_bytes;
		*req_address = m_request_addr;
		*req_port = m_request_port;

		return true;
	}

	return false;
}

bool Server::push_response(const std::vector<uint8_t>& response_bytes)
{
	if (!m_got_request)
	{
		return false;
	}

	m_response_bytes = response_bytes;
	m_got_request = false;
	m_response_semaphore.notify();

	return true;
}


void Server::server_thread_routine()
{
	while (true)
	{
		// Accept the new connection
		Socket new_sock;
		Address new_addr;
		Port new_port;
		new_sock = m_sock.accept(new_addr, new_port);
		if (!new_sock.is_valid())
		{
			// exit if the main socket closed
			break;
		}

		// handle request

		// read request size
		uint32_t request_size = 0;
		new_sock.recv((char*)&request_size, sizeof(uint32_t));
		request_size = ntohl(request_size);

		// read request
		int read_size = 0;
		std::vector<uint8_t> request_bytes(request_size, 0);
		read_size = new_sock.recv((char*)&request_bytes[0], request_bytes.size());
		request_bytes.resize(read_size);

		m_request_bytes = request_bytes;
		m_request_addr = new_addr;
		m_request_port = new_port;

		// wait for response
		m_got_request = true;
		m_response_semaphore.wait();

		m_got_request = false;
		std::vector<uint8_t> response_bytes = m_response_bytes;

		// send response size
		uint32_t response_size = response_bytes.size();
		response_size = htonl(response_size);
		new_sock.send_all((char*)&response_size, sizeof(uint32_t));

		// send response
		new_sock.send_all((const char*)&response_bytes[0], response_bytes.size());

		// wait for connection close
		read_size = new_sock.recv((char*)&request_bytes[0], request_bytes.size());
		new_sock.close();

	}

	if (m_sock.is_valid())
	{
		m_sock.close();
		m_sock = Socket();
	}
	m_is_running = false;
}

