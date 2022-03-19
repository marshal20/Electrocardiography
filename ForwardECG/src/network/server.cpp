#include "server.h"
#include <functional>


Server::Server()
{

}

Server::~Server()
{

}

bool Server::start(const Address addr, const Port port)
{
	// socket already been created
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

	return true;
}

bool Server::stop()
{
	if (!m_sock.is_valid())
	{
		return false;
	}

	m_exit_thread_semaphore.notify();
	if (m_server_thread.joinable())
	{
		m_server_thread.join();
	}
	return true;
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
	//// simple echo
	//{
	//	// Accept the new connection
	//	Socket new_sock;
	//	Address new_addr;
	//	Port new_port;
	//	new_sock = m_sock.accept(new_addr, new_port);

	//	char buff[1024];
	//	int currec;
	//	while ((currec = new_sock.recv(buff, sizeof(buff))) != 0)
	//	{
	//		new_sock.send(buff, currec);
	//	}

	//	return;
	//}
	
	//printf("Server thread\n");

	while (true)
	{
		// wait for exit signal
		if (m_exit_thread_semaphore.wait(0) == WAIT_RESULT_NO_TIMEOUT)
		{
			//printf("Server thread exited\n");
			break;
		}

		// Accept the new connection
		Socket new_sock;
		Address new_addr;
		Port new_port;
		new_sock = m_sock.accept(new_addr, new_port);


		// handle request

		// read request
		int read_size = 0;
		std::vector<uint8_t> request_bytes(8192, 0);
		read_size = new_sock.recv((char*)&request_bytes[0], sizeof(request_bytes.size()));
		request_bytes.resize(read_size);

		m_request_bytes = request_bytes;
		m_request_addr = new_addr;
		m_request_port = new_port;

		// wait for response
		m_got_request = true;
		m_response_semaphore.wait();

		//printf("Accepted a response\n");

		m_got_request = false;
		std::vector<uint8_t> response_bytes = m_response_bytes;

		// send response
		new_sock.send_all((const char*)&response_bytes[0], response_bytes.size());
		new_sock.close();

		//printf("sent the response (%d)\n", response_bytes.size());

	}

	m_sock.close();
	m_sock = Socket();
}

