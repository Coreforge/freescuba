#pragma once

#include <cstdio>
#include <openvr.h>
#include "../ipc_protocol.hpp"

#ifdef WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif

class IPCClient {
public:
	~IPCClient();

	void Connect();
	protocol::Response_t SendBlocking(const protocol::Request_t& request);

	void Send(const protocol::Request_t& request);
	protocol::Response_t Receive();
	#ifndef WIN32
	void Poll();
	bool IsConnected();
	#endif

private:
	#ifdef WIN32
	HANDLE pipe = INVALID_HANDLE_VALUE;
	#else
	int socketFd = -1;
	int connectionSocketFd = -1;
	FILE* pipe = nullptr; 
	#endif
};