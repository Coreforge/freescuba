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
	protocol::Response_t SendBlocking(const protocol::Request_t& request) const;

	void Send(const protocol::Request_t& request) const;
	protocol::Response_t Receive() const;

private:
	#ifdef WIN32
	HANDLE pipe = INVALID_HANDLE_VALUE;
	#else
	FILE* pipe = nullptr; 
	#endif
};