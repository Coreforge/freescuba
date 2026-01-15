#include "ipc_client.hpp"
#include <bits/types/struct_iovec.h>
#include <cerrno>
#include <cstddef>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <string>
#include <filesystem>

#ifndef WIN32
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>
#endif

#ifdef WIN32
const std::string WStringToString( const std::wstring& wstr )
{
	const int size_needed = WideCharToMultiByte( CP_UTF8, 0, &wstr[0], (int)wstr.size(), NULL, 0, NULL, NULL );
	std::string str_to( size_needed, 0 );
	WideCharToMultiByte( CP_UTF8, 0, &wstr[0], (int)wstr.size(), &str_to[0], size_needed, NULL, NULL );
	return str_to;
}

static const std::string LastErrorString( const DWORD lastError )
{
	LPWSTR buffer = nullptr;
	const size_t size = FormatMessageW(
		FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
		NULL, lastError, MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT), (LPWSTR)&buffer, 0, NULL
	);
	std::wstring message( buffer, size );
	LocalFree( buffer );
	return WStringToString( message );
}
#endif

IPCClient::~IPCClient()
{
	#ifdef WIN32
	if ( pipe && pipe != INVALID_HANDLE_VALUE ) {
		CloseHandle( pipe );
	}
	#else
	if (pipe) {
		fclose(pipe);
	}
	#endif
}

#ifndef WIN32
std::string getPipePath(){
	char* xdg_runtime_dir = getenv("XDG_RUNTIME_DIR");
	std::filesystem::path path = (xdg_runtime_dir == nullptr) ? std::filesystem::path("/tmp") : std::filesystem::path(xdg_runtime_dir);

	path /= "freescuba_socket";
	return path.string();
}

void IPCClient::Poll(){
	int acceptFd = accept4(socketFd, nullptr, 0, SOCK_NONBLOCK);
	if(acceptFd == -1){
		// since non-blocking IO is used, this will happen quite often
		if(!(errno == EAGAIN || errno == EWOULDBLOCK)){
			fprintf(stderr, "IPC socket accept failed: %d %s\n", errno, strerror(errno));
			return;
		}
	} else {
		if(connectionSocketFd == -1){
			connectionSocketFd = acceptFd;
		} else {
			if(close(acceptFd) == -1){
				fprintf(stderr, "IPC socket close of additional connection failed: %d %s\n", errno, strerror(errno));
				return;
			}
		}
	}
}

bool IPCClient::IsConnected(){
	if(connectionSocketFd != -1){
		return true;
	}
	return false;
}
#endif

// @TODO: Make exceptionless
void IPCClient::Connect()
{
	// Connect to pipe job
	#ifdef WIN32
	while ( true ) {

		LPTSTR pipeName = ( LPTSTR ) TEXT ( FREESCUBA_PIPE_NAME );
		pipe = CreateFile (
			pipeName,   // pipe name 
			GENERIC_READ |  // read and write access 
			GENERIC_WRITE,
			0,              // no sharing 
			NULL,           // default security attributes
			OPEN_EXISTING,  // opens existing pipe 
			0,              // default attributes 
			NULL );         // no template file 

		// Break if the pipe handle is valid. 
		if ( pipe != INVALID_HANDLE_VALUE ) {
			break;
		}

		// Exit if an error other than ERROR_PIPE_BUSY occurs. 
		if ( GetLastError() != ERROR_PIPE_BUSY ) {
			throw std::runtime_error(std::string("Could not open pipe. Got error ") + std::to_string(GetLastError()));
		}

		// All pipe instances are busy, so wait for 20 seconds. 
		if ( !WaitNamedPipe( pipeName, 20000 ) ) {
			throw std::runtime_error("Could not open pipe: 20 second wait timed out.");
		}
	}


	DWORD mode = PIPE_READMODE_MESSAGE;
	if ( !SetNamedPipeHandleState( pipe, &mode, 0, 0 ) ) {
		const DWORD lastError = GetLastError();
		throw std::runtime_error("Couldn't set pipe mode. Error " + std::to_string( lastError ) + ": " + LastErrorString( lastError ));
	}
	#else
	while(true){
		std::string path = getPipePath();
		// Note: I'm switching around the client-server roles here, as I think it makes more sense this way
		#warning "Probably don't actually create the pipe here"
		if((socketFd = socket(AF_UNIX, SOCK_SEQPACKET, 0)) == -1){
			throw std::runtime_error(std::string("Could not create socket: errno ") + std::to_string(errno));
		}
		if(fcntl(socketFd, F_SETFL, O_NONBLOCK) == -1){
			throw std::runtime_error(std::string("Failed to set socket to non-blocking: errno ") + std::to_string(errno));
		}
		sockaddr_un addr;
		addr.sun_family = AF_UNIX;
		std::memset(addr.sun_path, 0, sizeof(addr.sun_path));
		std::strncpy(addr.sun_path, path.c_str(), sizeof(addr.sun_path) - 1);
		// loop used for easy bind retry
		int retryBind = 0;
		while(retryBind < 2){
			if(bind(socketFd, (sockaddr*)&addr, sizeof(addr)) == -1){
				if(errno == EADDRINUSE){
					// this may or may not work, but if the socket is still hanging around from an earlier launch, just connect to it
					if(connect(socketFd, (sockaddr*)&addr, sizeof(addr)) == -1){
						// if this errors with ECONNREFUSED, it's a dangling socket that can be removed
						if(errno == ECONNREFUSED){
							if(unlink(path.c_str())){
								throw std::runtime_error(std::string("Could not unlink old socket: errno ") + std::to_string(errno));			
							}
							retryBind++;
							continue;
						}
						throw std::runtime_error(std::string("Could not connect to socket (after binding failed): errno ") + std::to_string(errno));
					}
				} else {
					throw std::runtime_error(std::string("Could not bind socket: errno ") + std::to_string(errno));
				}
			}
			break;
		}
		if(listen(socketFd, 1) == -1){
			throw std::runtime_error(std::string("Could not listen on socket: errno ") + std::to_string(errno));
		}
		break;
	}
	return;
	#endif

	const protocol::Response_t response = SendBlocking( protocol::Request_t( protocol::RequestHandshake ) );
	if ( response.type != protocol::ResponseHandshake || response.protocol.version != protocol::Version )
	{
		throw std::runtime_error(
			"Incorrect driver version installed, try reinstalling FreeScuba. (Client: " +
			std::to_string( protocol::Version ) +
			", Driver: " +
			std::to_string( response.protocol.version ) +
			")"
		);
	}
}

protocol::Response_t IPCClient::SendBlocking( const protocol::Request_t& request )
{
	Send( request );
	return Receive();
}

void IPCClient::Send( const protocol::Request_t& request )
{
	#ifdef WIN32
	DWORD bytesWritten;
	const BOOL success = WriteFile( pipe, &request, sizeof request, &bytesWritten, 0 );
	if ( !success )
	{
		const DWORD lastError = GetLastError();
		throw std::runtime_error( "Error writing IPC request. Error " + std::to_string( lastError ) + ": " + LastErrorString( lastError ) );
	}
	#else

	msghdr hdr;
	hdr.msg_name = nullptr;
	hdr.msg_namelen = 0;
	iovec iov;
	// cast discards const, but no data should be written
	iov.iov_base = (void*)&request;
	iov.iov_len = sizeof(request);

	hdr.msg_iov = &iov;
	hdr.msg_iovlen = 1;
	hdr.msg_control = nullptr;
	hdr.msg_controllen = 0;
	hdr.msg_flags = 0;

	size_t ret = sendmsg(connectionSocketFd, &hdr, 0);
	if(ret == -1){
		fprintf(stderr, "Failed to write to IPC socket: %d %s\n", errno, strerror(errno));
		close(connectionSocketFd);
		connectionSocketFd = -1;
	}
	#endif
}

protocol::Response_t IPCClient::Receive()
{
	protocol::Response_t response(protocol::ResponseInvalid);
	#ifdef WIN32
	DWORD bytesRead;

	const BOOL success = ReadFile( pipe, &response, sizeof response, &bytesRead, 0 );
	if ( !success ) {
		const DWORD lastError = GetLastError();
		if (lastError != ERROR_MORE_DATA) {
			throw std::runtime_error( "Error reading IPC response. Error " + std::to_string( lastError ) + ": " + LastErrorString( lastError ) );
		}
	}

	if ( bytesRead != sizeof response ) {
		throw std::runtime_error( "Invalid IPC response with size " + std::to_string( bytesRead ) );
	}
	#else

	iovec iov;
	iov.iov_base = &response;
	iov.iov_len = sizeof(response);
	msghdr hdr;
	hdr.msg_name = nullptr;
	hdr.msg_namelen = 0;
	hdr.msg_iov = &iov;
	hdr.msg_iovlen = 1;

	size_t ret = recvmsg(connectionSocketFd, &hdr, 0);
	if(ret == -1){
		if(errno != EAGAIN && errno != EWOULDBLOCK){
			fprintf(stderr, "Failed to read from IPC socket: %d %s\n", errno, strerror(errno));
			close(connectionSocketFd);
			connectionSocketFd = -1;
		}
	} else if(ret == 0 || ret == sizeof(response)){
		// nothing or probably a correct packet, response will be returned
	} else {
		// something with a weird size was received, protocol error
		fprintf(stderr, "Received IPC packet with unexpected length %ld\n", ret);
		response = protocol::Response_t(protocol::ResponseInvalid);
	}
	#endif

	return response;
}