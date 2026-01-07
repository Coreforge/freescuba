#include "ipc_client.hpp"
#include <stdexcept>
#include <string>
#include <filesystem>

#ifndef WIN32
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
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

	path /= "freescuba_pipe";
	return path.string();
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
		#warning "Probably don't actually create the pipe here"
		return;
		if(mkfifo(path.c_str(), 0660) == -1){
			throw std::runtime_error(std::string("Could not create named pipe: errno ") + std::to_string(errno));
		}
		pipe = fopen(path.c_str(), "r+");
		if(pipe == nullptr){
			throw std::runtime_error(std::string("Could not open named pipe: errno ") + std::to_string(errno));
		}
		break;
	}
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

protocol::Response_t IPCClient::SendBlocking( const protocol::Request_t& request ) const
{
	Send( request );
	return Receive();
}

void IPCClient::Send( const protocol::Request_t& request ) const
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
	#warning IPC stubbed!
	return;
	size_t ret = fwrite(&request, sizeof(request), 1, pipe);
	if(ret != 1){
		int err = ferror(pipe);
		throw std::runtime_error("Error writing IPC request. Error " + std::to_string(err));
	}
	#endif
}

protocol::Response_t IPCClient::Receive() const
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
	#warning IPC stubbed!
	return response;
	size_t ret = fread(&response, sizeof(response), 1, pipe);
	if(ret != 1){
		int err = ferror(pipe);
		throw std::runtime_error("Error reading IPC response. Error " + std::to_string(err));
	}
	#endif

	return response;
}