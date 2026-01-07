#include "serial_communication.hpp"
#include "contact_glove_structs.hpp"
#include "crc.hpp"
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <string>
#include <sys/types.h>
#ifdef WIN32
#include <SetupAPI.h>
#else 
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <sys/ioctl.h>
#endif

#include <cstring>
#include <filesystem>

#include <chrono>
#include "cobs.hpp"
#include <iomanip>

static const std::string c_serialDeviceId = "VID_10C4&PID_7B27";
static const uint32_t LISTENER_WAIT_TIME = 1000;

static void PrintBuffer(const std::string name, const uint8_t* buffer, const size_t size) {
    // @FIXME: Can we make this neater using printf?
    std::cout << "uint8_t " << name << "[" << std::dec << size << "] = { " << std::flush;

    for (size_t i = 0; i < size; i++) {
        std::cout << "0x" << std::uppercase << std::hex << std::setfill('0') << std::setw(2) << (((int)buffer[i]) & 0xFF) << std::flush;
        if (i != size - 1) {
            std::cout << ", " << std::flush;
        }
    }

    std::cout << " };" << std::endl;
}

#ifdef WIN32
int SerialCommunicationManager::GetComPort() const {

    HDEVINFO DeviceInfoSet;
    SP_DEVINFO_DATA DeviceInfoData;
    DEVPROPTYPE ulPropertyType;
    DWORD DeviceIndex       = 0;
    std::string DevEnum     = "USB";
    char szBuffer[1024]     = { 0 };
    DWORD dwSize            = 0;
    DWORD Error             = 0;

    DeviceInfoSet = SetupDiGetClassDevsA(NULL, DevEnum.c_str(), NULL, DIGCF_ALLCLASSES | DIGCF_PRESENT);

    if (DeviceInfoSet == INVALID_HANDLE_VALUE) {
        return -1;
    }

    ZeroMemory(&DeviceInfoData, sizeof(SP_DEVINFO_DATA));
    DeviceInfoData.cbSize = sizeof(SP_DEVINFO_DATA);
    // Receive information about an enumerated device

    while ( SetupDiEnumDeviceInfo(DeviceInfoSet, DeviceIndex, &DeviceInfoData) ) {
        DeviceIndex++;

        // Retrieves a specified Plug and Play device property
        if (SetupDiGetDeviceRegistryPropertyA(
            DeviceInfoSet,
            &DeviceInfoData,
            SPDRP_HARDWAREID,
            &ulPropertyType,
            (BYTE*)szBuffer,
            sizeof(szBuffer),  // The size, in bytes
            &dwSize)) {

            if ( std::string(szBuffer).find(c_serialDeviceId) == std::string::npos ) {
                continue;
            }

            HKEY hDeviceRegistryKey = { 0 };
            hDeviceRegistryKey      = SetupDiOpenDevRegKey(DeviceInfoSet, &DeviceInfoData, DICS_FLAG_GLOBAL, 0, DIREG_DEV, KEY_READ);

            if ( hDeviceRegistryKey == INVALID_HANDLE_VALUE ) {
                Error = GetLastError();
                break;
            } else {
                char pszPortName[20]    = { 0 };
                DWORD dwSize            = sizeof(pszPortName);
                DWORD dwType            = 0;

                if (
                    ( RegQueryValueExA(hDeviceRegistryKey, "PortName", NULL, &dwType, (LPBYTE)pszPortName, &dwSize) == ERROR_SUCCESS ) &&
                    ( dwType == REG_SZ ) )
                {
                    // @FIXME: Avoid allocating memory by using a std::string
                    //         Should we consider implementing a substr function which takes in a char* ?
                    std::string sPortName = pszPortName;
                    try {
                        if ( sPortName.substr( 0, 3 ) == "COM" ) {
                            int nPortNr = std::stoi( pszPortName + 3 );
                            if ( nPortNr != 0 ) {
                                return nPortNr;
                            }
                        }
                    } catch ( ... ) {
                        printf("Parsing failed for a port\n");
                    }
                }
                RegCloseKey(hDeviceRegistryKey);
            }
        }
    } // while ( SetupDiEnumDeviceInfo(DeviceInfoSet, DeviceIndex, &DeviceInfoData) )

    if (DeviceInfoSet) {
        SetupDiDestroyDeviceInfoList(DeviceInfoSet);
    }

    return -1;
}
#endif

static std::string GetLastErrorAsString() {
    #ifdef WIN32
    DWORD errorMessageID = ::GetLastError();
    if (errorMessageID == 0) {
        return std::string();
    }

    LPSTR messageBuffer = nullptr;

    size_t size = FormatMessageA(
        FORMAT_MESSAGE_ALLOCATE_BUFFER | FORMAT_MESSAGE_FROM_SYSTEM | FORMAT_MESSAGE_IGNORE_INSERTS,
        NULL,
        errorMessageID,
        MAKELANGID(LANG_NEUTRAL, SUBLANG_DEFAULT),
        (LPSTR)&messageBuffer,
        0,
        NULL);

    std::string message(messageBuffer, size);

    LocalFree(messageBuffer);

    return message;
    #endif
    return strerror(errno);
}

#ifdef WIN32
bool SerialCommunicationManager::SetCommunicationTimeout(
    const uint32_t ReadIntervalTimeout,
    const uint32_t ReadTotalTimeoutMultiplier,
    const uint32_t ReadTotalTimeoutConstant,
    const uint32_t WriteTotalTimeoutMultiplier,
    const uint32_t WriteTotalTimeoutConstant) const {

    COMMTIMEOUTS timeout = {
        .ReadIntervalTimeout            = MAXDWORD,
        .ReadTotalTimeoutMultiplier     = MAXDWORD,
        .ReadTotalTimeoutConstant       = 10,
        .WriteTotalTimeoutMultiplier    = WriteTotalTimeoutMultiplier,
        .WriteTotalTimeoutConstant      = WriteTotalTimeoutConstant,
    };

    if (!SetCommTimeouts(m_hSerial, &timeout)) {
        return false;
    }

    return true;
}
#endif

bool SerialCommunicationManager::Connect() {
#ifdef WIN32
    // We're not yet connected
    m_isConnected = false;

    LogMessage("Attempting connection to dongle...");

    short port = GetComPort();
    m_port = "\\\\.\\COM" + std::to_string(port);

    // Try to connect to the given port throuh CreateFile
    m_hSerial = CreateFileA(m_port.c_str(), GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);

    if (m_hSerial == INVALID_HANDLE_VALUE) {
        LogError("Received error connecting to port");
        return false;
    }

    // If connected we try to set the comm parameters
    DCB dcbSerialParams = { 0 };

    if (!GetCommState(m_hSerial, &dcbSerialParams)) {
        LogError("Failed to get current port parameters");
        return false;
    }

    // Define serial connection parameters for the arduino board
    dcbSerialParams.BaudRate        = CBR_115200;
    dcbSerialParams.ByteSize        = 8;
    dcbSerialParams.StopBits        = ONESTOPBIT;
    dcbSerialParams.Parity          = NOPARITY;

    // reset upon establishing a connection
    dcbSerialParams.fDtrControl     = DTR_CONTROL_ENABLE;
    dcbSerialParams.XonChar         = 0x11;
    dcbSerialParams.XoffLim         = 0x4000;
    dcbSerialParams.XoffChar        = 0x13;
    dcbSerialParams.EofChar         = 0x1A;
    dcbSerialParams.EvtChar         = 0;

    // set the parameters and check for their proper application
    if (!SetCommState(m_hSerial, &dcbSerialParams)) {
        LogError("Failed to set serial parameters");
        return false;
    }

    if (!SetCommunicationTimeout(MAXDWORD, MAXDWORD, 1000, 5, 0)) {
        LogError("Failed to set communication timeout");
        return false;
    }

    COMMTIMEOUTS timeout = {
        .ReadIntervalTimeout            = MAXDWORD,
        .ReadTotalTimeoutMultiplier     = MAXDWORD,
        .ReadTotalTimeoutConstant       = 10,
        .WriteTotalTimeoutMultiplier    = 5,
        .WriteTotalTimeoutConstant      = 0,
    };

    if (!SetCommTimeouts(m_hSerial, &timeout)) {
        LogError("Failed to set comm timeouts");

        return false;
    }

    // If everything went fine we're connected
    m_isConnected = true;

    LogMessage("Successfully connected to dongle");
    return true;
#else
    m_isConnected = false;
    LogMessage("Attempting connection to dongle...");

    #warning "Hardcoded device path for now, replace!"
    std::filesystem::path path = "/dev/ttyContactGloves";
    serial_fd = open(path.c_str(), O_RDWR);
    if(serial_fd < 0){
        LogError((std::string("Failed to open serial port: ") + std::to_string(errno)).c_str());
        return false;
    }

    struct termios tty;
    if(tcgetattr(serial_fd, &tty) != 0){
        LogError((std::string("Failed to get serial port attributes: ") + std::to_string(errno)).c_str());
        return false;
    }

    // 8N1, no hardware flow control, no modem stuff
    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |= CS8 | CREAD | CLOCAL;

    // non-canonical mode (raw data), no signal characters
    tty.c_lflag &= ~(ICANON | ISIG);

    // no software flow control either, or any other special handling
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);
    tty.c_oflag &= ~(OPOST | ONLCR);
    tty.c_cc[VTIME] = 1;    // block infinitely
    tty.c_cc[VMIN] = 1; // min. 1 byte read
    cfsetispeed(&tty, B115200);
    cfsetospeed(&tty, B115200);

    if(tcsetattr(serial_fd, TCSANOW, &tty) != 0){
        LogError((std::string("Failed to get serial port attributes: ") + std::to_string(errno)).c_str());
        return false;
    }
    int flags;
    if(ioctl(serial_fd, TIOCMGET, &flags) == -1){
        LogError((std::string("Failed to get serial port attributes: ") + std::to_string(errno)).c_str());
        return false;
    }
    flags |= TIOCM_DTR;
    if(ioctl(serial_fd, TIOCMSET, &flags) == -1){
        LogError((std::string("Failed to set serial port attributes: ") + std::to_string(errno)).c_str());
        return false;
    }

    m_isConnected = true;

    LogMessage("Successfully connected to dongle");

    // send some stuff, maybe it'll stop the dongle from crashing all the time
    uint8_t cmd1[] = {0x7, 0x0, 0x6b};
    //WriteCommandRaw(cmd1,sizeof(cmd1));
    uint8_t cmd2[] = {0x16, 0x1, 0x1, 0x1, 0x1, 0x16};
    //WriteCommandRaw(cmd2,sizeof(cmd2));

    return true;
#endif
};

void SerialCommunicationManager::WaitAttemptConnection() {
    LogMessage("Attempting to connect to dongle");
    while (m_threadActive && !IsConnected() && !Connect()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(LISTENER_WAIT_TIME));
    }
    if (!m_threadActive) {
        return;
    }
}

void SerialCommunicationManager::BeginListener(
    const std::function<void(const ContactGloveDevice_t handedness, const GloveInputData_t&)> inputCallback,
    const std::function<void(const ContactGloveDevice_t handedness, const GlovePacketFingers2_t&)> fingersCallback,
    const std::function<void(const DevicesStatus_t&)> statusCallback,
    const std::function<void(const DevicesFirmware_t&)> firmwareCallback) {

    m_fingersCallback   = fingersCallback;
    m_inputCallback     = inputCallback;
    m_statusCallback    = statusCallback;
    m_firmwareCallback  = firmwareCallback;

    m_threadActive = true;
    m_serialThread = std::thread(&SerialCommunicationManager::ListenerThread, this);
}

void SerialCommunicationManager::ListenerThread() {
    WaitAttemptConnection();

    PurgeBuffer();
    size_t waitcounter = 0;
    while (m_threadActive) {
        std::string receivedString;

        try {
            if (!ReceiveNextPacket(receivedString)) {
                LogMessage("Detected device error. Disconnecting device and attempting reconnection...");
                // UpdateDongleState(VRDongleState::disconnected);

                if (DisconnectFromDevice(false)) {
                    WaitAttemptConnection();
                    LogMessage("Successfully reconnected to device");
                    continue;
                }

                LogMessage("Could not disconnect from device. Closing listener...");
                Disconnect();

                return;
            }
        }
        catch (const std::invalid_argument& ia) {
            LogMessage((std::string("Received error from encoding: ") + ia.what()).c_str());
        }
        catch (...) {
            LogMessage("Received unknown error attempting to decode packet.");
        }

        // If we haven't received anything don't bother with trying to decode anything
        if (receivedString.empty()) {
            goto finish;
        }

    finish:
        if((waitcounter % 100) == 0){
            // this is just an easy way to send data for testing for now, until it's better understood

            // seems to be unreliable, enables the magnetra (on the left glove)
            uint8_t cmd2[] = {0x16, 0x1, 0x1, 0x1, 0x1};  // sometimes makes the glove play the startup noise, changes packet 0x15
            WriteCommandRaw(cmd2,sizeof(cmd2));
            // and on the right glove (arg[0] seems to be the glove index then)
            uint8_t cmd4[] = {0x16, 0x2, 0x1, 0x1, 0x1};
            WriteCommandRaw(cmd4, sizeof(cmd4));

            uint8_t cmd1[] = {0x7, 0x0};
            //WriteCommandRaw(cmd1,sizeof(cmd1));
            uint8_t cmd3[] = {0x16, 0x1, 0x4, 0x3, 0x0, 0x0, 0xff};
            //WriteCommandRaw(cmd3,sizeof(cmd3));

            uint8_t cmd5[] = {0x18, 0x2, 0x1};
            WriteCommandRaw(cmd5,sizeof(cmd5));
            uint8_t cmd6[] = {0x18, 0x1, 0x0b};
            WriteCommandRaw(cmd6,sizeof(cmd6));
            // triggers haptics on the right glove
            // 10 [b1, b2, duration (uint16, ms), b3]
            // b1 and b2 are fully unknown, b3 must be anything but 0
            uint8_t cmd7[] = {0x10, 0x64, 0x00, 0x50, 0x00, 0xff};
            //WriteCommandRaw(cmd7,sizeof(cmd7));
            printf("sent\n");
        }
        // gets sent periodically. No idea what it means, but it keeps the dongle from crashing
        uint8_t keepalive[] = {0x1, 0x0, 0x0, 0xf0, 0x0, 0x0, 0xf0};
        WriteCommandRaw(keepalive,sizeof(keepalive));
        // write anything we need to
        WriteQueued();
        waitcounter++;
    }
}

bool SerialCommunicationManager::ReceiveNextPacket(std::string& buff) {
    #ifdef WIN32
    DWORD dwRead    = 0;
    #else
    ssize_t dwRead = 0;
    #endif

    char thisChar   = 0x00;
    char lastChar   = 0x00;

    do {
        lastChar = thisChar;
        #ifdef WIN32
        if (!ReadFile(m_hSerial, &thisChar, 1, &dwRead, NULL)) {
            LogError("Error reading from file");
            return false;
        }
        #else
        dwRead = read(serial_fd, &thisChar, 1);
        if(dwRead == -1){
            LogError("Error reading from serial port");
            return false;
        }
        #endif

        if (dwRead == 0) {
            break;
        }
        //printf("read byte %d\n", (int)thisChar);
        // Delimeter is 0, since data is encoded using COBS
        if (thisChar == 0) {
            // the last byte will be 0
            uint8_t* pData = (uint8_t*)malloc(buff.size());
            if (pData != 0) {
                std::memset(pData, 0, buff.size());
                std::memcpy(pData, buff.c_str(), buff.size());

                // Decode data
                cobs::decode(pData, buff.size());

                crc CRC_Result = F_CRC_CalculateCheckSum(pData, buff.size());

                if (CRC_Result == CRC_RESULT_OK) {
                    ContactGlovePacket_t packet = {};
                    if (DecodePacket(pData, buff.size(), &packet)) {
                        switch (packet.type) {
                            // Invoke callback
                        case PacketType_t::GloveLeftData:
                            m_inputCallback(ContactGloveDevice_t::LeftGlove, packet.packet.gloveData);
                            break;
                        case PacketType_t::GloveRightData:
                            m_inputCallback(ContactGloveDevice_t::RightGlove, packet.packet.gloveData);
                            break;
                        case PacketType_t::GloveLeftFingers:
                            m_fingersCallback(ContactGloveDevice_t::LeftGlove, packet.packet.gloveFingers);
                            break;
                        case PacketType_t::GloveRightFingers:
                            m_fingersCallback(ContactGloveDevice_t::RightGlove, packet.packet.gloveFingers);
                            break;
                        case PacketType_t::DevicesStatus:
                            m_statusCallback(packet.packet.status);
                            break;
                        case PacketType_t::DevicesFirmware:
                            m_firmwareCallback(packet.packet.firmware);
                            break;
                        }
                    }
                }

                free(pData);
            }

            // Clear buffer
            buff = "";
            return true;    // return to give the thread a chance to do other things (like sending data)
        }
        else {
            // Only add the current character if 0 we aren't on the delimeter
            buff += thisChar;
        }

        if (buff.size() > MAX_PACKET_SIZE) {
            LogError("Overflowed controller input. Resetting...");
            buff = "";
            break;
        }

    } while ((!(thisChar == -1 && lastChar == -1)) && m_threadActive);

    return true;
}

void SerialCommunicationManager::WriteCommand(const std::string& command) {
    std::scoped_lock lock(m_writeMutex);

    if (!m_isConnected) {
        LogMessage("Cannot write to dongle as it is not connected.");

        return;
    }

    m_queuedWrite = command + "\r\n";
}

// appends the CRC
void SerialCommunicationManager::WriteCommandRaw(const uint8_t* data, size_t size) {
    std::scoped_lock lock(m_writeMutex);

    if (!m_isConnected) {
        LogMessage("Cannot write to dongle as it is not connected.");

        return;
    }
    // all this string stuff is ugly, I should maybe replace it with a vector
    std::string buf(size + 1, 0);
    std::memcpy(buf.data(), data, size);
    crc checksum = F_CRC_CalculateCheckSum(data, size);
    buf.data()[size] = checksum;
    std::string encoded(size + 3, 0);
    cobs::encode((uint8_t*)buf.data(), buf.size(), (uint8_t*)encoded.data());

    m_queuedWrite += encoded;
}

bool SerialCommunicationManager::WriteQueued() {
    std::scoped_lock lock(m_writeMutex);

    if (!m_isConnected) {
        return false;
    }

    if (m_queuedWrite.empty()) {
        return true;
    }

    const char* buf = m_queuedWrite.data();
    #ifdef WIN32
    DWORD bytesSend;
    if (!WriteFile(this->m_hSerial, (void*)buf, (DWORD)m_queuedWrite.size(), &bytesSend, 0)) {
        LogError("Error writing to port");
        return false;
    }
    #else
    ssize_t sent = write(serial_fd, buf, m_queuedWrite.size());
    if(sent == -1){
        LogError("Error writing to port");
        return false;
    }
    if(sent != m_queuedWrite.size()){
        // if this happens, just make a loop that retries the unsent data
        LogError("Wrote too little to serial port");
        return false;
    }
    #endif

    //printf("Wrote: %s\n", m_queuedWrite.c_str());
    //PrintBuffer("SentData", (uint8_t*)m_queuedWrite.data(), m_queuedWrite.size());

    m_queuedWrite.clear();

    return true;
}

bool SerialCommunicationManager::PurgeBuffer() const {
    #ifdef WIN32
    return PurgeComm(m_hSerial, PURGE_RXCLEAR | PURGE_TXCLEAR);
    #else
    #warning "NOT IMPLEMENTED"
    return true;
    #endif
}

void SerialCommunicationManager::Disconnect() {
    printf("Attempting to disconnect serial\n");
    if (m_threadActive.exchange(false)) {
        #ifdef WIN32
        CancelIoEx(m_hSerial, nullptr);
        #else
        #warning "NOT IMPLEMENTED"
        #endif
        m_serialThread.join();

        printf("Serial joined\n");
    }

    if (IsConnected()) {
        DisconnectFromDevice(true);
    }

    printf("Serial finished disconnecting\n");
}

bool SerialCommunicationManager::DisconnectFromDevice(bool writeDeactivate) {
    if (writeDeactivate) {
        WriteCommand("BP+VS");
    }
    else {
        LogMessage("Not deactivating Input API as dongle was forcibly disconnected");
    }
    #ifdef WIN32
    if (!CloseHandle(m_hSerial)) {
        LogError("Error disconnecting from device");
        return false;
    }
    #else 
    if(!close(serial_fd)){
        LogError("Error disconnecting from device");
        return false;
    }
    #endif

    m_isConnected = false;

    LogMessage("Successfully disconnected from device");
    return true;
};

bool SerialCommunicationManager::IsConnected() const {
    return m_isConnected;
};

void SerialCommunicationManager::LogError(const char* message) const {
    // message with port name and last error
    printf("%s (%s) - Error: %s\n", message, m_port.c_str(), GetLastErrorAsString().c_str());
}

void SerialCommunicationManager::LogWarning(const char* message) const {
    // message with port name
    printf("%s (%s) - Warning: %s", message, m_port.c_str(), GetLastErrorAsString().c_str());
}

void SerialCommunicationManager::LogMessage(const char* message) const {
    // message with port name
    printf("%s (%s)\n", message, m_port.c_str());
}

bool SerialCommunicationManager::DecodePacket(const uint8_t* pData, const size_t length, ContactGlovePacket_t* outPacket) const {

    bool decoded = false;

    switch (pData[0]) {
    case GLOVE_LEFT_PACKET_DATA:
    {
        // Assign type
        outPacket->type = PacketType_t::GloveLeftData;

        // Extract data
        outPacket->packet.gloveData.joystickX = ((uint16_t*)(pData))[1];
        outPacket->packet.gloveData.joystickY = ((uint16_t*)(pData))[2];

        uint8_t button_mask = ((uint8_t*)pData)[1];

        // KNOWN VALUES:
        // None             :: 0x3F   0011 1111
        // System Up        :: 0x2F   0010 1111
        // System Down      :: 0x37   0011 0111
        // A (BTN_DOWN)     :: 0x3E   0011 1110
        // B (BTN_UP)       :: 0x3D   0011 1101
        // Joystick Click   :: 0x3B   0011 1011
        // Magnetra unavail :: 0x1F   0001 1111

        outPacket->packet.gloveData.hasMagnetra     =   (button_mask & CONTACT_GLOVE_INPUT_MASK_MAGNETRA_PRESENT)   == CONTACT_GLOVE_INPUT_MASK_MAGNETRA_PRESENT;
        outPacket->packet.gloveData.systemUp        = !((button_mask & CONTACT_GLOVE_INPUT_MASK_SYSTEM_UP)          == CONTACT_GLOVE_INPUT_MASK_SYSTEM_UP);
        outPacket->packet.gloveData.systemDown      = !((button_mask & CONTACT_GLOVE_INPUT_MASK_SYSTEM_DOWN)        == CONTACT_GLOVE_INPUT_MASK_SYSTEM_DOWN);
        outPacket->packet.gloveData.buttonUp        = !((button_mask & CONTACT_GLOVE_INPUT_MASK_BUTTON_UP)          == CONTACT_GLOVE_INPUT_MASK_BUTTON_UP);
        outPacket->packet.gloveData.buttonDown      = !((button_mask & CONTACT_GLOVE_INPUT_MASK_BUTTON_DOWN)        == CONTACT_GLOVE_INPUT_MASK_BUTTON_DOWN);
        outPacket->packet.gloveData.joystickClick   = !((button_mask & CONTACT_GLOVE_INPUT_MASK_JOYSTICK_CLICK)     == CONTACT_GLOVE_INPUT_MASK_JOYSTICK_CLICK);

        // printf("data[L]:: magnetra:%d, sysUp:%d, sysDn:%d, btnUp:%d, btnDn:%d, joyClk:%d (0x%04hX,0x%04hX)\n",
        //     outPacket->packet.gloveData.hasMagnetra,
        //     outPacket->packet.gloveData.systemUp,
        //     outPacket->packet.gloveData.systemDown,
        //     outPacket->packet.gloveData.buttonUp,
        //     outPacket->packet.gloveData.buttonDown,
        //     outPacket->packet.gloveData.joystickClick,
        //     outPacket->packet.gloveData.joystickX,
        //     outPacket->packet.gloveData.joystickY);

        // Left glove data packet
        // PrintBuffer("glove_data_left", pData, length);

        decoded = true;
    }

    break;
    case GLOVE_LEFT_PACKET_DATA_2:
    {
        // Assign type
        outPacket->type = PacketType_t::GloveLeftData;

        // Extract data
        outPacket->packet.gloveData.joystickX = 0xff - (pData)[3];
        outPacket->packet.gloveData.joystickY = (pData)[2];
        outPacket->packet.gloveData.trigger = pData[4];

        uint8_t button_mask = ((uint8_t*)pData)[1];
        uint8_t button2_mask = ((uint8_t*)pData)[5];

        // KNOWN VALUES:
        // None             :: 0x1f ff   0001 1111 1111 1111
        // "touchpad" Up    :: 0x1f 8a   0001 1111 1000 1010
        // "touchpad" Down  :: 0x17 ff   0001 0111 1111 1111
        // A (BTN_DOWN)     :: 0x1d ff   0001 1101 1111 1111
        // B (BTN_UP)       :: 0x1e ff   0001 1110 1111 1111
        // Joystick Click   :: 0x1f 01   0001 1111 0000 0001
        // System button       0x1f b9   0001 1111 1011 1001
        // trigger click       0x1b      0001 1011
        // the second button value seems to not be a mask, but just a few fixed values instead

        outPacket->packet.gloveData.hasMagnetra     =   (button_mask & CONTACT_GLOVE_2_INPUT_MASK_MAGNETRA_PRESENT)   == CONTACT_GLOVE_2_INPUT_MASK_MAGNETRA_PRESENT;
        outPacket->packet.gloveData.systemUp        = !((button2_mask & CONTACT_GLOVE_2_INPUT_MASK_SYSTEM_UP)          == CONTACT_GLOVE_2_INPUT_MASK_SYSTEM_UP);
        outPacket->packet.gloveData.systemDown      = !((button_mask & CONTACT_GLOVE_2_INPUT_MASK_SYSTEM_DOWN)        == CONTACT_GLOVE_2_INPUT_MASK_SYSTEM_DOWN);
        outPacket->packet.gloveData.buttonUp        = !((button_mask & CONTACT_GLOVE_2_INPUT_MASK_BUTTON_UP)          == CONTACT_GLOVE_2_INPUT_MASK_BUTTON_UP);
        outPacket->packet.gloveData.buttonDown      = !((button_mask & CONTACT_GLOVE_2_INPUT_MASK_BUTTON_DOWN)        == CONTACT_GLOVE_2_INPUT_MASK_BUTTON_DOWN);
        outPacket->packet.gloveData.joystickClick   = !((button2_mask & CONTACT_GLOVE_2_INPUT_MASK_JOYSTICK_CLICK)     == CONTACT_GLOVE_2_INPUT_MASK_JOYSTICK_CLICK);
        outPacket->packet.gloveData.triggerClick   = !((button_mask & CONTACT_GLOVE_2_INPUT_MASK_TRIGGER_CLICK)     == CONTACT_GLOVE_2_INPUT_MASK_TRIGGER_CLICK);
        outPacket->packet.gloveData.systemButton = button2_mask == CONTACT_GLOVE_2_INPUT_SYSTEM_BUTTON;

        // printf("data[R]:: magnetra:%d, sysUp:%d, sysDn:%d, btnUp:%d, btnDn:%d, joyClk:%d (0x%04hX,0x%04hX)\n",
        //     outPacket->packet.gloveData.hasMagnetra,
        //     outPacket->packet.gloveData.systemUp,
        //     outPacket->packet.gloveData.systemDown,
        //     outPacket->packet.gloveData.buttonUp,
        //     outPacket->packet.gloveData.buttonDown,
        //     outPacket->packet.gloveData.joystickClick,
        //     outPacket->packet.gloveData.joystickX,
        //     outPacket->packet.gloveData.joystickY);

        // Left glove data packet
         //PrintBuffer("glove_data_left", pData, length);

        decoded = true;
    }
    break;

    case GLOVE_RIGHT_PACKET_DATA:
    {
        // Assign type
        outPacket->type = PacketType_t::GloveRightData;

        // Extract data
        outPacket->packet.gloveData.joystickX = ((uint16_t*)(pData))[1];
        outPacket->packet.gloveData.joystickY = ((uint16_t*)(pData))[2];

        uint8_t button_mask = ((uint8_t*)pData)[1];

        // KNOWN VALUES:
        // None             :: 0x3F   0011 1111
        // System Up        :: 0x2F   0010 1111
        // System Down      :: 0x37   0011 0111
        // A (BTN_DOWN)     :: 0x3E   0011 1110
        // B (BTN_UP)       :: 0x3D   0011 1101
        // Joystick Click   :: 0x3B   0011 1011
        // Magnetra unavail :: 0x1F   0001 1111

        outPacket->packet.gloveData.hasMagnetra     =   (button_mask & CONTACT_GLOVE_INPUT_MASK_MAGNETRA_PRESENT)   == CONTACT_GLOVE_INPUT_MASK_MAGNETRA_PRESENT;
        outPacket->packet.gloveData.systemUp        = !((button_mask & CONTACT_GLOVE_INPUT_MASK_SYSTEM_UP)          == CONTACT_GLOVE_INPUT_MASK_SYSTEM_UP);
        outPacket->packet.gloveData.systemDown      = !((button_mask & CONTACT_GLOVE_INPUT_MASK_SYSTEM_DOWN)        == CONTACT_GLOVE_INPUT_MASK_SYSTEM_DOWN);
        outPacket->packet.gloveData.buttonUp        = !((button_mask & CONTACT_GLOVE_INPUT_MASK_BUTTON_UP)          == CONTACT_GLOVE_INPUT_MASK_BUTTON_UP);
        outPacket->packet.gloveData.buttonDown      = !((button_mask & CONTACT_GLOVE_INPUT_MASK_BUTTON_DOWN)        == CONTACT_GLOVE_INPUT_MASK_BUTTON_DOWN);
        outPacket->packet.gloveData.joystickClick   = !((button_mask & CONTACT_GLOVE_INPUT_MASK_JOYSTICK_CLICK)     == CONTACT_GLOVE_INPUT_MASK_JOYSTICK_CLICK);

        // printf("data[R]:: magnetra:%d, sysUp:%d, sysDn:%d, btnUp:%d, btnDn:%d, joyClk:%d (0x%04hX,0x%04hX)\n",
        //     outPacket->packet.gloveData.hasMagnetra,
        //     outPacket->packet.gloveData.systemUp,
        //     outPacket->packet.gloveData.systemDown,
        //     outPacket->packet.gloveData.buttonUp,
        //     outPacket->packet.gloveData.buttonDown,
        //     outPacket->packet.gloveData.joystickClick,
        //     outPacket->packet.gloveData.joystickX,
        //     outPacket->packet.gloveData.joystickY);

        // Right glove data packet
        // PrintBuffer("glove_data_right", pData, length);

        decoded = true;
    }
    break;
    case GLOVE_RIGHT_PACKET_DATA_2:
    {
        // Assign type
        outPacket->type = PacketType_t::GloveRightData;

        // Extract data
        outPacket->packet.gloveData.joystickX = 0xff - (pData)[3];
        outPacket->packet.gloveData.joystickY = (pData)[2];
        outPacket->packet.gloveData.trigger = pData[4];

        uint8_t button_mask = ((uint8_t*)pData)[1];
        uint8_t button2_mask = ((uint8_t*)pData)[5];

        // KNOWN VALUES:
        // None             :: 0x1f ff   0001 1111 1111 1111
        // "touchpad" Up    :: 0x1f 8a   0001 1111 1000 1010
        // "touchpad" Down  :: 0x17 ff   0001 0111 1111 1111
        // A (BTN_DOWN)     :: 0x1d ff   0001 1101 1111 1111
        // B (BTN_UP)       :: 0x1e ff   0001 1110 1111 1111
        // Joystick Click   :: 0x1f 01   0001 1111 0000 0001
        // System button       0x1f b9   0001 1111 1011 1001
        // trigger click       0x1b      0001 1011
        // the second button value seems to not be a mask, but just a few fixed values instead

        outPacket->packet.gloveData.hasMagnetra     =   (button_mask & CONTACT_GLOVE_2_INPUT_MASK_MAGNETRA_PRESENT)   == CONTACT_GLOVE_2_INPUT_MASK_MAGNETRA_PRESENT;
        outPacket->packet.gloveData.systemUp        = !((button2_mask & CONTACT_GLOVE_2_INPUT_MASK_SYSTEM_UP)          == CONTACT_GLOVE_2_INPUT_MASK_SYSTEM_UP);
        outPacket->packet.gloveData.systemDown      = !((button_mask & CONTACT_GLOVE_2_INPUT_MASK_SYSTEM_DOWN)        == CONTACT_GLOVE_2_INPUT_MASK_SYSTEM_DOWN);
        outPacket->packet.gloveData.buttonUp        = !((button_mask & CONTACT_GLOVE_2_INPUT_MASK_BUTTON_UP)          == CONTACT_GLOVE_2_INPUT_MASK_BUTTON_UP);
        outPacket->packet.gloveData.buttonDown      = !((button_mask & CONTACT_GLOVE_2_INPUT_MASK_BUTTON_DOWN)        == CONTACT_GLOVE_2_INPUT_MASK_BUTTON_DOWN);
        outPacket->packet.gloveData.joystickClick   = !((button2_mask & CONTACT_GLOVE_2_INPUT_MASK_JOYSTICK_CLICK)     == CONTACT_GLOVE_2_INPUT_MASK_JOYSTICK_CLICK);
        outPacket->packet.gloveData.triggerClick   = !((button_mask & CONTACT_GLOVE_2_INPUT_MASK_TRIGGER_CLICK)     == CONTACT_GLOVE_2_INPUT_MASK_TRIGGER_CLICK);
        outPacket->packet.gloveData.systemButton = button2_mask == CONTACT_GLOVE_2_INPUT_SYSTEM_BUTTON;

        // printf("data[R]:: magnetra:%d, sysUp:%d, sysDn:%d, btnUp:%d, btnDn:%d, joyClk:%d (0x%04hX,0x%04hX)\n",
        //     outPacket->packet.gloveData.hasMagnetra,
        //     outPacket->packet.gloveData.systemUp,
        //     outPacket->packet.gloveData.systemDown,
        //     outPacket->packet.gloveData.buttonUp,
        //     outPacket->packet.gloveData.buttonDown,
        //     outPacket->packet.gloveData.joystickClick,
        //     outPacket->packet.gloveData.joystickX,
        //     outPacket->packet.gloveData.joystickY);

        // Right glove data packet
         //PrintBuffer("glove_data_right", pData, length);

        decoded = true;
    }
    break;
    case DEVICES_VERSIONS:
    {
        // Assign type
        outPacket->type = PacketType_t::DevicesFirmware;

        // data is 0x01 0x06 3 times

        // This is version a.b
        outPacket->packet.firmware.dongleMajor      = ((uint8_t*)pData)[1];
        outPacket->packet.firmware.dongleMinor      = ((uint8_t*)pData)[2];
        outPacket->packet.firmware.gloveLeftMajor   = ((uint8_t*)pData)[3];
        outPacket->packet.firmware.gloveLeftMinor   = ((uint8_t*)pData)[4];
        outPacket->packet.firmware.gloveRightMajor  = ((uint8_t*)pData)[5];
        outPacket->packet.firmware.gloveRightMinor  = ((uint8_t*)pData)[6];

        // PrintBuffer("firmware_version", pData, length);

        decoded = true;
    }
    break;

    case DEVICES_STATUS:
    {
        // Assign type
        outPacket->type = PacketType_t::DevicesStatus;

        // uint8_t dongle4[8] = { 0x00, 0x1E, 0x50, 0x5F, 0x5A, 0x60, 0x01, 0xA2 };

        outPacket->packet.status.gloveLeftBattery   = ((uint8_t*)pData)[1];
        outPacket->packet.status.gloveRightBattery  = ((uint8_t*)pData)[3];

        // printf("glove[L]:: %d%%  ; glove[R]:: %d%%\n", outPacket->packet.status.gloveLeftBattery, outPacket->packet.status.gloveRightBattery);

        // PrintBuffer("device_status", pData, length);

        decoded = true;
    }
    break;


    case GLOVE_POWER_ON_PACKET:
    {
        // Assign type
        outPacket->type = PacketType_t::GlovePowerOn;

        // PrintBuffer("glove_power_on", pData, length);

        decoded = true;
    }
    break;


    case GLOVE_LEFT_PACKET_FINGERS:
    {
        // Assign type
        outPacket->type = PacketType_t::GloveLeftFingers;

        // PrintBuffer("glove_left_fingers", pData, length);
        //printf("Left glove: ");
        if(length == 37){
            // contactgloves2 data
            // indexroot: 10, 12
            // indextip: 11
            // middletip 8
            // middleroot 7, 9
            // ringroot: 4,6
            // ringtip: 5
            // pinkyroot: 1,3
            // pinkytip: 2
            // thumbroot: 16
            // thumbtip: 14
            // thumb is 13-16
            // 17 is unknown
            //for(size_t i = 0; i < 18; i++){
            //    printf("\t[%d]: %d\n", i, ((uint16_t*)(pData + 1))[i]);
            //}
            GlovePacketFingers2_t* fingers = (GlovePacketFingers2_t*)(pData + 1);
            outPacket->packet.gloveFingers = *fingers;
        } else {
            // Extract data
            outPacket->packet.gloveFingers.fingerThumbTip   = ((uint16_t*)(pData + 1))[9];
            outPacket->packet.gloveFingers.fingerThumbRoot  = ((uint16_t*)(pData + 1))[8];
            outPacket->packet.gloveFingers.fingerIndexTip   = ((uint16_t*)(pData + 1))[7];
            outPacket->packet.gloveFingers.fingerIndexRoot1  = ((uint16_t*)(pData + 1))[6];
            outPacket->packet.gloveFingers.fingerMiddleTip  = ((uint16_t*)(pData + 1))[5];
            outPacket->packet.gloveFingers.fingerMiddleRoot1 = ((uint16_t*)(pData + 1))[4];
            outPacket->packet.gloveFingers.fingerRingTip    = ((uint16_t*)(pData + 1))[3];
            outPacket->packet.gloveFingers.fingerRingRoot1   = ((uint16_t*)(pData + 1))[2];
            outPacket->packet.gloveFingers.fingerPinkyTip   = ((uint16_t*)(pData + 1))[0];
            outPacket->packet.gloveFingers.fingerPinkyRoot1  = ((uint16_t*)(pData + 1))[1];

            // printf(
            //     "fingers[L]:: (%d, %d) (%d, %d) (%d, %d) (%d, %d) (%d, %d)\n",
            //     outPacket->packet.gloveFingers.fingerThumbTip,
            //     outPacket->packet.gloveFingers.fingerThumbRoot,
            //     outPacket->packet.gloveFingers.fingerIndexTip,
            //     outPacket->packet.gloveFingers.fingerIndexRoot,
            //     outPacket->packet.gloveFingers.fingerMiddleTip,
            //     outPacket->packet.gloveFingers.fingerMiddleRoot,
            //     outPacket->packet.gloveFingers.fingerRingTip,
            //     outPacket->packet.gloveFingers.fingerRingRoot,
            //     outPacket->packet.gloveFingers.fingerPinkyTip,
            //     outPacket->packet.gloveFingers.fingerPinkyRoot);
        }

        decoded = true;

    }
    break;
    case GLOVE_LEFT_PACKET_IMU:
    {
        // @TODO: Imu data is not usable

        // Assign type
        outPacket->type = PacketType_t::GloveLeftImu;
        // Extract data
        outPacket->packet.gloveImu.imu1 = ((uint16_t*)(pData + 1))[0];
        outPacket->packet.gloveImu.imu2 = ((uint16_t*)(pData + 1))[1];

        outPacket->packet.gloveImu.imu3 = ((uint16_t*)(pData + 1))[2];
        outPacket->packet.gloveImu.imu4 = ((uint16_t*)(pData + 1))[3];

        outPacket->packet.gloveImu.imu5 = ((float*)(pData + 1))[4];
        outPacket->packet.gloveImu.imu6 = ((float*)(pData + 1))[5];

        // printf(
        //     "imu[L]:: %d, %d, %d, %d [%.6f, %.6f]\n",
        //     outPacket->packet.gloveImu.imu1,
        //     outPacket->packet.gloveImu.imu2,
        //     outPacket->packet.gloveImu.imu3,
        //     outPacket->packet.gloveImu.imu4,
        //     outPacket->packet.gloveImu.imu5,
        //     outPacket->packet.gloveImu.imu6);

        // PrintBuffer("glove_left_imu", pData, length);
        decoded = true;

    }
    break;

    case GLOVE_RIGHT_PACKET_FINGERS:
    {
        // Assign type
        outPacket->type = PacketType_t::GloveRightFingers;

        if(length == 37){
            // contactgloves2 data
            // indexroot: 10, 12
            // indextip: 11
            // middletip 8
            // middleroot 7, 9
            // ringroot: 4,6
            // ringtip: 5
            // pinkyroot: 1,3
            // pinkytip: 2
            // thumbroot: 16
            // thumbtip: 14
            // thumb is 13-16
            // 17 is unknown
            //for(size_t i = 0; i < 18; i++){
            //    printf("\t[%d]: %d\n", i, ((uint16_t*)(pData + 1))[i]);
            //}
            GlovePacketFingers2_t* fingers = (GlovePacketFingers2_t*)(pData + 1);
            outPacket->packet.gloveFingers = *fingers;
        } else {
            // Extract data
            outPacket->packet.gloveFingers.fingerThumbTip   = ((uint16_t*)(pData + 1))[9];
            outPacket->packet.gloveFingers.fingerThumbRoot  = ((uint16_t*)(pData + 1))[8];
            outPacket->packet.gloveFingers.fingerIndexTip   = ((uint16_t*)(pData + 1))[7];
            outPacket->packet.gloveFingers.fingerIndexRoot1  = ((uint16_t*)(pData + 1))[6];
            outPacket->packet.gloveFingers.fingerMiddleTip  = ((uint16_t*)(pData + 1))[5];
            outPacket->packet.gloveFingers.fingerMiddleRoot1 = ((uint16_t*)(pData + 1))[4];
            outPacket->packet.gloveFingers.fingerRingTip    = ((uint16_t*)(pData + 1))[3];
            outPacket->packet.gloveFingers.fingerRingRoot1   = ((uint16_t*)(pData + 1))[2];
            outPacket->packet.gloveFingers.fingerPinkyTip   = ((uint16_t*)(pData + 1))[0];
            outPacket->packet.gloveFingers.fingerPinkyRoot1  = ((uint16_t*)(pData + 1))[1];
        }
        // printf(
        //     "fingers[R]:: (%d, %d) (%d, %d) (%d, %d) (%d, %d) (%d, %d)\n",
        //     outPacket->packet.gloveFingers.fingerThumbTip,
        //     outPacket->packet.gloveFingers.fingerThumbRoot,
        //     outPacket->packet.gloveFingers.fingerIndexTip,
        //     outPacket->packet.gloveFingers.fingerIndexRoot,
        //     outPacket->packet.gloveFingers.fingerMiddleTip,
        //     outPacket->packet.gloveFingers.fingerMiddleRoot,
        //     outPacket->packet.gloveFingers.fingerRingTip,
        //     outPacket->packet.gloveFingers.fingerRingRoot,
        //     outPacket->packet.gloveFingers.fingerPinkyTip,
        //     outPacket->packet.gloveFingers.fingerPinkyRoot);

        // PrintBuffer("glove_right_fingers", pData, length);

        decoded = true;

    }
    break;
    case GLOVE_RIGHT_PACKET_IMU:
    {
        // @TODO: Imu data is not usable

        // Assign type
        outPacket->type = PacketType_t::GloveRightImu;
        // Extract data
        outPacket->packet.gloveImu.imu1 = ((uint16_t*)(pData + 1))[0];
        outPacket->packet.gloveImu.imu2 = ((uint16_t*)(pData + 1))[1];

        outPacket->packet.gloveImu.imu3 = ((uint16_t*)(pData + 1))[2];
        outPacket->packet.gloveImu.imu4 = ((uint16_t*)(pData + 1))[3];

        outPacket->packet.gloveImu.imu5 = ((float*)(pData + 1))[0];
        outPacket->packet.gloveImu.imu6 = ((float*)(pData + 1))[1];

        // printf(
        //     "imu[R]:: %d, %d, %d, %d [%.6f, %.6f]\n",
        //     outPacket->packet.gloveImu.imu1,
        //     outPacket->packet.gloveImu.imu2,
        //     outPacket->packet.gloveImu.imu3,
        //     outPacket->packet.gloveImu.imu4,
        //     outPacket->packet.gloveImu.imu5,
        //     outPacket->packet.gloveImu.imu6);

        // PrintBuffer("glove_right_imu", pData, length);
        decoded = true;

    }

    break;

    default:
        // @FIXME: Use proper logging library
        //printf("[WARN] Got unknown packet with command code 0x%02hX!!\n", pData[0]);
        PrintBuffer("unknown_packet", pData, length);
        break;
    }

    return decoded;
}

/*
WRITE FILE:
```
02 01 01 02 f0 01 03 f0 02 00           | 01 00 00 F0 00 00 F0 02
05 18 01 01 e1 00                       | 18 01 01 E1
05 18 02 01 de 00                       | 18 02 01 DE
05 18 01 0b d7 00                       | 18 01 0B D7
05 18 02 0b e8 00                       | 18 02 0B E8
05 16 01 04 03 01 03 ff 70 00           | 16 01 04 03 00 00 FF 70
05 16 02 04 03 01 03 ff 0b 00           | 16 02 04 03 00 00 FF 0B
03 0f 32 02 64 03 ff 9f 00              | 0F 32 00 64 00 FF 9F
```
*/