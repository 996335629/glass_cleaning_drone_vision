#include "communication/uart_interface.h"
#include <iostream>

#ifdef _WIN32
#include <windows.h>
#else
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstring>
#endif

class UARTInterface::Impl {
public:
#ifdef _WIN32
    HANDLE handle = INVALID_HANDLE_VALUE;
#else
    int fd = -1;
#endif
    Config config;
    bool is_open = false;
};

UARTInterface::UARTInterface() : impl_(std::make_unique<Impl>()) {}

UARTInterface::~UARTInterface()
{
    close();
}

bool UARTInterface::open(const Config &config)
{
    impl_->config = config;

#ifdef _WIN32
    std::string port_path = "\\\\.\\" + config.port_name;
    impl_->handle = CreateFileA(
        port_path.c_str(),
        GENERIC_READ | GENERIC_WRITE,
        0, nullptr, OPEN_EXISTING, 0, nullptr
    );

    if (impl_->handle == INVALID_HANDLE_VALUE) {
        std::cerr << "[UART] Failed to open: " << config.port_name << std::endl;
        return false;
    }

    DCB dcb = { 0 };
    dcb.DCBlength = sizeof(DCB);
    GetCommState(impl_->handle, &dcb);

    dcb.BaudRate = static_cast<DWORD>(config.baud_rate);
    dcb.ByteSize = 8;
    dcb.StopBits = ONESTOPBIT;
    dcb.Parity = NOPARITY;

    SetCommState(impl_->handle, &dcb);

    COMMTIMEOUTS timeouts = { 0 };
    timeouts.ReadIntervalTimeout = 50;
    timeouts.ReadTotalTimeoutConstant = config.timeout_ms;
    timeouts.ReadTotalTimeoutMultiplier = 10;
    SetCommTimeouts(impl_->handle, &timeouts);
#else
    impl_->fd = ::open(config.port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (impl_->fd < 0) {
        std::cerr << "[UART] Failed to open: " << config.port_name << std::endl;
        return false;
    }

    struct termios tty;
    memset(&tty, 0, sizeof(tty));
    tcgetattr(impl_->fd, &tty);

    speed_t baud = B115200;
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);

    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |= CS8;
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;

    tcsetattr(impl_->fd, TCSANOW, &tty);
#endif

    impl_->is_open = true;
    std::cout << "[UART] Opened: " << config.port_name << std::endl;
    return true;
}

void UARTInterface::close()
{
    if (!impl_->is_open) return;

#ifdef _WIN32
    if (impl_->handle != INVALID_HANDLE_VALUE) {
        CloseHandle(impl_->handle);
        impl_->handle = INVALID_HANDLE_VALUE;
    }
#else
    if (impl_->fd >= 0) {
        ::close(impl_->fd);
        impl_->fd = -1;
    }
#endif

    impl_->is_open = false;
}

bool UARTInterface::isOpen() const
{
    return impl_->is_open;
}

int UARTInterface::send(const uint8_t *data, size_t length)
{
    if (!impl_->is_open) return -1;

#ifdef _WIN32
    DWORD written = 0;
    WriteFile(impl_->handle, data, static_cast<DWORD>(length), &written, nullptr);
    return static_cast<int>(written);
#else
    return static_cast<int>(write(impl_->fd, data, length));
#endif
}

int UARTInterface::receive(uint8_t *buffer, size_t max_length)
{
    if (!impl_->is_open) return -1;

#ifdef _WIN32
    DWORD read_bytes = 0;
    ReadFile(impl_->handle, buffer, static_cast<DWORD>(max_length), &read_bytes, nullptr);
    return static_cast<int>(read_bytes);
#else
    return static_cast<int>(read(impl_->fd, buffer, max_length));
#endif
}

void UARTInterface::flushReceive()
{
#ifdef _WIN32
    PurgeComm(impl_->handle, PURGE_RXCLEAR);
#else
    tcflush(impl_->fd, TCIFLUSH);
#endif
}

void UARTInterface::flushSend()
{
#ifdef _WIN32
    PurgeComm(impl_->handle, PURGE_TXCLEAR);
#else
    tcflush(impl_->fd, TCOFLUSH);
#endif
}
