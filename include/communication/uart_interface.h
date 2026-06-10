#pragma once

#include <string>
#include <memory>
#include <cstdint>

class UARTInterface {
public:
    enum class BaudRate {
        BAUD_9600 = 9600,
        BAUD_115200 = 115200,
        BAUD_460800 = 460800
    };

    struct Config {
        std::string port_name;
        BaudRate baud_rate = BaudRate::BAUD_115200;
        int timeout_ms = 1000;
    };

    UARTInterface();
    ~UARTInterface();

    bool open(const Config &config);
    void close();
    bool isOpen() const;

    int send(const uint8_t *data, size_t length);
    int receive(uint8_t *buffer, size_t max_length);
    void flushReceive();
    void flushSend();

private:
    class Impl;
    std::unique_ptr<Impl> impl_;
};
