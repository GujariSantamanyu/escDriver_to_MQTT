/*******************************************************************************
 * 
 *      File:    SimpleSerialComm.h
 *      
 *******************************************************************************
 *
 *      Target product: escDriver_to_MQTT
 *      Purpose: Simple serial communication interface for RS-485
 *
 *******************************************************************************
 */

#ifndef SIMPLE_SERIAL_COMM_H
#define SIMPLE_SERIAL_COMM_H

#include "Types.h"
#include "EscalatorStatus.h"
#include <vector>
#include <map>
#include <iostream>
#include <iomanip>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

class SimpleSerialComm
{
public:
    SimpleSerialComm() : portFd_(-1), debugLevel_(0), pollCount_(0) { }

    virtual ~SimpleSerialComm() { ClosePort(); }

    /// Open and configure the serial port for raw 8E1 communication.
    bool OpenPort(const std::string& portName, uint32 baudRate)
    {
        if (debugLevel_ > 0) {
            std::cout << "[SimpleSerialComm] Opening port: " << portName 
                      << " at " << baudRate << " baud" << std::endl;
        }

        ClosePort();
        portFd_ = open(portName.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (portFd_ < 0) {
            std::cerr << "[SimpleSerialComm] ERROR: Cannot open " << portName
                      << ": " << std::strerror(errno) << std::endl;
            return false;
        }

        termios settings;
        if (tcgetattr(portFd_, &settings) != 0) {
            std::cerr << "[SimpleSerialComm] ERROR: Cannot read serial settings: "
                      << std::strerror(errno) << std::endl;
            ClosePort();
            return false;
        }

        speed_t speed;
        if (!GetBaudConstant(baudRate, speed)) {
            std::cerr << "[SimpleSerialComm] ERROR: Unsupported baud rate: "
                      << baudRate << std::endl;
            ClosePort();
            return false;
        }

        cfmakeraw(&settings);
        settings.c_cflag &= ~(CSIZE | PARODD | CSTOPB);
        settings.c_cflag |= CS8 | CLOCAL | CREAD | PARENB;
#ifdef CRTSCTS
        settings.c_cflag &= ~CRTSCTS;
#endif
        settings.c_iflag &= ~(IXON | IXOFF | IXANY);
        settings.c_iflag |= INPCK;
        settings.c_cc[VMIN] = 0;
        settings.c_cc[VTIME] = 0;
        if (cfsetispeed(&settings, speed) != 0 ||
            cfsetospeed(&settings, speed) != 0 ||
            tcsetattr(portFd_, TCSANOW, &settings) != 0) {
            std::cerr << "[SimpleSerialComm] ERROR: Cannot configure serial port: "
                      << std::strerror(errno) << std::endl;
            ClosePort();
            return false;
        }

        tcflush(portFd_, TCIOFLUSH);
        portName_ = portName;

        return true;
    }

    /// Close serial port
    void ClosePort()
    {
        if (portFd_ >= 0) {
            if (debugLevel_ > 0)
                std::cout << "[SimpleSerialComm] Closing port" << std::endl;
            close(portFd_);
            portFd_ = -1;
        }
    }

    /// Send a message to escalator
    bool SendMessage(uint8 address, const std::vector<uint8>& message)
    {
        if (portFd_ < 0) {
            std::cerr << "[SimpleSerialComm] ERROR: Port not open" << std::endl;
            return false;
        }
        if (message.empty())
            return false;

        if (debugLevel_ > 1) {
            std::cout << "[SimpleSerialComm] Sending message to address " << (int)address;
            std::cout << " (" << message.size() << " bytes)" << std::endl;
        }

        size_t sent = 0;
        while (sent < message.size()) {
            pollfd descriptor = { portFd_, POLLOUT, 0 };
            int ready = poll(&descriptor, 1, 1000);
            if (ready <= 0 || (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)))
                return false;

            ssize_t result = write(portFd_, &message[sent], message.size() - sent);
            if (result > 0) {
                sent += static_cast<size_t>(result);
            } else if (result < 0 && errno != EINTR && errno != EAGAIN) {
                std::cerr << "[SimpleSerialComm] ERROR: Serial write failed: "
                          << std::strerror(errno) << std::endl;
                return false;
            }
        }
        return true;
    }

    /// Receive one framed short or long protocol message.
    bool ReceiveMessage(uint8& address, std::vector<uint8>& message, uint32 timeoutMs = 1000)
    {
        if (portFd_ < 0) {
            std::cerr << "[SimpleSerialComm] ERROR: Port not open" << std::endl;
            return false;
        }

        message.clear();
        const auto deadline = std::chrono::steady_clock::now() +
            std::chrono::milliseconds(timeoutMs);
        uint8 header = 0;
        do {
            if (!ReadByte(header, deadline)) {
                if (debugLevel_ > 0)
                    std::cerr << "[SimpleSerialComm] RX timeout waiting for frame header"
                              << std::endl;
                return false;
            }
        } while (header != 0x10 && header != 0x68 && header != 0xE5);

        const size_t headerSize = header == 0x10 ? 5 : (header == 0x68 ? 4 : 1);
        message.push_back(header);
        while (message.size() < headerSize) {
            uint8 value;
            if (!ReadByte(value, deadline)) {
                if (debugLevel_ > 0)
                    std::cerr << "[SimpleSerialComm] RX timeout waiting for frame body"
                              << std::endl;
                message.clear();
                return false;
            }
            message.push_back(value);
        }

        if (header == 0x68) {
            const size_t frameSize = static_cast<size_t>(message[1]) + 6;
            if (frameSize < 8) {
                message.clear();
                return false;
            }
            while (message.size() < frameSize) {
                uint8 value;
                if (!ReadByte(value, deadline)) {
                    if (debugLevel_ > 0)
                        std::cerr << "[SimpleSerialComm] RX timeout waiting for long frame body"
                                  << std::endl;
                    message.clear();
                    return false;
                }
                message.push_back(value);
            }
        }

        if (header == 0x10)
            address = message[1];
        else if (header == 0x68)
            address = message[4];
        ++pollCount_;

        if (debugLevel_ > 1) {
            std::cout << "[SimpleSerialComm] Received frame from address " << (int)address
                      << " (" << message.size() << " bytes):";
            for (size_t i = 0; i < message.size(); ++i)
                std::cout << " " << std::hex << std::uppercase << std::setw(2)
                          << std::setfill('0') << static_cast<int>(message[i]);
            std::cout << std::dec << std::setfill(' ') << std::endl;
        }

        return true;
    }

    /// Register an escalator at given address
    bool RegisterEscalator(uint8 address, const EscalatorStatus& status)
    {
        escalators_[address] = status;
        escalators_[address].SetID(address);
        
        if (debugLevel_ > 0) {
            std::cout << "[SimpleSerialComm] Registered escalator at address " << (int)address << std::endl;
        }
        return true;
    }

    /// Get escalator status
    bool GetEscalatorStatus(uint8 address, EscalatorStatus& status)
    {
        auto it = escalators_.find(address);
        if (it != escalators_.end()) {
            status = it->second;
            return true;
        }
        return false;
    }

    /// Update escalator status (simulates data changes)
    bool UpdateEscalatorStatus(uint8 address, const EscalatorStatus& status)
    {
        auto it = escalators_.find(address);
        if (it != escalators_.end()) {
            it->second = status;
            it->second.UpdateTimestamp();
            return true;
        }
        return false;
    }

    /// Get all registered escalators
    const std::map<uint8, EscalatorStatus>& GetEscalators() const
    {
        return escalators_;
    }

    void SetDebugLevel(uint32 level) { debugLevel_ = level; }

    uint32 GetPollCount() const { return pollCount_; }

private:
    static bool GetBaudConstant(uint32 baudRate, speed_t& speed)
    {
        switch (baudRate) {
            case 4800: speed = B4800; return true;
            case 9600: speed = B9600; return true;
            case 19200: speed = B19200; return true;
            case 38400: speed = B38400; return true;
            default: return false;
        }
    }

    bool ReadByte(uint8& value, std::chrono::steady_clock::time_point deadline)
    {
        while (true) {
            auto remaining = std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count();
            if (remaining <= 0)
                return false;

            pollfd descriptor = { portFd_, POLLIN, 0 };
            int ready = poll(&descriptor, 1, static_cast<int>(remaining));
            if (ready == 0)
                return false;
            if (ready < 0) {
                if (errno == EINTR)
                    continue;
                return false;
            }
            if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
                return false;

            ssize_t result = read(portFd_, &value, 1);
            if (result == 1) {
                if (debugLevel_ > 2)
                    std::cout << "[SimpleSerialComm] RX byte: 0x" << std::hex
                              << std::uppercase << std::setw(2) << std::setfill('0')
                              << static_cast<int>(value) << std::dec << std::setfill(' ')
                              << std::endl;
                return true;
            }
            if (result < 0 && errno != EINTR && errno != EAGAIN)
                return false;
        }
    }

    int portFd_;
    std::string portName_;
    uint32 debugLevel_;
    uint32 pollCount_;
    std::map<uint8, EscalatorStatus> escalators_;
};

#endif // SIMPLE_SERIAL_COMM_H
