/*******************************************************************************
 * 
 *      File:    Driver.cpp
 *      
 *******************************************************************************
 *
 *      Target product: escDriver_to_MQTT
 *      Purpose: Core driver implementation
 *
 *******************************************************************************
 */

#include "Driver.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <unistd.h>
#include <ctime>

namespace {
const char* DataPointName(uint16 id)
{
    switch (id) {
        case 0x1001: return "Version";
        case 0x1010: return "Fault data";
        case 0x1020: return "Operation status";
        case 0x1021: return "Speed";
        case 0x1022: return "Brake distance";
        case 0x1023: return "WBS";
        case 0x1080: return "Power on time";
        case 0x1081: return "Breakdown time";
        case 0x1082: return "Operation time up";
        case 0x1083: return "Operation time down";
        case 0x1084: return "Inspection time";
        case 0x10A0: return "Running up";
        case 0x10A1: return "Running down";
        case 0x10A2: return "Number of faults";
        case 0x10A3: return "Emergency stops";
        case 0x0020: return "Operation status (OVT 401)";
        case 0x0080: return "Power on time (OVT 401)";
        default: return "Unknown data point";
    }
}

const char* DataPointUnit(uint16 id)
{
    switch (id) {
        case 0x1021: return "m/sec";
        case 0x1022: return "mm";
        case 0x1080: case 0x1081: case 0x1082: case 0x1083: case 0x1084: return "s";
        case 0x0080: return "min";
        default: return "";
    }
}

uint32 DataPointScale(uint16 id)
{
    if (id == 0x1021) return 1000;
    if (id == 0x1001) return 100;
    return 1;
}

void PrintDataPoint(const DataPointReply& point)
{
    std::cout << "  " << DataPointName(point.id) << " (0x"
              << std::hex << std::uppercase << std::setw(4) << std::setfill('0')
              << point.id << "): " << std::dec << std::setfill(' ');

    if (point.type == 14) {
        if (point.raw.size() <= 1)
            std::cout << "<empty payload>";
        for (size_t i = 1; i < point.raw.size(); ++i)
            std::cout << " " << std::hex << std::uppercase << std::setw(2)
                      << std::setfill('0') << static_cast<int>(point.raw[i]);
        std::cout << std::dec << std::setfill(' ');
    } else {
        uint64_t value = 0;
        for (size_t i = 0; i < point.raw.size(); ++i)
            value = (value << 8) | point.raw[i];

        const uint32 scale = DataPointScale(point.id);
        if (scale == 1000)
            std::cout << static_cast<double>(value) / scale;
        else if (scale == 100)
            std::cout << static_cast<double>(value) / scale;
        else
            std::cout << value;

        const char* unit = DataPointUnit(point.id);
        if (unit[0] != '\0')
            std::cout << " " << unit;
    }

    if (!point.acknowledged)
        std::cout << " (not acknowledged)";
    std::cout << std::endl;
}
}

Driver::Driver() 
    : state_(NVLD_CST), 
      driverIndex_(0), 
      debugLevel_(0), 
      running_(false),
      pollIntervalMs_(500),
      pollCount_(0)
{
    if (debugLevel_ > 0)
        std::cout << "[Driver] Constructor called" << std::endl;
}

Driver::~Driver()
{
    if (debugLevel_ > 0)
        std::cout << "[Driver] Destructor called" << std::endl;
}

bool Driver::Setup(
    const uint32 driverIndex,
    const uint32 debugLevel,
    const UIntData_t &addresses)
{
    if (debugLevel_ > 0)
        std::cout << "[Driver::Setup] Starting setup..." << std::endl;

    driverIndex_ = driverIndex;
    debugLevel_ = debugLevel;
    addresses_ = addresses;

    messageCodec_.SetDebugLevel(debugLevel_);
    serialComm_.SetDebugLevel(debugLevel_);

    if (debugLevel_ > 0) {
        std::cout << "[Driver::Setup] Driver Index: " << driverIndex_ << std::endl;
        std::cout << "[Driver::Setup] Debug Level: " << debugLevel_ << std::endl;
        std::cout << "[Driver::Setup] Number of addresses: " << addresses_.size() << std::endl;
        for (size_t i = 0; i < addresses_.size(); ++i) {
            std::cout << "  Address[" << i << "]: " << addresses_[i] << std::endl;
        }
    }

    if (addresses_.empty()) {
        addresses_.push_back(1);
        std::cout << "[Driver::Setup] No address specified, polling address 01" << std::endl;
    }

    for (size_t i = 0; i < addresses_.size(); ++i) {
        if (addresses_[i] > 255) {
            std::cerr << "[Driver::Setup] ERROR: Address must be between 0 and 255: "
                      << addresses_[i] << std::endl;
            return false;
        }
    }

    const std::string portName = "/dev/ttyUSB0";
    if (!serialComm_.OpenPort(portName, 4800)) {
        std::cerr << "[Driver::Setup] ERROR: Failed to open serial port" << std::endl;
        return false;
    }

    for (size_t i = 0; i < addresses_.size(); ++i) {
        EscalatorStatus status;
        status.SetID(addresses_[i]);
        serialComm_.RegisterEscalator(static_cast<uint8>(addresses_[i]), status);
    }

    // Initialization successful
    state_ = RUNNING_CST;
    running_ = true;

    std::cout << "[Driver::Setup] Setup completed successfully" << std::endl;
    return true;
}

bool Driver::Start()
{
    if (debugLevel_ > 0)
        std::cout << "[Driver::Start] Starting driver main loop..." << std::endl;

    if (state_ != RUNNING_CST) {
        std::cerr << "[Driver::Start] ERROR: Driver not in RUNNING state" << std::endl;
        return false;
    }

    running_ = true;
    std::cout << "[Driver::Start] Driver started. Reading escalator data..." << std::endl;
    return true;
}

void Driver::Stop()
{
    if (debugLevel_ > 0)
        std::cout << "[Driver::Stop] Stopping driver..." << std::endl;

    if (state_ != ABORT_REQUESTED_CST && state_ != ABORTED_CST) {
        state_ = ABORT_REQUESTED_CST;
    }
    running_ = false;
    serialComm_.ClosePort();

    std::cout << "[Driver::Stop] Driver stopped" << std::endl;
}

bool Driver::IsStopping() const
{
    return (state_ == ABORT_REQUESTED_CST || state_ == ABORTED_CST || !running_);
}

bool Driver::GetEscalatorStatus(uint8 address, EscalatorStatus& status) const
{
    auto it = escalatorData_.find(address);
    if (it != escalatorData_.end()) {
        status = it->second;
        return true;
    }
    return false;
}

void Driver::ProcessData()
{
    if (debugLevel_ > 2)
        std::cout << "[Driver::ProcessData] Processing escalator data..." << std::endl;

    // Get next escalator address (round-robin)
    const auto& escalators = serialComm_.GetEscalators();
    if (escalators.empty()) {
        return;
    }

    auto it = escalators.begin();
    std::advance(it, pollCount_ % escalators.size());
    const uint8 address = it->first;
    static const std::pair<uint16, uint8> queryParameters[] = {
        { 0x1021, 1 }, { 0x1080, 2 }, { 0x1081, 2 }, { 0x10A2, 2 },
        { 0x1084, 2 }, { 0x1082, 2 }, { 0x1083, 2 }, { 0x10A0, 2 },
        { 0x10A1, 2 }, { 0x10A3, 2 }, { 0x1022, 1 }, { 0x1023, 1 },
        { 0x1020, 2 }
    };
    const size_t queryIndex = pollCount_ % (sizeof(queryParameters) / sizeof(queryParameters[0]));
    const uint16 queryId = queryParameters[queryIndex].first;
    const uint8 queryType = queryParameters[queryIndex].second;

    auto sendAndReceive = [&](const std::vector<uint8>& request, const char* phase) {
        if (!serialComm_.SendMessage(address, request)) {
            std::cerr << "[Driver::ProcessData] ERROR: Failed to send " << phase
                      << " to address " << static_cast<int>(address) << std::endl;
            return false;
        }

        std::vector<uint8> responseMsg;
        uint8 responseAddress = address;
        if (!serialComm_.ReceiveMessage(responseAddress, responseMsg)) {
            if (debugLevel_ > 0)
                std::cerr << "[Driver::ProcessData] No response to " << phase
                          << " from address " << static_cast<int>(address) << std::endl;
            return false;
        }

        usleep(150000);//150 mSec

        bool validReply = false;
        bool dummyReply = false;
        uint8 decodedAddress = 0;
        std::vector<DataPointReply> values;
        if (!responseMsg.empty() && responseMsg[0] == 0xE5) {
            validReply = true;
            dummyReply = true;
            decodedAddress = address;
        } else if (!responseMsg.empty() && responseMsg.back() == 0x16 && responseMsg[0] == 0x10) {
            bool slaveLocked = false;
            bool writeSucceeded = false;
            bool eventDataAvailable = false;
            validReply = messageCodec_.DecodeShortReply(responseMsg, decodedAddress,
                slaveLocked, writeSucceeded, eventDataAvailable);
        } else if (!responseMsg.empty() && responseMsg[0] == 0x68) {
            validReply = messageCodec_.DecodeLongReply(responseMsg, decodedAddress, values);
        }

        if (!validReply || decodedAddress != address) {
            if (debugLevel_ > 1)
                std::cerr << "[Driver::ProcessData] Invalid or unexpected response to "
                          << phase << " from address " << static_cast<int>(responseAddress)
                          << std::endl;
            return false;
        }

        EscalatorStatus status;
        auto existing = escalatorData_.find(address);
        if (existing != escalatorData_.end())
            status = existing->second;
        status.SetID(address);
        status.SetConnected(true);
        status.UpdateTimestamp();
        escalatorData_[address] = status;

        if (debugLevel_ > 1) {
            std::cout << "[Driver::ProcessData] "
                      << (dummyReply ? "ACK (E5)" : "Valid response")
                      << " to " << phase << " from address "
                      << static_cast<int>(address) << std::endl;
        }
        if (debugLevel_ > 0) {
            for (size_t i = 0; i < values.size(); ++i)
                PrintDataPoint(values[i]);
        }
        return true;
    };

    std::vector<uint8> request;

    // Match the reference sequence: Poll A, Poll B, query, Poll B, Poll A.
    messageCodec_.SetToggleBit(0);
    messageCodec_.EncodePollMsg(address, request);
    sendAndReceive(request, "Poll A");

    messageCodec_.SetToggleBit(1);
    messageCodec_.EncodePollMsg(address, request);
    sendAndReceive(request, "Poll B");

    messageCodec_.SetToggleBit(0);
    std::vector<std::pair<uint16, uint8>> requestedPoint(1,
        std::make_pair(queryId, queryType));
    if (messageCodec_.EncodeQueryMsg(address, requestedPoint, request))
        sendAndReceive(request, DataPointName(queryId));
    else
        std::cerr << "[Driver::ProcessData] ERROR: Failed to encode data point query"
                  << std::endl;

    messageCodec_.SetToggleBit(1);
    messageCodec_.EncodePollMsg(address, request);
    sendAndReceive(request, "Poll B");

    messageCodec_.SetToggleBit(0);
    messageCodec_.EncodePollMsg(address, request);
    sendAndReceive(request, "Poll A");

    pollCount_++;
    usleep(pollIntervalMs_ * 1000);
}

