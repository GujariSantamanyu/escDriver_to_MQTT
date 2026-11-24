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
#include "DataPoint.h"
#include "MQTTPayload.h"
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <limits>
#include <stdexcept>
#include <unistd.h>
#include <ctime>

namespace {
const char* SerialConfigPath = "/home/sensorlab/Desktop/esc-config/EscRS0.ini";

struct SerialConfig {
    std::string portName;
    uint32 baudRate;
    uint32 dataBits;
    uint32 parity;
    uint32 stopBits;
};

std::string Trim(std::string value)
{
    const size_t first = value.find_first_not_of(" \t\r\n");
    if (first == std::string::npos)
        return std::string();
    const size_t last = value.find_last_not_of(" \t\r\n");
    return value.substr(first, last - first + 1);
}

bool ParseUint32(const std::string& text, uint32& value)
{
    try {
        size_t end = 0;
        const unsigned long parsed = std::stoul(text, &end);
        if (end != text.size() || parsed > std::numeric_limits<uint32>::max())
            return false;
        value = static_cast<uint32>(parsed);
        return true;
    } catch (const std::invalid_argument&) {
        return false;
    } catch (const std::out_of_range&) {
        return false;
    }
}

bool ReadSerialConfig(SerialConfig& config)
{
    std::ifstream file(SerialConfigPath);
    if (!file) {
        std::cerr << "[Driver::Setup] ERROR: Cannot open serial configuration: "
                  << SerialConfigPath << std::endl;
        return false;
    }

    std::string line;
    const std::string key = "RSParams=";
    while (std::getline(file, line)) {
        line = Trim(line);
        if (line.compare(0, key.size(), key) != 0)
            continue;

        std::stringstream values(line.substr(key.size()));
        std::string field;
        std::vector<std::string> fields;
        while (std::getline(values, field, ','))
            fields.push_back(Trim(field));

        if (fields.size() != 5 ||
            !ParseUint32(fields[1], config.baudRate) ||
            !ParseUint32(fields[2], config.dataBits) ||
            !ParseUint32(fields[3], config.parity) ||
            !ParseUint32(fields[4], config.stopBits) ||
            fields[0].empty()) {
            std::cerr << "[Driver::Setup] ERROR: Invalid RSParams in "
                      << SerialConfigPath << std::endl;
            return false;
        }

        config.portName = fields[0];
        return true;
    }

    std::cerr << "[Driver::Setup] ERROR: RSParams not found in "
              << SerialConfigPath << std::endl;
    return false;
}

const char* DataPointName(uint16 id)
{
    switch (id) {
        case ECO3000_DP_VERSION: return "Version";
        case ECO3000_DP_FAULT_STATUS: return "Fault data";
        case ECO3000_DP_OPER_STATUS: return "Operation status";
        case ECO3000_DP_SPEED: return "Speed";
        case ECO3000_DP_BRAKE_DISTANCE: return "Brake distance";
        case ECO3000_DP_WBS: return "WBS";
        case ECO3000_DP_POWER_ON_TIME: return "Power on time";
        case ECO3000_DP_BREAK_DOWN_TIME: return "Breakdown time";
        case ECO3000_DP_OPERTIME_UP: return "Operation time up";
        case ECO3000_DP_OPERTIME_DOWN: return "Operation time down";
        case ECO3000_DP_INSPECTION_TIME: return "Inspection time";
        case ECO3000_DP_RUNS_UP: return "Running up";
        case ECO3000_DP_RUNS_DOWN: return "Running down";
        case ECO3000_DP_FAULT_COUNT: return "Number of faults";
        case ECO3000_DP_EMERGENCY_STOPS: return "Emergency stops";
        case 0x0020: return "Operation status (OVT 401)";
        case 0x0080: return "Power on time (OVT 401)";
        default: return "Unknown data point";
    }
}

const char* DataPointUnit(uint16 id)
{
    switch (id) {
        case ECO3000_DP_SPEED: return "m/sec";
        case ECO3000_DP_BRAKE_DISTANCE: return "mm";
        case ECO3000_DP_POWER_ON_TIME:
        case ECO3000_DP_BREAK_DOWN_TIME:
        case ECO3000_DP_OPERTIME_UP:
        case ECO3000_DP_OPERTIME_DOWN:
        case ECO3000_DP_INSPECTION_TIME: return "s";
        case 0x0080: return "min";
        default: return "";
    }
}

uint32 DataPointScale(uint16 id)
{
    if (id == ECO3000_DP_SPEED) return 1000;
    if (id == ECO3000_DP_VERSION) return 100;
    return 1;
}

void PrintOperationStatus(uint32 value)
{
    static const char* const bitNames[] = {
        "direction up", "direction down", "running", "ready to run",
        nullptr, nullptr, nullptr, nullptr,
        "inverter operation", "line operation", "standby speed", nullptr,
        nullptr, "emergency stop", "common fault", "shutdown fault",
        "up", "down", "running", "not ready", "start mode",
        "construct mode", "inspect mode"
    };

    std::cout << "    (raw 0x" << std::hex << std::uppercase << std::setw(8)
              << std::setfill('0') << value << std::dec << std::setfill(' ')
              << ")" << std::endl;

    for (uint32 bit = 0; bit < sizeof(bitNames) / sizeof(bitNames[0]); ++bit) {
        if (bitNames[bit] != nullptr && (value & (uint32(1) << bit)) != 0)
            std::cout << "    - " << bitNames[bit] << std::endl;
    }

    const char* modeName = "unknown";
    switch ((value >> 4) & 0x0F) {
        case 0: modeName = "no operation mode"; break;
        case 1: modeName = "automatic mode"; break;
        case 2: modeName = "continuous mode"; break;
        case 3: modeName = "2 direction mode"; break;
        case 4: modeName = "energy saving mode (standby speed)"; break;
        case 15: modeName = "inspection mode"; break;
    }
    std::cout << "    - operation mode = " << modeName << std::endl;
    std::cout << "    - version = "
              << ((value & (uint32(1) << 31)) ? "ANSI" : "EN") << std::endl;
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

    if (point.id == ECO3000_DP_OPER_STATUS && point.type == 2 && point.raw.size() == 4) {
        uint32 value = 0;
        for (size_t i = 0; i < point.raw.size(); ++i)
            value = (value << 8) | point.raw[i];
        PrintOperationStatus(value);
    }
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

    SerialConfig serialConfig;
    if (!ReadSerialConfig(serialConfig)) {
        return false;
    }
    if (!serialComm_.OpenPort(
            serialConfig.portName,
            serialConfig.baudRate,
            serialConfig.dataBits,
            serialConfig.parity,
            serialConfig.stopBits)) {
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
    static const std::vector<std::pair<uint16, uint8>> queryParameters = {
        { ECO3000_DP_FAULT_STATUS, 14 },
        { ECO3000_DP_OPER_STATUS, 2 },
        { ECO3000_DP_SPEED, 1 },
        { ECO3000_DP_BRAKE_DISTANCE, 1 }
    };

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
        if (!values.empty()) {
            std::cout << "[MQTT] Payload: "
                      << MQTTPayload::Build(std::to_string(address), values)
                      << std::endl;
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
    if (messageCodec_.EncodeQueryMsg(address, queryParameters, request))
        sendAndReceive(request, "Multi-point query");
    else
        std::cerr << "[Driver::ProcessData] ERROR: Failed to encode multi-point query"
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

