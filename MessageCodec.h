/*******************************************************************************
 * 
 *      File:    MessageCodec.h
 *      
 *******************************************************************************
 *
 *      Target product: escDriver_to_MQTT
 *      Purpose: RS-485 message encoding/decoding based on actual escDriver protocol
 *      Source: /NGMO/escDriver/TransportMsgWrapper.cpp
 *
 *******************************************************************************
 */

#ifndef MESSAGE_CODEC_H
#define MESSAGE_CODEC_H

#include "Types.h"
#include <vector>
#include <iostream>
#include <iomanip>

struct DataPointReply
{
    uint16 id;
    uint8 control;
    uint8 type;
    bool acknowledged;
    std::vector<uint8> raw;
};

class MessageCodec
{
public:
    // Message type constants (from MsgCodes.h)
    static const uint8 TLM_DUMMY_MSG_ID = 0xE5;
    static const uint8 TLM_SHORT_MSG_ID = 0x10;
    static const uint8 TLM_LONG_MSG_ID = 0x68;
    static const uint8 TLM_MSG_ENDMARK = 0x16;
    static const uint8 TLM_RS486_DUMMY_CHAR = 0xE0;

    // Sizes
    static const uint32 TLM_SHORT_MSG_SIZE = 5;
    static const uint32 TLM_LONG_SKELETON_SIZE = 8;
    static const uint32 TLM_LONG_DATA_MAX_SIZE = 253;

    // Bit Shifts
    static const uint8 TLM_REQ_RESP_SHIFT = 0;
    static const uint8 TLM_SELECT_SHIFT = 3;
    static const uint8 TLM_SEQ_SHIFT = 4;
    static const uint8 TLM_CTRL_SHIFT = 5;
    static const uint8 TLM_LOCKED_SHIFT = 3;
    static const uint8 TLM_ACK_SHIFT = 4;
    static const uint8 TLM_DA_SHIFT = 7;

    // Magic Values
    static const uint8 TLM_CTRL_READ = 5;
    static const uint8 TLM_CTRL_WRITE = 3;
    static const uint32 TLM_LONG_MSG_EXTRA_MAGIC = 2;
    static const uint32 TLM_LONG_DATA_START_INDEX = 6;

    // Checksum
    static const uint32 TLM_SHORT_CS_START = 1;
    static const uint32 TLM_SHORT_CS_LEN = 2;
    static const uint32 TLM_LONG_CS_START = 4;

    // Application Layer constants
    static const uint32 ALM_DP_QUERY_SIZE = 3;
    static const uint32 ALM_DP_MIN_REPLY_SIZE = 3;
    static const uint8 ALM_CTRL_READ_SHIFT = 6;
    static const uint8 ALM_CTRL_WRITE_SHIFT = 7;
    static const uint8 ALM_CTRL_ACK_SHIFT = 5;
    static const uint8 ALM_CTRL_TYPE_MSK = 0xF;

    // Command Codes (from Escalator.cpp)
    static const uint8 CMD_CLEAR_RESET = 255;    // Clear/Reset (before stop)
    static const uint8 CMD_STOP_DOWN = 1;        // Stop (facing down direction)
    static const uint8 CMD_STOP_UP = 2;          // Stop (facing up direction)
    static const uint8 CMD_START_DOWN = 3;       // Start (direction down)
    static const uint8 CMD_START_UP = 4;         // Start (direction up)
    static const uint8 CMD_READY_MODE = 13;      // Ready Mode (after direction change)

    MessageCodec() : toggleBit_(0), debugLevel_(0) {}

    /// Create a POLL message (SHORT MESSAGE)
    /// Format: [0x10] [Address] [Function] [CRC] [0x16]
    /// 
    /// Based on TransportMsgWrapper::EncodePollMsg
    bool EncodePollMsg(uint8 address, std::vector<uint8>& msg)
    {
        msg.clear();
        msg.resize(TLM_SHORT_MSG_SIZE);

        msg[0] = TLM_SHORT_MSG_ID;  // 0x10
        msg[1] = address;
        msg[2] = GetFct(true, toggleBit_, TLM_CTRL_READ);
        msg[3] = CalcChecksum(msg, TLM_SHORT_CS_START, TLM_SHORT_CS_LEN);
        msg[4] = TLM_MSG_ENDMARK;  // 0x16

        if (debugLevel_ > 1) {
            std::cout << "[MessageCodec] POLL message: ";
            PrintMessage(msg);
        }

        toggleBit_ = (toggleBit_ + 1) & 1;
        return true;
    }

    /// Create a QUERY message (LONG MESSAGE)
    /// Format: [0x68] [Len] [Len] [0x68] [Addr] [Fct] [DP...] [CRC] [0x16]
    ///
    /// Based on TransportMsgWrapper::EncodeQueryMsg
    bool EncodeQueryMsg(
        uint8 address,
        const std::vector<std::pair<uint16, uint8>>& dataPoints,
        std::vector<uint8>& msg)
    {
        uint32 dataSize = dataPoints.size();
        if (dataSize == 0 || (dataSize * ALM_DP_QUERY_SIZE) > TLM_LONG_DATA_MAX_SIZE)
            return false;

        uint32 size = TLM_LONG_SKELETON_SIZE + (dataSize * ALM_DP_QUERY_SIZE);
        msg.resize(size);

        msg[0] = TLM_LONG_MSG_ID;  // 0x68
        msg[1] = (dataSize * ALM_DP_QUERY_SIZE) + TLM_LONG_MSG_EXTRA_MAGIC;
        msg[2] = msg[1];  // Length copy
        msg[3] = TLM_LONG_MSG_ID;  // 0x68 redundant
        msg[4] = address;
        msg[5] = GetFct(true, toggleBit_, TLM_CTRL_READ);

        uint32 mindex = TLM_LONG_DATA_START_INDEX;
        for (uint32 index = 0; index < dataSize; ++index)
        {
            uint16 dataPoint = dataPoints[index].first;
            uint8 dpType = dataPoints[index].second;

            msg[mindex++] = (dataPoint >> 8) & 0xFF;  // High byte
            msg[mindex++] = (dataPoint & 0xFF);       // Low byte
            msg[mindex++] = GetALCtrl(true, false, dpType);
        }

        uint32 csBytes = mindex - 4;
        msg[mindex++] = CalcChecksum(msg, TLM_LONG_CS_START, csBytes);
        msg[mindex++] = TLM_MSG_ENDMARK;

        if (debugLevel_ > 1) {
            std::cout << "[MessageCodec] QUERY message with " << dataSize << " points: ";
            PrintMessage(msg);
        }

        toggleBit_ = (toggleBit_ + 1) & 1;
        return true;
    }

    /// Create a COMMAND message (LONG MESSAGE)
    /// Format: [0x68] [Len] [Len] [0x68] [Addr] [Fct] [DP_High] [DP_Low] [Ctrl] [Cmd] [CRC] [0x16]
    ///
    /// Based on TransportMsgWrapper::EncodeCommandMsg
    bool EncodeCommandMsg(
        uint8 address,
        uint16 dataPoint,
        uint8 commandCode,
        std::vector<uint8>& msg)
    {
        uint32 dataSize = 1;  // One byte for command
        uint32 msgDataSize = dataSize + ALM_DP_QUERY_SIZE;  // 1 + 3 = 4

        if (msgDataSize > TLM_LONG_DATA_MAX_SIZE)
            return false;

        uint32 size = TLM_LONG_SKELETON_SIZE + msgDataSize;
        msg.resize(size);

        msg[0] = TLM_LONG_MSG_ID;  // 0x68
        msg[1] = msgDataSize + TLM_LONG_MSG_EXTRA_MAGIC;
        msg[2] = msg[1];
        msg[3] = TLM_LONG_MSG_ID;
        msg[4] = address;
        msg[5] = GetFct(true, toggleBit_, TLM_CTRL_WRITE);

        uint32 mindex = TLM_LONG_DATA_START_INDEX;
        msg[mindex++] = (dataPoint >> 8) & 0xFF;
        msg[mindex++] = (dataPoint & 0xFF);
        msg[mindex++] = GetALCtrl(false, true, 0);  // Write only, Type=0
        msg[mindex++] = commandCode;

        uint32 csBytes = mindex - 4;
        msg[mindex++] = CalcChecksum(msg, TLM_LONG_CS_START, csBytes);
        msg[mindex++] = TLM_MSG_ENDMARK;

        if (debugLevel_ > 1) {
            std::cout << "[MessageCodec] COMMAND message (code=" << (int)commandCode << "): ";
            PrintMessage(msg);
        }

        toggleBit_ = (toggleBit_ + 1) & 1;
        return true;
    }

    /// Create a FAULT DETAILS message (LONG MESSAGE) - ECO3000 only
    /// Format: [0x68] [Len] [Len] [0x68] [Addr] [Fct] [DP_High] [DP_Low] [Ctrl] [DataLen] [Data...] [CRC] [0x16]
    ///
    /// Based on TransportMsgWrapper::EncodeFaultDetailsMsg
    bool EncodeFaultDetailsMsg(
        uint8 address,
        uint16 dataPoint,
        const std::vector<uint8>& faultData,
        std::vector<uint8>& msg)
    {
        uint32 dataSize = faultData.size();
        if (dataSize == 0 || dataSize > 250)
            return false;

        uint32 msgDataSize = dataSize + 1 + ALM_DP_QUERY_SIZE;  // Data + Len + DP(3)

        if (msgDataSize > TLM_LONG_DATA_MAX_SIZE)
            return false;

        uint32 size = TLM_LONG_SKELETON_SIZE + msgDataSize;
        msg.resize(size);

        msg[0] = TLM_LONG_MSG_ID;  // 0x68
        msg[1] = msgDataSize + TLM_LONG_MSG_EXTRA_MAGIC;
        msg[2] = msg[1];
        msg[3] = TLM_LONG_MSG_ID;
        msg[4] = address;
        msg[5] = GetFct(true, toggleBit_, TLM_CTRL_WRITE);

        uint32 mindex = TLM_LONG_DATA_START_INDEX;
        msg[mindex++] = (dataPoint >> 8) & 0xFF;
        msg[mindex++] = (dataPoint & 0xFF);
        msg[mindex++] = GetALCtrl(true, true, 0);  // Both Read AND Write!
        msg[mindex++] = static_cast<uint8>(dataSize);

        for (uint32 index = 0; index < dataSize; ++index)
            msg[mindex++] = faultData[index];

        uint32 csBytes = mindex - 4;
        msg[mindex++] = CalcChecksum(msg, TLM_LONG_CS_START, csBytes);
        msg[mindex++] = TLM_MSG_ENDMARK;

        if (debugLevel_ > 1) {
            std::cout << "[MessageCodec] FAULT DETAILS message (" << dataSize << " bytes): ";
            PrintMessage(msg);
        }

        toggleBit_ = (toggleBit_ + 1) & 1;
        return true;
    }

    /// Parse SHORT message (ACK/Poll response)
    bool DecodeShortReply(
        const std::vector<uint8>& data,
        uint8& address,
        bool& slaveLocked,
        bool& writeSucceeded,
        bool& eventDataAvailable)
    {
        if (data.size() != TLM_SHORT_MSG_SIZE)
            return false;

        if (data[0] != TLM_SHORT_MSG_ID)
            return false;

        address = data[1];

        // Verify checksum
        uint8 cs = CalcChecksum(data, TLM_SHORT_CS_START, TLM_SHORT_CS_LEN);
        if (cs != data[3])
            return false;

        uint8 ctrl = data[2];
        
        // Parse status flags
        slaveLocked = (ctrl & (1 << TLM_LOCKED_SHIFT)) != 0;
        writeSucceeded = (ctrl & (1 << TLM_ACK_SHIFT)) == 0;  // 0 means success
        eventDataAvailable = (ctrl & (1 << TLM_DA_SHIFT)) != 0;

        if (debugLevel_ > 1) {
            std::cout << "[MessageCodec] ACK reply from address " << (int)address;
            std::cout << " (Locked=" << slaveLocked << ", Success=" << writeSucceeded;
            std::cout << ", DataAvailable=" << eventDataAvailable << ")" << std::endl;
        }

        return true;
    }

    /// Parse LONG message (Query response with data)
    bool DecodeLongReply(
        const std::vector<uint8>& data,
        uint8& address,
        std::vector<DataPointReply>& values)
    {
        values.clear();
        if (data.size() < TLM_LONG_SKELETON_SIZE)
            return false;

        if (data[0] != TLM_LONG_MSG_ID || data[3] != TLM_LONG_MSG_ID ||
            data.back() != TLM_MSG_ENDMARK)
            return false;

        if (data[1] != data[2] || data[1] < TLM_LONG_MSG_EXTRA_MAGIC ||
            data.size() != static_cast<size_t>(data[1]) + 6)
            return false;

        address = data[4];

        uint32 csPos = data.size() - 2;
        uint8 cs = CalcChecksum(data, TLM_LONG_CS_START, csPos - TLM_LONG_CS_START);
        if (cs != data[csPos])
            return false;

        uint32 idx = TLM_LONG_DATA_START_INDEX;
        while (idx < csPos) {
            if (idx + 3 > csPos)
                return false;

            DataPointReply point;
            point.id = (static_cast<uint16>(data[idx]) << 8) | data[idx + 1];
            point.control = data[idx + 2];
            point.type = point.control & ALM_CTRL_TYPE_MSK;
            point.acknowledged = (point.control & (1 << ALM_CTRL_ACK_SHIFT)) != 0;
            idx += 3;

            size_t dataSize = 0;
            switch (point.type) {
                case 0: dataSize = 1; break;
                case 1: dataSize = 2; break;
                case 2: dataSize = 4; break;
                case 14:
                    if (idx >= csPos)
                        return false;
                    dataSize = static_cast<size_t>(data[idx]) + 1;
                    break;
                default:
                    return false;
            }
            if (idx + dataSize > csPos)
                return false;

            point.raw.assign(data.begin() + idx, data.begin() + idx + dataSize);
            idx += dataSize;
            values.push_back(point);
        }

        if (debugLevel_ > 1) {
            std::cout << "[MessageCodec] DATA reply from address " << (int)address;
            std::cout << " with " << values.size() << " values" << std::endl;
        }

        return true;
    }

    void SetDebugLevel(uint32 level) { debugLevel_ = level; }

    void SetToggleBit(uint8 toggle) { toggleBit_ = toggle & 1; }

    uint8 GetToggleBit() const { return toggleBit_; }

    void PrintMessage(const std::vector<uint8>& msg, const std::string& label = "")
    {
        if (!label.empty())
            std::cout << label << ": ";
        
        for (size_t i = 0; i < msg.size(); ++i) {
            if (i > 0) std::cout << " ";
            std::cout << "0x" << std::hex << std::uppercase << std::setw(2) 
                      << std::setfill('0') << (int)msg[i];
        }
        std::cout << std::dec << std::endl;
    }

private:
    /// Calculate Function Control Byte
    /// Based on TransportMsgWrapper::GetFct
    uint8 GetFct(bool isReq, uint8 toggle, uint8 ctrl) const
    {
        uint8 fct = 0;
        if (isReq)
            fct |= (1 << TLM_REQ_RESP_SHIFT);  // Bit 0: Request
        fct |= (1 << TLM_SELECT_SHIFT);       // Bit 3: Select
        if (toggle)
            fct |= (1 << TLM_SEQ_SHIFT);      // Bit 4: Sequence/Toggle
        fct |= (ctrl << TLM_CTRL_SHIFT);      // Bits 5-7: Control
        return fct;
    }

    /// Calculate simple checksum (sum mod 256)
    /// Based on TransportMsgWrapper::CalcChecksum
    uint8 CalcChecksum(const std::vector<uint8>& msg, uint32 startByte, uint32 numBytes) const
    {
        if (msg.size() < startByte + numBytes)
            return 0;

        uint32 tmpCS = msg[startByte];
        for (uint32 index = startByte + 1; index < startByte + numBytes; ++index)
            tmpCS = (tmpCS + msg[index]) % 256;

        return static_cast<uint8>(tmpCS);
    }

    /// Get Application Layer Control Byte
    /// Based on TransportMsgWrapper::GetALCtrl
    uint8 GetALCtrl(bool dpRead, bool dpWrite, uint8 type) const
    {
        uint8 ctrl = (type & ALM_CTRL_TYPE_MSK);
        if (dpRead)
            ctrl |= (1 << ALM_CTRL_READ_SHIFT);
        if (dpWrite)
            ctrl |= (1 << ALM_CTRL_WRITE_SHIFT);
        return ctrl;
    }

    uint8 toggleBit_;
    uint32 debugLevel_;
};

#endif // MESSAGE_CODEC_H
