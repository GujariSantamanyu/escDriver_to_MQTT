/*******************************************************************************
 * 
 *      File:    Driver.h
 *      
 *******************************************************************************
 *
 *      Target product: escDriver_to_MQTT
 *      Purpose: Core driver for escalator to MQTT bridge
 *
 *******************************************************************************
 */

#ifndef _DRIVER_H
#define _DRIVER_H

#include "Types.h"
#include "SimpleSerialComm.h"
#include "MessageCodec.h"
#include "EscalatorStatus.h"
#include <map>

class Driver {
 private:
    Driver(Driver &);  // No copy constructor

    CtrlState_t state_;
    uint32 driverIndex_;
    uint32 debugLevel_;
    UIntData_t addresses_;
    bool running_;

    SimpleSerialComm serialComm_;      ///< Serial communication interface
    MessageCodec messageCodec_;        ///< Message encoding/decoding
    std::map<uint8, EscalatorStatus> escalatorData_;  ///< Cached escalator status
    uint32 pollIntervalMs_;            ///< Polling interval in milliseconds
    uint32 pollCount_;                 ///< Number of polls performed

 public:
    Driver();
    ~Driver();

    /// Setup the whole driver
    bool Setup(
        const uint32 driverIndex,      ///< Driver index
        const uint32 debugLevel,       ///< Debug level (0-3)
        const UIntData_t &addresses    ///< Escalator addresses
    );

    /// Start the driver main loop
    bool Start();

    /// Stop the driver gracefully
    void Stop();

    /// Check if stopping is in progress
    bool IsStopping() const;

    /// Get current driver state
    CtrlState_t GetState() const { return state_; }

    /// Process escalator data and publish to MQTT
    void ProcessData();

    /// Get escalator status by address
    bool GetEscalatorStatus(uint8 address, EscalatorStatus& status) const;

    /// Get poll count
    uint32 GetPollCount() const { return pollCount_; }
};

#endif // _DRIVER_H
