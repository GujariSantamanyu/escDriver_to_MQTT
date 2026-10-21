/*******************************************************************************
 * 
 *      File:    EscalatorStatus.h
 *      
 *******************************************************************************
 *
 *      Target product: escDriver_to_MQTT
 *      Purpose: Data structure for escalator status and data points
 *
 *******************************************************************************
 */

#ifndef ESCALATOR_STATUS_H
#define ESCALATOR_STATUS_H

#include "Types.h"
#include <ctime>
#include <iostream>

class EscalatorStatus
{
public:
    enum Mode_t {
        MODE_NONE = 0,
        MODE_AUTO = 1,
        MODE_CONTINUOUS = 2,
        MODE_USER = 3,
        MODE_ENERGY_SAVE = 4,
        MODE_INSPECTION = 5
    };

    enum Direction_t {
        DIR_DOWN = 0,
        DIR_UP = 1,
        DIR_NONE = 2
    };

    enum OperStatus_t {
        OPER_STOPPED = 0,
        OPER_INVERTER = 1,
        OPER_MAINS = 2,
        OPER_ERROR = 3
    };

    EscalatorStatus()
        : escalatorID_(0),
          mode_(MODE_NONE),
          direction_(DIR_NONE),
          status_(OPER_STOPPED),
          speed_(0.0f),
          load_(0),
          motorTemp_(0),
          faultCount_(0),
          lastUpdate_(time(NULL)),
          isConnected_(false)
    {
    }

    // Getters and setters
    uint32 GetID() const { return escalatorID_; }
    void SetID(uint32 id) { escalatorID_ = id; }

    Mode_t GetMode() const { return mode_; }
    void SetMode(Mode_t m) { mode_ = m; }

    Direction_t GetDirection() const { return direction_; }
    void SetDirection(Direction_t d) { direction_ = d; }

    OperStatus_t GetStatus() const { return status_; }
    void SetStatus(OperStatus_t s) { status_ = s; }

    float GetSpeed() const { return speed_; }
    void SetSpeed(float s) { speed_ = s; }

    uint32 GetLoad() const { return load_; }
    void SetLoad(uint32 l) { load_ = l; }

    uint32 GetMotorTemp() const { return motorTemp_; }
    void SetMotorTemp(uint32 t) { motorTemp_ = t; }

    uint32 GetFaultCount() const { return faultCount_; }
    void SetFaultCount(uint32 f) { faultCount_ = f; }

    bool IsConnected() const { return isConnected_; }
    void SetConnected(bool c) { isConnected_ = c; }

    time_t GetLastUpdate() const { return lastUpdate_; }
    void UpdateTimestamp() { lastUpdate_ = time(NULL); }

    // Print status to console
    void PrintStatus() const
    {
        std::cout << "\n[EscalatorStatus] ID: " << escalatorID_ << std::endl;
        std::cout << "  Mode: ";
        switch (mode_) {
            case MODE_AUTO: std::cout << "AUTO"; break;
            case MODE_CONTINUOUS: std::cout << "CONTINUOUS"; break;
            case MODE_USER: std::cout << "USER"; break;
            case MODE_ENERGY_SAVE: std::cout << "ENERGY_SAVE"; break;
            case MODE_INSPECTION: std::cout << "INSPECTION"; break;
            default: std::cout << "UNKNOWN"; break;
        }
        std::cout << std::endl;

        std::cout << "  Direction: ";
        switch (direction_) {
            case DIR_UP: std::cout << "UP"; break;
            case DIR_DOWN: std::cout << "DOWN"; break;
            case DIR_NONE: std::cout << "STOPPED"; break;
            default: std::cout << "UNKNOWN"; break;
        }
        std::cout << std::endl;

        std::cout << "  Status: ";
        switch (status_) {
            case OPER_STOPPED: std::cout << "STOPPED"; break;
            case OPER_INVERTER: std::cout << "INVERTER"; break;
            case OPER_MAINS: std::cout << "MAINS"; break;
            case OPER_ERROR: std::cout << "ERROR"; break;
            default: std::cout << "UNKNOWN"; break;
        }
        std::cout << std::endl;

        std::cout << "  Speed: " << speed_ << " m/s" << std::endl;
        std::cout << "  Load: " << load_ << "%" << std::endl;
        std::cout << "  Motor Temp: " << motorTemp_ << "°C" << std::endl;
        std::cout << "  Faults: " << faultCount_ << std::endl;
        std::cout << "  Connected: " << (isConnected_ ? "YES" : "NO") << std::endl;
    }

private:
    uint32 escalatorID_;
    Mode_t mode_;
    Direction_t direction_;
    OperStatus_t status_;
    float speed_;
    uint32 load_;
    uint32 motorTemp_;
    uint32 faultCount_;
    time_t lastUpdate_;
    bool isConnected_;
};

#endif // ESCALATOR_STATUS_H
