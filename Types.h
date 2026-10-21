/*******************************************************************************
 * 
 *      File:    Types.h
 *      
 *******************************************************************************
 *
 *      Target product: escDriver_to_MQTT
 *      Purpose: Basic types for escalator MQTT bridge
 *
 *******************************************************************************
 */

#ifndef TYPES_H
#define TYPES_H

#include <string>
#include <vector>
#include <map>
#include <cstdint>

using std::string;

typedef uint8_t uint8;
typedef uint16_t uint16;
typedef uint32_t uint32;
typedef int32_t sint32;
typedef std::vector<uint32> UIntData_t;

typedef enum {
    NVLD_CST = 0,         // Setup failed
    RUNNING_CST,          // Alive and kicking
    FAILED_CST,           // Failed but still running
    ABORT_REQUESTED_CST,  // Abort requested
    ABORTED_CST,          // Down and Out
} CtrlState_t;

#endif // TYPES_H
