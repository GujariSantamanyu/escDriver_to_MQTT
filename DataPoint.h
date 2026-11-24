/*******************************************************************************
 *
 *      File:    DataPoint.h
 *
 *******************************************************************************
 *
 *      Target product: escDriver_to_MQTT
 *      Purpose: Escalator data-point identifiers
 *
 *******************************************************************************
 */

#ifndef DATA_POINT_H
#define DATA_POINT_H

#include "Types.h"

// ECO3000 Data Points
static const uint32 ECO3000_DP_VERSION = 0x1001;
static const uint32 ECO3000_DP_FAULT_STATUS = 0x1010;
static const uint32 ECO3000_DP_EXTRA_INFO = 0x1012;
static const uint32 ECO3000_DP_OPER_STATUS = 0x1020;
static const uint32 ECO3000_DP_SPEED = 0x1021;
static const uint32 ECO3000_DP_BRAKE_DISTANCE = 0x1022;
static const uint32 ECO3000_DP_WBS = 0x1023;
static const uint32 ECO3000_DP_REMOTE_CTRL = 0x1070;
static const uint32 ECO3000_DP_POWER_ON_TIME = 0x1080;
static const uint32 ECO3000_DP_BREAK_DOWN_TIME = 0x1081;
static const uint32 ECO3000_DP_OPERTIME_UP = 0x1082;
static const uint32 ECO3000_DP_OPERTIME_DOWN = 0x1083;
static const uint32 ECO3000_DP_INSPECTION_TIME = 0x1084;
static const uint32 ECO3000_DP_RUNS_UP = 0x10A0;
static const uint32 ECO3000_DP_RUNS_DOWN = 0x10A1;
static const uint32 ECO3000_DP_FAULT_COUNT = 0x10A2;
static const uint32 ECO3000_DP_EMERGENCY_STOPS = 0x10A3;
static const uint32 ECO3000_DP_PASSENGERS_UP = 0x10A4;
static const uint32 ECO3000_DP_PASSENGERS_DOWN = 0x10A5;

#endif // DATA_POINT_H
