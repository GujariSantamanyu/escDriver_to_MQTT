/*******************************************************************************
 *
 *      File:    MQTTPayload.cpp
 *
 *******************************************************************************
 *
 *      Target product: escDriver_to_MQTT
 *      Purpose: Build escalator status MQTT payloads and topics
 *
 *******************************************************************************
 */

#include "MQTTPayload.h"
#include "DataPoint.h"
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

MQTTPayload::Values MQTTPayload::ReadValues(
    const std::vector<DataPointReply>& dataPoints)
{
    Values values;
    for (size_t index = 0; index < dataPoints.size(); ++index) {
        const DataPointReply& point = dataPoints[index];
        const uint32 value = ReadUnsigned(point.raw);
        switch (point.id) {
            case ECO3000_DP_OPER_STATUS:
                values.operationStatus = value;
                break;
            case ECO3000_DP_SPEED:
                values.runningSpeed = value;
                break;
            case ECO3000_DP_BRAKE_DISTANCE:
                values.brakingDistance = value;
                break;
            default:
                break;
        }
    }
    return values;
}

uint32 MQTTPayload::ReadUnsigned(const std::vector<uint8>& raw)
{
    uint32 value = 0;
    for (size_t index = 0; index < raw.size(); ++index)
        value = (value << 8) | raw[index];
    return value;
}

std::string MQTTPayload::CurrentUtcTime()
{
    const auto now = std::chrono::system_clock::now();
    const auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
    const std::time_t seconds = std::chrono::system_clock::to_time_t(now);
    std::tm utc = {};
    gmtime_r(&seconds, &utc);

    std::ostringstream time;
    time << std::put_time(&utc, "%Y-%m-%dT%H:%M:%S")
         << '.' << std::setfill('0') << std::setw(3)
         << milliseconds.count() << 'Z';
    return time.str();
}

std::string MQTTPayload::EscapeJson(const std::string& value)
{
    std::ostringstream escaped;
    for (size_t index = 0; index < value.size(); ++index) {
        const unsigned char character = value[index];
        switch (character) {
            case '"': escaped << "\\\""; break;
            case '\\': escaped << "\\\\"; break;
            case '\b': escaped << "\\b"; break;
            case '\f': escaped << "\\f"; break;
            case '\n': escaped << "\\n"; break;
            case '\r': escaped << "\\r"; break;
            case '\t': escaped << "\\t"; break;
            default:
                if (character < 0x20)
                    escaped << "\\u" << std::hex << std::setw(4)
                            << std::setfill('0') << static_cast<int>(character)
                            << std::dec;
                else
                    escaped << character;
                break;
        }
    }
    return escaped.str();
}

std::string MQTTPayload::Build(
    const std::string& equipmentNumber,
    const std::vector<DataPointReply>& dataPoints)
{
    const Values values = ReadValues(dataPoints);
    const uint32 status = values.operationStatus;
    const bool running = (status & (1U << 2)) != 0;
    const bool inverter = (status & (1U << 8)) != 0;
    const bool line = (status & (1U << 9)) != 0;
    const bool standby = (status & (1U << 10)) != 0;
    const bool commonFault = (status & (1U << 14)) != 0;
    const bool shutdownFault = (status & (1U << 15)) != 0;
    const uint32 mode = (status >> 4) & 0x0F;

    std::string systemState = "NormalStop";
    if (shutdownFault)
        systemState = "Shutdown";
    else if (commonFault)
        systemState = "FaultStop";
    else if (running && standby)
        systemState = "RunningStandby";
    else if (running && inverter)
        systemState = "RunningInverter";
    else if (running)
        systemState = "RunningYD";

    std::string systemMode = "Normal";
    if (mode == 15)
        systemMode = "Inspection";
    else if (mode == 0)
        systemMode = "None";

    std::string operationMode = "None";
    if (mode == 1)
        operationMode = "KAM";
    else if (mode == 2)
        operationMode = "KCM";
    else if (mode == 3)
        operationMode = "K2D";
    else if (mode == 4)
        operationMode = "KSB";

    std::string direction = "None";
    if ((status & (1U << 0)) != 0)
        direction = "Up";
    else if ((status & (1U << 1)) != 0)
        direction = "Down";

    const std::string powerMode = (inverter && !line) ? "Inverter" : "Star";

    std::string speedLevel = "Primary";
    if (!running)
        speedLevel = "Stopped";
    else if (mode == 15)
        speedLevel = "Inspection";
    else if (standby)
        speedLevel = "Standby";

    std::string faultStatus = "NoFault";
    if (shutdownFault)
        faultStatus = "ManualReset";
    else if (commonFault)
        faultStatus = "KeyStart";

    std::ostringstream payload;
    payload << "{"
            << "\"time\":\"" << CurrentUtcTime() << "\","
            << "\"recreated\":false,"
            << "\"eq\":\"" << EscapeJson(equipmentNumber) << "\","
            << "\"system_state\":\"" << systemState << "\","
            << "\"system_mode\":\"" << systemMode << "\","
            << "\"operation_mode\":\"" << operationMode << "\","
            << "\"operation_direction\":\"" << direction << "\","
            << "\"power_mode\":\"" << powerMode << "\","
            << "\"speed_level\":\"" << speedLevel << "\","
            << "\"fault_status\":\"" << faultStatus << "\","
            << "\"running_speed\":" << values.runningSpeed << ","
            << "\"braking_distance\":" << values.brakingDistance << ","
            << "\"braking_distance_speed\":0,"
            << "\"dynamic_braking_option\":false,"
            << "\"speed_info\":{"
            << "\"motor_frequency\":0,"
            << "\"mainshaft_frequency\":0,"
            << "\"handrail_left_frequency\":0,"
            << "\"handrail_right_frequency\":0"
            << "}}";
    return payload.str();
}

std::string MQTTPayload::BuildTopic(
    const std::string& siteId,
    const std::string& set,
    const std::string& node)
{
    return "siteapi.from/" + siteId + "/" + set + "/" + node +
        "/escalator_v1/status";
}
