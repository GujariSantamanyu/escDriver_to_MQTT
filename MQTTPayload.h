/*******************************************************************************
 *
 *      File:    MQTTPayload.h
 *
 *******************************************************************************
 *
 *      Target product: escDriver_to_MQTT
 *      Purpose: Build escalator status MQTT payloads and topics
 *
 *******************************************************************************
 */

#ifndef MQTT_PAYLOAD_H
#define MQTT_PAYLOAD_H

#include "MessageCodec.h"
#include <string>
#include <vector>

class MQTTPayload
{
public:
    static std::string Build(
        const std::string& equipmentNumber,
        const std::vector<DataPointReply>& dataPoints);

    static std::string BuildTopic(
        const std::string& siteId,
        const std::string& set,
        const std::string& node);

private:
    struct Values
    {
        uint32 operationStatus;
        uint32 runningSpeed;
        uint32 brakingDistance;

        Values() : operationStatus(0), runningSpeed(0), brakingDistance(0) { }
    };

    static Values ReadValues(const std::vector<DataPointReply>& dataPoints);
    static uint32 ReadUnsigned(const std::vector<uint8>& raw);
    static std::string CurrentUtcTime();
    static std::string EscapeJson(const std::string& value);
};

#endif // MQTT_PAYLOAD_H
