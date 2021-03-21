#include <Arduino.h>

#include "DeviceTypeSerializer.h"
#include "BaseDeviceType.h"
#include "../HAMqtt.h"
#include "../HADevice.h"

static const char CharSlash[] PROGMEM = {"/"};
static const char CharUnderscore[] PROGMEM = {"_"};
static const char CharQuotation[] PROGMEM = {"\""};

const char* DeviceTypeSerializer::ConfigTopic = "config";
const char* DeviceTypeSerializer::EventTopic = "event";
const char* DeviceTypeSerializer::AvailabilityTopic = "avail";
const char* DeviceTypeSerializer::StateTopic = "state";
const char* DeviceTypeSerializer::CommandTopic = "cmd";
const char* DeviceTypeSerializer::Online = "online";
const char* DeviceTypeSerializer::Offline = "offline";
const char* DeviceTypeSerializer::StateOn = "ON";
const char* DeviceTypeSerializer::StateOff = "OFF";

uint16_t DeviceTypeSerializer::calculateTopicLength(
    const char* component,
    const char* objectId,
    const char* suffix,
    bool includeNullTerminator
)
{
    const char* prefix = HAMqtt::instance()->getDiscoveryPrefix();
    if (prefix == nullptr) {
        return 0;
    }

    uint16_t size =
        strlen(prefix) + 1 + // prefix with slash
        strlen(suffix);

    if (component != nullptr) {
        size += strlen(component) + 1; // component with slash
    }

    if (HAMqtt::instance()->getDevice() != nullptr) {
        size += strlen(HAMqtt::instance()->getDevice()->getUniqueId()) + 1; // device ID with slash
    }

    if (objectId != nullptr) {
        size += strlen(objectId) + 1; // with slash
    }

    if (includeNullTerminator) {
        size += 1;
    }

    return size;
}

uint16_t DeviceTypeSerializer::generateTopic(
    char* output,
    const char* component,
    const char* objectId,
    const char* suffix
)
{
    const char* prefix = HAMqtt::instance()->getDiscoveryPrefix();
    if (prefix == nullptr) {
        return 0;
    }

    strcpy(output, prefix);
    strcat_P(output, CharSlash);

    if (component != nullptr) {
        strcat(output, component);
        strcat_P(output, CharSlash);
    }

    if (HAMqtt::instance()->getDevice() != nullptr) {
        strcat(output, HAMqtt::instance()->getDevice()->getUniqueId());
        strcat_P(output, CharSlash);
    }

    if (objectId != nullptr) {
        strcat(output, objectId);
        strcat_P(output, CharSlash);
    }

    strcat(output, suffix);
    return strlen(output) + 1; // size with null terminator
}

uint16_t DeviceTypeSerializer::calculateBaseJsonDataSize()
{
    return 2; // opening and closing brackets of the JSON data
}

uint16_t DeviceTypeSerializer::calculateNameFieldSize(const char* name)
{
    if (name == nullptr) {
        return 0;
    }

    // Field format: ,"name":"[NAME]"
    return strlen(name) + 10; // 10 - length of the JSON decorators for this field
}

uint16_t DeviceTypeSerializer::calculateUniqueIdFieldSize(
    const char* name
)
{
    HADevice const* device = HAMqtt::instance()->getDevice();
    if (device == nullptr || name == nullptr) {
        return 0;
    }

    // Field format: ,"uniq_id":"[DEVICE ID]_[NAME]"
    return (
        strlen(device->getUniqueId()) +
        strlen(name) +
        14 // 14 - length of the JSON decorators for this field
    );
}

uint16_t DeviceTypeSerializer::calculateAvailabilityFieldSize(
    const BaseDeviceType* const dt
)
{
    const HADevice* device = HAMqtt::instance()->getDevice();
    if (device == nullptr) {
        return 0;
    }

    const bool& sharedAvailability = device->isSharedAvailabilityEnabled();
    if (!sharedAvailability && !dt->isAvailabilityConfigured()) {
        return 0;
    }

    const uint16_t& availabilityTopicLength = calculateTopicLength(
        (sharedAvailability ? nullptr : dt->componentName()),
        (sharedAvailability ? nullptr : dt->name()),
        AvailabilityTopic,
        false
    );

    if (availabilityTopicLength == 0) {
        return 0;
    }

    // Field format: ,"avty_t":"[TOPIC]"
    return availabilityTopicLength + 12; // 12 - length of the JSON decorators for this field
}

uint16_t DeviceTypeSerializer::calculateRetainFieldSize(bool retain)
{
    if (!retain) {
        return 0;
    }

    // Field format: ,"ret":true
    return 11;
}

uint16_t DeviceTypeSerializer::calculateDeviceFieldSize(
    const char* serializedDevice
)
{
    if (serializedDevice == nullptr) {
        return 0;
    }

    // Field format: ,"dev":[DEVICE]
    return strlen(serializedDevice) + 7; // 7 - length of the JSON decorators for this field
}

void DeviceTypeSerializer::mqttWriteBeginningJson()
{
    static const char Data[] PROGMEM = {"{"};
    HAMqtt::instance()->writePayload_P(Data);
}

void DeviceTypeSerializer::mqttWriteEndJson()
{
    static const char Data[] PROGMEM = {"}"};
    HAMqtt::instance()->writePayload_P(Data);
}

void DeviceTypeSerializer::mqttWriteConstCharField(
    const char* prefix,
    const char* value,
    bool quoteSuffix
)
{
    if (prefix == nullptr || value == nullptr) {
        return;
    }

    HAMqtt::instance()->writePayload_P(prefix);
    HAMqtt::instance()->writePayload(value, strlen(value));

    if (quoteSuffix) {
        HAMqtt::instance()->writePayload_P(CharQuotation);
    }
}

void DeviceTypeSerializer::mqttWriteNameField(const char* name)
{
    if (name == nullptr) {
        return;
    }

    static const char Prefix[] PROGMEM = {",\"name\":\""};
    mqttWriteConstCharField(Prefix, name);
}

void DeviceTypeSerializer::mqttWriteUniqueIdField(
    const char* name
)
{
    if (name == nullptr) {
        return;
    }

    HADevice const* device = HAMqtt::instance()->getDevice();
    if (device == nullptr) {
        return;
    }

    static const char Prefix[] PROGMEM = {",\"uniq_id\":\""};

    uint8_t uniqueIdLength = strlen(name) + strlen(device->getUniqueId()) + 2; // underscore and null temrinator
    char uniqueId[uniqueIdLength];
    strcpy(uniqueId, name);
    strcat_P(uniqueId, CharUnderscore);
    strcat(uniqueId, device->getUniqueId());

    mqttWriteConstCharField(Prefix, uniqueId);
}

void DeviceTypeSerializer::mqttWriteAvailabilityField(
    const BaseDeviceType* const dt
)
{
    const HADevice* device = HAMqtt::instance()->getDevice();
    if (device == nullptr) {
        return;
    }

    const bool& sharedAvailability = device->isSharedAvailabilityEnabled();
    if (!sharedAvailability && !dt->isAvailabilityConfigured()) {
        return;
    }

    const uint16_t& topicSize = calculateTopicLength(
        (sharedAvailability ? nullptr : dt->componentName()),
        (sharedAvailability ? nullptr : dt->name()),
        AvailabilityTopic
    );
    if (topicSize == 0) {
        return;
    }

    char availabilityTopic[topicSize];
    generateTopic(
        availabilityTopic,
        (sharedAvailability ? nullptr : dt->componentName()),
        (sharedAvailability ? nullptr : dt->name()),
        AvailabilityTopic
    );

    if (strlen(availabilityTopic) == 0) {
        return;
    }

    static const char Prefix[] PROGMEM = {",\"avty_t\":\""};
    mqttWriteConstCharField(Prefix, availabilityTopic);
}

void DeviceTypeSerializer::mqttWriteRetainField(
    bool retain
)
{
    if (!retain) {
        return;
    }

    static const char Prefix[] PROGMEM = {",\"ret\":"};
    mqttWriteConstCharField(
        Prefix,
        "true",
        false
    );
}

void DeviceTypeSerializer::mqttWriteDeviceField(
    const char* serializedDevice
)
{
    if (serializedDevice == nullptr) {
        return;
    }

    static const char Data[] PROGMEM = {",\"dev\":"};

    HAMqtt::instance()->writePayload_P(Data);
    HAMqtt::instance()->writePayload(serializedDevice, strlen(serializedDevice));
}

bool DeviceTypeSerializer::mqttWriteTopicField(
    const BaseDeviceType* const dt,
    const char* jsonPrefix,
    const char* topicSuffix
)
{
    if (jsonPrefix == nullptr || topicSuffix == nullptr) {
        return false;
    }

    const uint16_t& topicSize = DeviceTypeSerializer::calculateTopicLength(
        dt->componentName(),
        dt->name() ,
        topicSuffix
    );
    if (topicSize == 0) {
        return false;
    }

    char topic[topicSize];
    DeviceTypeSerializer::generateTopic(
        topic,
        dt->componentName(),
        dt->name(),
        topicSuffix
    );

    if (strlen(topic) == 0) {
        return false;
    }

    DeviceTypeSerializer::mqttWriteConstCharField(jsonPrefix, topic);
    return true;
}

bool DeviceTypeSerializer::mqttPublishMessage(
    const BaseDeviceType* const dt,
    const char* topicSuffix,
    const char* data
)
{
    if (topicSuffix == nullptr || data == nullptr) {
        return false;
    }

    const uint16_t& topicSize = calculateTopicLength(
        dt->componentName(),
        dt->name(),
        topicSuffix
    );
    if (topicSize == 0) {
        return false;
    }

    char topic[topicSize];
    generateTopic(
        topic,
        dt->componentName(),
        dt->name(),
        topicSuffix
    );

    if (strlen(topic) == 0) {
        return false;
    }

    return HAMqtt::instance()->publish(
        topic,
        data,
        true
    );
}

bool DeviceTypeSerializer::mqttSubscribeTopic(
    const BaseDeviceType* const dt,
    const char* topicSuffix
)
{
    const uint16_t& topicSize = calculateTopicLength(
        dt->componentName(),
        dt->name(),
        topicSuffix
    );
    if (topicSize == 0) {
        return false;
    }

    char topic[topicSize];
    generateTopic(
        topic,
        dt->componentName(),
        dt->name(),
        topicSuffix
    );

    if (strlen(topic) == 0) {
        return false;
    }

    return HAMqtt::instance()->subscribe(topic);
}
