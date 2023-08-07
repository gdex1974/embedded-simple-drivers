#pragma once

#include "I2CDevice.h"
#include "Sps30Error.h"
#include "Sps30DataTypes.h"
#include <variant>

namespace embedded
{

class Sps30I2C
{
public:
    static constexpr uint8_t maxSerialLen = 32;

    explicit Sps30I2C(embedded::I2CBus& i2c) : sps30Device(i2c, 0x69), firmwareVersion(0) {}

    Sps30Error probe();
    std::variant<Sps30Error, Sps30VersionInformation> getVersion();
    Sps30Error getSerial(char serial[maxSerialLen]);

    Sps30Error startMeasurement(bool floating = true);
    Sps30Error stopMeasurement();
    bool isDataReady();
    std::variant<Sps30Error, Sps30MeasurementData> readMeasurement();

    std::variant<Sps30Error, uint32_t> getFanAutoCleaningInterval();
    Sps30Error setFanAutoCleaningInterval(uint32_t interval_seconds);
    Sps30Error startManualFanCleaning();

    Sps30Error resetSensor();
    Sps30Error sleep();
    Sps30Error wakeUp();

    std::variant<Sps30Error, uint32_t> readDeviceStatusRegister();
    Sps30Error clearDeviceStatusRegister();

private:
    Sps30Error readVersion();
    bool sendCommand(uint16_t command);
    Sps30Error sendCommandGetResponce(uint16_t cmd, uint8_t *bytes, uint16_t size, uint32_t delay_ms = 0);
    Sps30Error readBytesWithCRC(uint8_t *data, uint16_t size);
    embedded::I2CDevice sps30Device;
    uint16_t firmwareVersion;
    bool measurementInFloat {};
};

}
