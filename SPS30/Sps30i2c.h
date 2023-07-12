#pragma once
#include "I2CDevice.h"
#include "Sps30Error.h"
#include <tuple>

namespace embedded
{

class Sps30I2C
{
public:
    static constexpr uint8_t maxSerialLen = 32;

    struct measurementData {
        union
        {
            struct
            {
                float mc_1p0;
                float mc_2p5;
                float mc_4p0;
                float mc_10p0;
                float nc_0p5;
                float nc_1p0;
                float nc_2p5;
                float nc_4p0;
                float nc_10p0;
                float typical_particle_size;
            } floatData;
            struct
            {
                uint16_t mc_1p0;
                uint16_t mc_2p5;
                uint16_t mc_4p0;
                uint16_t mc_10p0;
                uint16_t nc_0p5;
                uint16_t nc_1p0;
                uint16_t nc_2p5;
                uint16_t nc_4p0;
                uint16_t nc_10p0;
                uint16_t typical_particle_size;
            } unsignedData;
        };
        bool measureInFloat;
    };

    struct versionInformation {
        uint8_t firmware_major;
        uint8_t firmware_minor;
    };

    explicit Sps30I2C(embedded::I2CBus& i2c) : sps30Device(i2c, 0x69), firmwareVersion(0) {}

    Sps30Error probe();
    std::tuple<versionInformation, Sps30Error> getVersion();
    Sps30Error getSerial(char serial[maxSerialLen]);

    Sps30Error startMeasurement(bool floating = true);
    Sps30Error stopMeasurement();
    bool isDataReady();
    std::tuple<measurementData, Sps30Error> readMeasurement();

    std::tuple<uint32_t, Sps30Error> getFanAutoCleaningInterval();
    Sps30Error setFanAutoCleaningInterval(uint32_t interval_seconds);
    Sps30Error startManualFanCleaning();

    Sps30Error resetSensor();
    Sps30Error sleep();
    Sps30Error wakeUp();

    std::tuple<uint32_t, Sps30Error> readDeviceStatusRegister();
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
