#pragma once

#include "ShdlcTransport.h"
#include "Sps30Error.h"

#include <cstdint>
#include <variant>

namespace embedded
{

class Sps30Uart
{
public:
    static constexpr uint8_t maxDeviceInformationlLength = 32;

    explicit Sps30Uart(embedded::PacketUart &uart) : transport(uart) {}

    struct measurementData
    {
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

    struct versionInformation
    {
        uint8_t firmware_major;
        uint8_t firmware_minor;
        uint8_t hardware_revision;
        uint8_t shdlc_major;
        uint8_t shdlc_minor;
    };

    Sps30Error probe();

    Sps30Error getSerial(char serial[maxDeviceInformationlLength]);
    std::variant<Sps30Error, versionInformation> getVersion();

    Sps30Error startMeasurement(bool floating = true);
    Sps30Error stopMeasurement();
    std::variant<Sps30Error, measurementData> readMeasurement();

    Sps30Error sleep();
    Sps30Error wakeUp();
    Sps30Error resetSensor();

    std::variant<Sps30Error, uint32_t> getFanAutoCleaningInterval();
    Sps30Error setFanAutoCleaningInterval(uint32_t intervalSeconds);
    Sps30Error startManualFanCleaning();

private:
    ShdlcTransport transport;
};

}
