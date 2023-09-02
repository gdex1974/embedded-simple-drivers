#pragma once

#include <cstdint>
#include <optional>

namespace embedded
{

struct Sps30MeasurementData
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

struct Sps30ShdlcInformation
{
    uint8_t hardware_revision;
    uint8_t shdlc_major;
    uint8_t shdlc_minor;
};

struct Sps30VersionInformation
{
    uint8_t firmware_major;
    uint8_t firmware_minor;
    std::optional<Sps30ShdlcInformation> shdlc;
};

struct Sps30SerialNumber
{
    char serial[32];
};

}