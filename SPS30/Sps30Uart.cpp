#include <cstring>
#include "EndianConversion.h"
#include "Sps30Uart.h"
#include "ShdlcTransport.h"
#include "PacketUart.h"
#include "Delays.h"
#include "Debug.h"

namespace
{
    constexpr uint8_t sps30ShdlcAddr = 0x00;
}

namespace embedded
{

Sps30Error Sps30Uart::probe()
{
    wakeUp();
    char serial[maxDeviceInformationlLength];
    uint8_t paramBuf[] = { 0x00 }; // Get product type
    const auto result = transport.sendAndReceive(sps30ShdlcAddr, 0xd0, paramBuf,
                                    { (uint8_t*)serial, maxDeviceInformationlLength });
    if (result == Sps30Error::Success)
    {
        if (std::strcmp(serial, "00080000") != 0)
        {
            return Sps30Error::UnsupportedCommand;
        }
    }
    return result;
}

Sps30Error Sps30Uart::getSerial(char serial[maxDeviceInformationlLength])
{
    uint8_t paramBuf[] = { 0x03 }; // Get serial number
    return transport.sendAndReceive(sps30ShdlcAddr, 0xd0, paramBuf,
                                    { (uint8_t*)serial, maxDeviceInformationlLength });
}

Sps30Error Sps30Uart::startMeasurement(bool floating)
{
    uint8_t paramBuf[2] = { 0x01, floating ? (uint8_t)0x03u : (uint8_t)0x05u };

    return transport.sendAndReceive(sps30ShdlcAddr, 0x00, paramBuf, {});
}

Sps30Error Sps30Uart::stopMeasurement()
{
    return transport.sendAndReceive(
            sps30ShdlcAddr, 0x01, {}, {});
}

std::variant<Sps30Error, Sps30Uart::measurementData> Sps30Uart::readMeasurement()
{
    std::variant<Sps30Error, Sps30Uart::measurementData> result;
    uint32_t data[10];
    embedded::BytesView bytesView { (uint8_t*)data, sizeof(data) };

    const auto transportResult = transport.sendAndReceive(sps30ShdlcAddr, 0x03, {}, bytesView);
    if (transportResult== Sps30Error::Success)
    {
        auto& measurement = result.emplace<measurementData>();
        if (bytesView.size() == 40)
        {
            measurement.floatData.mc_1p0 = embedded::changeEndianessToFloat(data[0]);
            measurement.floatData.mc_2p5 = embedded::changeEndianessToFloat(data[1]);
            measurement.floatData.mc_4p0 = embedded::changeEndianessToFloat(data[2]);
            measurement.floatData.mc_10p0 = embedded::changeEndianessToFloat(data[3]);
            measurement.floatData.nc_0p5 = embedded::changeEndianessToFloat(data[4]);
            measurement.floatData.nc_1p0 = embedded::changeEndianessToFloat(data[5]);
            measurement.floatData.nc_2p5 = embedded::changeEndianessToFloat(data[6]);
            measurement.floatData.nc_4p0 = embedded::changeEndianessToFloat(data[7]);
            measurement.floatData.nc_10p0 = embedded::changeEndianessToFloat(data[8]);
            measurement.floatData.typical_particle_size = embedded::changeEndianessToFloat(data[9]);
            measurement.measureInFloat = true;
        }
        else
        {
            auto* unsignedData = reinterpret_cast<uint16_t*>(data);
            measurement.unsignedData.mc_1p0 = embedded::changeEndianess(unsignedData[0]);
            measurement.unsignedData.mc_2p5 = embedded::changeEndianess(unsignedData[1]);
            measurement.unsignedData.mc_4p0 = embedded::changeEndianess(unsignedData[2]);
            measurement.unsignedData.mc_10p0 = embedded::changeEndianess(unsignedData[3]);
            measurement.unsignedData.nc_0p5 = embedded::changeEndianess(unsignedData[4]);
            measurement.unsignedData.nc_1p0 = embedded::changeEndianess(unsignedData[5]);
            measurement.unsignedData.nc_2p5 = embedded::changeEndianess(unsignedData[6]);
            measurement.unsignedData.nc_4p0 = embedded::changeEndianess(unsignedData[7]);
            measurement.unsignedData.nc_10p0 = embedded::changeEndianess(unsignedData[8]);
            measurement.unsignedData.typical_particle_size = embedded::changeEndianess(unsignedData[9]);
            measurement.measureInFloat = false;
        }
    }
    else
    {
        result = transportResult;
    }

    return result;
}

Sps30Error Sps30Uart::sleep()
{
    return transport.sendAndReceive(sps30ShdlcAddr, 0x10, {}, {});
}

Sps30Error Sps30Uart::wakeUp()
{
    DEBUG_LOG("Sps30Uart processing wakeup request")
    auto result = transport.activateTransport();
    if (result != Sps30Error::Success)
    {
        return result;
    }
    return transport.sendAndReceive(sps30ShdlcAddr, 0x11, {}, {});
}

std::variant<Sps30Error, uint32_t> Sps30Uart::getFanAutoCleaningInterval()
{
    uint8_t tx_data[] = { 0x00 };

    union
    {
        uint32_t data;
        uint8_t bytes[4];
    } data;
    auto transportResult = transport.sendAndReceive(sps30ShdlcAddr, 0x80, tx_data, data.bytes);
    if (transportResult == Sps30Error::Success)
    {
        return embedded::changeEndianess(data.data);
    }

    return transportResult;
}

Sps30Error Sps30Uart::setFanAutoCleaningInterval(uint32_t intervalSeconds)
{
#pragma pack(push, 1)
    union
    {
        struct
        {
            uint8_t subCommand;
            uint32_t interval;
        };
        uint8_t cleaningCommand[5];
    } cleaningCommand;
#pragma pack(pop)

    cleaningCommand.subCommand = 0x00;
    cleaningCommand.interval = embedded::changeEndianess(intervalSeconds);

    return transport.sendAndReceive(
            sps30ShdlcAddr, 0x80, cleaningCommand.cleaningCommand, {});
}

Sps30Error Sps30Uart::startManualFanCleaning()
{
    return transport.sendAndReceive(sps30ShdlcAddr, 0x56, {}, {});
}

std::variant<Sps30Error, Sps30Uart::versionInformation> Sps30Uart::getVersion()
{
    uint8_t data[7];

    auto result = transport.sendAndReceive(sps30ShdlcAddr, 0xd1, {}, data);
    if (result == Sps30Error::Success)
    {
        return versionInformation {
                .firmware_major = data[0],
                .firmware_minor = data[1],
                .hardware_revision = data[3],
                .shdlc_major = data[5],
                .shdlc_minor = data[6]
        };
    }
    return result;
}

Sps30Error Sps30Uart::resetSensor()
{
    auto result = transport.send(sps30ShdlcAddr, 0xd3, {});
    if (result == Sps30Error::Success)
    {
        embedded::delay(100);
    }
    return result;
}

}
