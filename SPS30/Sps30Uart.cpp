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

Sps30Error Sps30Uart::getSerial(Sps30SerialNumber &serial)
{
    uint8_t paramBuf[] = { 0x03 }; // Get serial number
    BytesView serialView { reinterpret_cast<unsigned char*>(serial.serial), sizeof(serial.serial) };
    return transport.sendAndReceive(sps30ShdlcAddr, 0xd0, paramBuf, serialView);
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

std::variant<Sps30Error, Sps30MeasurementData> Sps30Uart::readMeasurement()
{
    uint32_t data[10];
    embedded::BytesView bytesView { (uint8_t*)data, sizeof(data) };

    const auto transportResult = transport.sendAndReceive(sps30ShdlcAddr, 0x03, {}, bytesView);
    if (transportResult != Sps30Error::Success)
    {
        return transportResult;
    }

    if (bytesView.size() == 40)
    {
        return Sps30MeasurementData {
                .floatData {
                    .mc_1p0 = embedded::changeEndianessToFloat(data[0]),
                    .mc_2p5 = embedded::changeEndianessToFloat(data[1]),
                    .mc_4p0 = embedded::changeEndianessToFloat(data[2]),
                    .mc_10p0 = embedded::changeEndianessToFloat(data[3]),
                    .nc_0p5 = embedded::changeEndianessToFloat(data[4]),
                    .nc_1p0 = embedded::changeEndianessToFloat(data[5]),
                    .nc_2p5 = embedded::changeEndianessToFloat(data[6]),
                    .nc_4p0 = embedded::changeEndianessToFloat(data[7]),
                    .nc_10p0 = embedded::changeEndianessToFloat(data[8]),
                    .typical_particle_size = embedded::changeEndianessToFloat(data[9])
                },
                .measureInFloat = true
        };
    }
    else
    {
        auto* unsignedData = reinterpret_cast<uint16_t*>(data);
        return Sps30MeasurementData {
                .unsignedData {
                    .mc_1p0 = embedded::changeEndianess(unsignedData[0]),
                    .mc_2p5 = embedded::changeEndianess(unsignedData[1]),
                    .mc_4p0 = embedded::changeEndianess(unsignedData[2]),
                    .mc_10p0 = embedded::changeEndianess(unsignedData[3]),
                    .nc_0p5 = embedded::changeEndianess(unsignedData[4]),
                    .nc_1p0 = embedded::changeEndianess(unsignedData[5]),
                    .nc_2p5 = embedded::changeEndianess(unsignedData[6]),
                    .nc_4p0 = embedded::changeEndianess(unsignedData[7]),
                    .nc_10p0 = embedded::changeEndianess(unsignedData[8]),
                    .typical_particle_size = embedded::changeEndianess(unsignedData[9])
                },
                .measureInFloat = false
        };
    }
}

Sps30Error Sps30Uart::sleep()
{
    return transport.sendAndReceive(sps30ShdlcAddr, 0x10, {}, {});
}

Sps30Error Sps30Uart::wakeUp()
{
    DEBUG_LOG("Sps30Uart processing wakeup request")
    if (const auto result = activateTransport(); result != Sps30Error::Success)
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

std::variant<Sps30Error, Sps30VersionInformation> Sps30Uart::getVersion()
{
    uint8_t data[7];

    auto result = transport.sendAndReceive(sps30ShdlcAddr, 0xd1, {}, data);
    if (result == Sps30Error::Success)
    {
        return Sps30VersionInformation {
                .firmware_major = data[0],
                .firmware_minor = data[1],
                .shdlc = Sps30ShdlcInformation {
                    .hardware_revision = data[3], .shdlc_major = data[5], .shdlc_minor = data[6]
                }
        };
    }
    return result;
}

Sps30Error Sps30Uart::resetSensor()
{
    auto result = transport.sendAndReceive(sps30ShdlcAddr, 0xd3, {}, {});
    if (result == Sps30Error::Success)
    {
        embedded::delay(100);
    }
    return result;
}

}
