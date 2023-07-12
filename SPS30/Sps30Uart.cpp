#include "EndianConversion.h"
#include "Sps30Uart.h"
#include "shdlc.h"
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
    DEBUG_LOG("Sps30Uart probe begin")
    wakeUp();
    DEBUG_LOG("Sps30Uart wakeup command sent")
    auto versionTuple = Sps30Uart::getVersion();
    auto &versioninfo = std::get<0>(versionTuple);
    auto &result = std::get<1>(versionTuple);
    if (result == Sps30Error::Success)
    {
        firmwareVersion = (versioninfo.firmware_major << 16) | versioninfo.firmware_minor;
    }
    return result;
}

Sps30Error Sps30Uart::getSerial(char serial[maxSerialLen])
{
    uint8_t paramBuf[] = { 0x03 };

    return transport.SendAndReceive(sps30ShdlcAddr, 0xd0, sizeof(paramBuf), paramBuf,
                                    { (uint8_t*)serial, maxSerialLen });
}

Sps30Error Sps30Uart::startMeasurement(bool floating)
{
    uint8_t paramBuf[2] = { 0x01, floating ? (uint8_t)0x03u : (uint8_t)0x05u };

    return transport.SendAndReceive(sps30ShdlcAddr, 0x00, sizeof(paramBuf), paramBuf);
}

Sps30Error Sps30Uart::stopMeasurement()
{
    return transport.SendAndReceive(
            sps30ShdlcAddr, 0x01, 0, (uint8_t*)nullptr);
}

std::tuple<Sps30Uart::measurementData, Sps30Error> Sps30Uart::readMeasurement()
{
    std::tuple<Sps30Uart::measurementData, Sps30Error> result;
    uint32_t data[10];
    embedded::BytesView bytesView { (uint8_t*)data, sizeof(data) };

    std::get<1>(result) = transport.SendAndReceive(sps30ShdlcAddr, 0x03, 0, (uint8_t*)nullptr, bytesView);
    if (std::get<1>(result) == Sps30Error::Success)
    {
        auto &measurement = std::get<0>(result);
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

    return result;
}

Sps30Error Sps30Uart::sleep()
{
    return transport.SendAndReceive(sps30ShdlcAddr, 0x10, 0, (uint8_t*)nullptr);
}

Sps30Error Sps30Uart::wakeUp()
{
    DEBUG_LOG("Sps30Uart processing wakeup request")
    auto result = transport.ActivateTransport();
    if (result != Sps30Error::Success)
    {
        return result;
    }
    return transport.SendAndReceive(sps30ShdlcAddr, 0x11, 0, (uint8_t*)nullptr);
}

std::tuple<uint32_t, Sps30Error> Sps30Uart::getFanAutoCleaningInterval()
{
    std::tuple<uint32_t, Sps30Error> result;
    uint8_t tx_data[] = { 0x00 };

    uint32_t data;
    std::get<1>(result) = transport.SendAndReceive(
            sps30ShdlcAddr, 0x80, sizeof(tx_data), tx_data, { (uint8_t*)&data, sizeof(data) });
    if (std::get<1>(result) == Sps30Error::Success)
    {
        std::get<0>(result) = embedded::changeEndianess(data);
    }

    return result;
}

Sps30Error Sps30Uart::setFanAutoCleaningInterval(uint32_t interval_seconds)
{
    uint8_t cleaning_command[5];

    cleaning_command[0] = 0x00;
    *reinterpret_cast<uint32_t*>(&cleaning_command[1]) = embedded::changeEndianess(interval_seconds);

    return transport.SendAndReceive(
            sps30ShdlcAddr, 0x80, sizeof(cleaning_command), cleaning_command);
}

Sps30Error Sps30Uart::startManualFanCleaning()
{
    return transport.SendAndReceive(sps30ShdlcAddr, 0x56, 0, (uint8_t*)nullptr);
}

std::tuple<Sps30Uart::versionInformation, Sps30Error> Sps30Uart::getVersion()
{
    std::tuple<Sps30Uart::versionInformation, Sps30Error> result;
    uint8_t data[7];

    std::get<1>(result) = transport.SendAndReceive(
            sps30ShdlcAddr, 0xd1, 0, (uint8_t*)nullptr, { data, sizeof(data) });
    if (std::get<1>(result) == Sps30Error::Success)
    {
        auto &version_information = std::get<0>(result);
        version_information.firmware_major = data[0];
        version_information.firmware_minor = data[1];
        version_information.hardware_revision = data[3];
        version_information.shdlc_major = data[5];
        version_information.shdlc_minor = data[6];
    }
    return result;
}

Sps30Error Sps30Uart::resetSensor()
{
    auto result = transport.Send(sps30ShdlcAddr, 0xd3, 0, nullptr);
    if (result == Sps30Error::Success)
    {
        embedded::delay(100);
    }
    return result;
}

}
