#include "Sps30i2c.h"
#include "EndianConversion.h"
#include "Delays.h"
#include "Debug.h"

namespace
{

enum class SPS30Command : uint16_t
{
    StartMeasurement = 0x0010,
    StopMeasurement = 0x0104,
    ReadMeasurement = 0x0300,
    GetDataReady = 0x0202,
    AutocleanInterval = 0x8004,
    GetFirmwareVersion = 0xd100,
    GetSerial = 0xd033,
    Reset = 0xd304,
    Sleep = 0x1001,
    ReadDeviceStatusReg = 0xd206,
    StartManualFanCleaning = 0x5607,
    WakeUp = 0x1103
};

constexpr uint16_t StartMeasurementFloat = 0x0300;
constexpr uint16_t StartMeasurementInteger = 0x0500;
constexpr uint16_t StartStopDelay = 20;
constexpr uint16_t CommandDelay = 5;
constexpr uint16_t FlashWriteDalay = 20;

constexpr uint8_t crcLength = 1;
constexpr uint8_t maxReadBufferSize = 64;
constexpr uint8_t maxWriteBufferSize = 32;
constexpr uint8_t I2CBlockSize = 2;

uint8_t calculateSps30Crc(const uint8_t data[2])
{
    uint8_t crc = 0xFF;

    for (int i = 0; i < 2; ++i)
    {
        crc ^= (data[i]);
        for (int bit = 8; bit > 0; --bit)
        {
            if (crc & 0x80)
            {
                crc = (crc << 1) ^ 0x31u;
            }
            else
            {
                crc = (crc << 1);
            }
        }
    }
    return crc;
}
}

namespace embedded
{

Sps30Error Sps30I2C::probe()
{
    Sps30I2C::wakeUp();

    return readVersion();
}

std::variant<Sps30Error, Sps30VersionInformation> Sps30I2C::getVersion()
{
    std::variant<Sps30Error, Sps30VersionInformation> result;
    if (auto err = readVersion(); err == Sps30Error::Success)
    {
        result = Sps30VersionInformation {
                .firmware_major = firmwareVersion[0],
                .firmware_minor = firmwareVersion[1],
                .shdlc= std::nullopt
        };
    }
    else
    {
        result = err;
    }
    return result;
}

Sps30Error Sps30I2C::readVersion()
{
    return sendCommandGetResponce((uint16_t)SPS30Command::GetFirmwareVersion, firmwareVersion, 20);
}

Sps30Error Sps30I2C::getSerial(Sps30SerialNumber &serial)
{
    BytesView serialView { reinterpret_cast<unsigned char*>(serial.serial), sizeof(serial.serial) };
    return sendCommandGetResponce((uint16_t)SPS30Command::GetSerial, serialView);
}

Sps30Error Sps30I2C::startMeasurement(bool floating)
{
    if (!floating && (readVersion() != Sps30Error::Success || firmwareVersion[0] < 2))
    {
        return Sps30Error::UnsupportedCommand;
    }
    measurementInFloat = floating;

#pragma pack(push, 1)
    union
    {
        struct
        {
            uint16_t command;
            uint16_t param;
            uint8_t crc;
        };
        uint8_t buf[5];
    } command {
            {embedded::changeEndianess((uint16_t)SPS30Command::StartMeasurement),
            embedded::changeEndianess(floating ? StartMeasurementFloat : StartMeasurementInteger),
            0}
    };
#pragma pack(pop)

    command.crc = calculateSps30Crc(command.buf+2);
    DEBUG_LOG("Sending Sps30I2C comand: " << embedded::BytesView(command.buf));
    if (!sps30Device.sendSync(command.buf, sizeof(command.buf)))
    {
        return Sps30Error::TransportError;
    }

    embedded::delay(StartStopDelay);
    return Sps30Error::Success;
}

Sps30Error Sps30I2C::stopMeasurement()
{
    if (!sendCommand((uint16_t)SPS30Command::StopMeasurement))
    {
        return Sps30Error::TransportError;
    }

    embedded::delay(StartStopDelay);
    return Sps30Error::Success;
}

bool Sps30I2C::isDataReady()
{
    uint16_t readedFlag;
    auto result = sendCommandGetResponce((uint16_t)SPS30Command::GetDataReady,
                                         { reinterpret_cast<uint8_t*>(&readedFlag), sizeof readedFlag });
    return (result == Sps30Error::Success) && (embedded::changeEndianess(readedFlag) != 0);
}

std::variant<Sps30Error, Sps30MeasurementData> Sps30I2C::readMeasurement()
{
    if (measurementInFloat)
    {
        union
        {
            uint32_t dwords[10];
            uint8_t bytes[40];
        } data;
        const auto error = sendCommandGetResponce((uint16_t)SPS30Command::ReadMeasurement,
                                                  data.bytes);

        if (error == Sps30Error::Success)
        {
            return Sps30MeasurementData {
                    .floatData {
                            .mc_1p0 = embedded::changeEndianessToFloat(data.dwords[0]),
                            .mc_2p5 = embedded::changeEndianessToFloat(data.dwords[1]),
                            .mc_4p0 = embedded::changeEndianessToFloat(data.dwords[2]),
                            .mc_10p0 = embedded::changeEndianessToFloat(data.dwords[3]),
                            .nc_0p5 = embedded::changeEndianessToFloat(data.dwords[4]),
                            .nc_1p0 = embedded::changeEndianessToFloat(data.dwords[5]),
                            .nc_2p5 = embedded::changeEndianessToFloat(data.dwords[6]),
                            .nc_4p0 = embedded::changeEndianessToFloat(data.dwords[7]),
                            .nc_10p0 = embedded::changeEndianessToFloat(data.dwords[8]),
                            .typical_particle_size = embedded::changeEndianessToFloat(data.dwords[9])
                    },
                    .measureInFloat = true
            };
        }
        return error;
    }
    else
    {
        union
        {
            uint16_t words[10];
            uint8_t bytes[20];
        } data;
        const auto error = sendCommandGetResponce((uint16_t)SPS30Command::ReadMeasurement, data.bytes);

        if (error == Sps30Error::Success)
        {
            return Sps30MeasurementData {
                    .unsignedData {
                            .mc_1p0 = embedded::changeEndianess(data.words[0]),
                            .mc_2p5 = embedded::changeEndianess(data.words[1]),
                            .mc_4p0 = embedded::changeEndianess(data.words[2]),
                            .mc_10p0 = embedded::changeEndianess(data.words[3]),
                            .nc_0p5 = embedded::changeEndianess(data.words[4]),
                            .nc_1p0 = embedded::changeEndianess(data.words[5]),
                            .nc_2p5 = embedded::changeEndianess(data.words[6]),
                            .nc_4p0 = embedded::changeEndianess(data.words[7]),
                            .nc_10p0 = embedded::changeEndianess(data.words[8]),
                            .typical_particle_size = embedded::changeEndianess(data.words[9])
                    },
                    .measureInFloat = false
            };
        }
        return error;
    }
}

std::variant<Sps30Error, uint32_t> Sps30I2C::getFanAutoCleaningInterval()
{
    uint32_t data;
    const auto result =
            sendCommandGetResponce((uint16_t)SPS30Command::AutocleanInterval,
                                   { reinterpret_cast<uint8_t*>(&data), sizeof(data) }, CommandDelay);
    if (result == Sps30Error::Success)
    {
        return embedded::changeEndianess(data);
    }

    return result;
}

Sps30Error Sps30I2C::setFanAutoCleaningInterval(uint32_t interval_seconds)
{
    union
    {
        uint32_t seconds;
        uint16_t tmp[2];
    } converter { embedded::changeEndianess(interval_seconds) };
    uint8_t buf[8];
    *((uint16_t*)buf) = embedded::changeEndianess((uint16_t)SPS30Command::AutocleanInterval);
    *((uint16_t*)(buf + 2)) = converter.tmp[0];
    buf[4] = calculateSps30Crc(buf + 2);
    *((uint16_t*)(buf + 5)) = converter.tmp[1];
    buf[7] = calculateSps30Crc(buf + 5);
    DEBUG_LOG("Sending Sps30I2C comand: " << embedded::BytesView(buf, sizeof(buf)));
    if (!sps30Device.sendSync(buf, sizeof(buf)))
    {
        return Sps30Error::TransportError;
    }

    embedded::delay(FlashWriteDalay);
    return Sps30Error::Success;
}

Sps30Error Sps30I2C::startManualFanCleaning()
{
    if (!sendCommand((uint16_t)SPS30Command::StartManualFanCleaning))
    {
        return Sps30Error::TransportError;
    }

    embedded::delay(CommandDelay);
    return Sps30Error::Success;
}

Sps30Error Sps30I2C::resetSensor()
{
    if (!sendCommand((uint16_t)SPS30Command::Reset))
    {
        return Sps30Error::TransportError;
    }
    embedded::delay(200);
    return Sps30Error::Success;
}

Sps30Error Sps30I2C::sleep()
{
    if (readVersion() != Sps30Error::Success || firmwareVersion[0] < 2)
    {
        return Sps30Error::UnsupportedCommand;
    }
    if (!sendCommand((uint16_t)SPS30Command::Sleep))
    {
        return Sps30Error::TransportError;
    }

    embedded::delay(CommandDelay);
    return Sps30Error::Success;
}

Sps30Error Sps30I2C::wakeUp()
{
    if (firmwareVersion[0] < 0x2)
    { //Allow to try if the version is unknown yet
        return Sps30Error::UnsupportedCommand;
    }
    sendCommand((uint16_t)SPS30Command::WakeUp); // workaround of I2C initiate sequence
    if (!sendCommand((uint16_t)SPS30Command::WakeUp))
    {
        return Sps30Error::TransportError;
    }

    embedded::delay(CommandDelay);
    return Sps30Error::Success;
}

std::variant<Sps30Error, uint32_t> Sps30I2C::readDeviceStatusRegister()
{
    if (readVersion() != Sps30Error::Success || firmwareVersion[0] < 2 || firmwareVersion[1] < 2)
    {
        return Sps30Error::UnsupportedCommand;
    }

    union
    {
        uint32_t value;
        uint8_t bytes[4];
    } value;
    const auto result = sendCommandGetResponce((uint16_t)SPS30Command::ReadDeviceStatusReg,
                                               value.bytes, CommandDelay);
    if (result == Sps30Error::Success)
    {
        return embedded::changeEndianess(value.value);
    }
    return result;
}

Sps30Error Sps30I2C::clearDeviceStatusRegister()
{
    if (readVersion() != Sps30Error::Success || firmwareVersion[0] < 2)
    {
        return Sps30Error::UnsupportedCommand;
    }
    if (sendCommand((uint16_t)SPS30Command::ReadDeviceStatusReg))
    {
        embedded::delay(CommandDelay);
        return Sps30Error::Success;
    }
    return Sps30Error::TransportError;
}

Sps30Error Sps30I2C::readBytesWithCRC(const BytesView bytes)
{
    const auto readSize = static_cast<uint16_t>(bytes.size() * (1 + crcLength / I2CBlockSize));
    uint8_t buf8[maxReadBufferSize];

    if (!sps30Device.receiveSync(buf8, readSize))
    {
        return Sps30Error::TransportError;
    }
    DEBUG_LOG("Recieved " << (int)readSize << " bytes from I2C:" << embedded::BytesView(buf8, readSize))

    auto data = bytes.begin();
    for (uint16_t i = 0, j = 0; i < readSize && j < bytes.size(); ++i)
    {
        auto crc = calculateSps30Crc(&buf8[i]);
        if (crc != buf8[i + I2CBlockSize])
        {
            DEBUG_LOG("Calculated CRC is " << crc << ", expected " << buf8[i + I2CBlockSize])
            return Sps30Error::DataError;
        }

        data[j++] = buf8[i++];
        data[j++] = buf8[i++];
    }

    return Sps30Error::Success;
}

bool Sps30I2C::sendCommand(uint16_t command)
{
    union
    {
        uint16_t cmd;
        uint8_t bytes[2];
    } cmd { .cmd = embedded::changeEndianess(command) };
    DEBUG_LOG("Sending Sps30I2C comand: " << embedded::BytesView(cmd.bytes))
    return sps30Device.sendSync(cmd.bytes, sizeof(cmd.bytes));
}

Sps30Error Sps30I2C::sendCommandGetResponce(uint16_t cmd, const embedded::BytesView bytes, uint32_t delay_ms)
{
    if (!sendCommand(cmd))
    {
        return Sps30Error::TransportError;
    }

    if (delay_ms)
    {
        embedded::delay(delay_ms);
    }
    return readBytesWithCRC(bytes);
}

}
