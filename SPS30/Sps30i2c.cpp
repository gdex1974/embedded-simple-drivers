/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "Sps30i2c.h"
#include "EndianConversion.h"
#include "Delays.h"
#include "Debug.h"

namespace
{
    enum class SPS30Command : uint16_t
    {
        cmdStartMeasurement = 0x0010,
        cmdStopMeasurement = 0x0104,
        cmdReadMeasurement = 0x0300,
        cmdGetDataReady = 0x0202,
        cmdAutocleanInterval = 0x8004,
        cmdGetFirmwareVersion = 0xd100,
        cmdGetSerial = 0xd033,
        cmdReset = 0xd304,
        cmdSleep = 0x1001,
        cmdReadDeviceStatusReg = 0xd206,
        cmdStartManualFanCleaning = 0x5607,
        cmdWakeUp = 0x1103
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
                    crc = (crc << 1) ^ 0x31u;
                else
                    crc = (crc << 1);
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
            .firmware_major = static_cast<uint8_t>((firmwareVersion & 0xff00) >> 8),
            .firmware_minor = static_cast<uint8_t>((firmwareVersion & 0x00ff)),
     std::nullopt
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
    Sps30Error result = Sps30Error::Success;
    if (firmwareVersion == 0)
    {
        result = sendCommandGetResponce((uint16_t)SPS30Command::cmdGetFirmwareVersion,
                                        reinterpret_cast<uint8_t*>(&firmwareVersion),
                                        sizeof(firmwareVersion), 20);
        firmwareVersion = embedded::changeEndianess(firmwareVersion);
    }
    return result;
}

Sps30Error Sps30I2C::getSerial(char serial[maxSerialLen])
{
    return sendCommandGetResponce((uint16_t)SPS30Command::cmdGetSerial, (uint8_t*)serial, maxSerialLen);
}

Sps30Error Sps30I2C::startMeasurement(bool floating)
{
    if (!floating && (readVersion() != Sps30Error::Success || firmwareVersion < 0x200))
        return Sps30Error::UnsupportedCommand;
    measurementInFloat = floating;
    uint8_t buf[5];
    *((uint16_t*)buf) = embedded::changeEndianess((uint16_t)SPS30Command::cmdStartMeasurement);
    *((uint16_t*)(buf + 2)) =
            floating ? embedded::changeEndianess(StartMeasurementFloat) : embedded::changeEndianess(
                    StartMeasurementInteger);
    buf[4] = calculateSps30Crc(buf + 2);
    DEBUG_LOG("Sending Sps30I2C comand: " << embedded::BytesView(buf, sizeof(buf)));
    if (!sps30Device.sendSync(buf, sizeof(buf)))
        return Sps30Error::TransportError;

    embedded::delay(StartStopDelay);
    return Sps30Error::Success;
}

Sps30Error Sps30I2C::stopMeasurement()
{
    if (!sendCommand((uint16_t)SPS30Command::cmdStopMeasurement))
        return Sps30Error::TransportError;

    embedded::delay(StartStopDelay);
    return Sps30Error::Success;
}

bool Sps30I2C::isDataReady()
{
    uint16_t readedFlag;
    auto result = sendCommandGetResponce((uint16_t)SPS30Command::cmdGetDataReady,
                                         reinterpret_cast<uint8_t*>(&readedFlag), sizeof readedFlag);
    return (result == Sps30Error::Success) && (embedded::changeEndianess(readedFlag) != 0);
}

std::variant<Sps30Error, Sps30MeasurementData> Sps30I2C::readMeasurement()
{
    if (measurementInFloat)
    {
        uint32_t data[10];
        const auto error = sendCommandGetResponce((uint16_t)SPS30Command::cmdReadMeasurement,
                                       reinterpret_cast<uint8_t*>(data), sizeof(data));

        if (error == Sps30Error::Success)
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
        return error;
    }
    else
    {
        uint16_t data[10];
        const auto error = sendCommandGetResponce((uint16_t)SPS30Command::cmdReadMeasurement,
                                       reinterpret_cast<uint8_t*>(data), sizeof(data));

        if (error == Sps30Error::Success)
        {
            return Sps30MeasurementData {
                    .unsignedData {
                            .mc_1p0 = embedded::changeEndianess(data[0]),
                            .mc_2p5 = embedded::changeEndianess(data[1]),
                            .mc_4p0 = embedded::changeEndianess(data[2]),
                            .mc_10p0 = embedded::changeEndianess(data[3]),
                            .nc_0p5 = embedded::changeEndianess(data[4]),
                            .nc_1p0 = embedded::changeEndianess(data[5]),
                            .nc_2p5 = embedded::changeEndianess(data[6]),
                            .nc_4p0 = embedded::changeEndianess(data[7]),
                            .nc_10p0 = embedded::changeEndianess(data[8]),
                            .typical_particle_size = embedded::changeEndianess(data[9])
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
            sendCommandGetResponce((uint16_t)SPS30Command::cmdAutocleanInterval, reinterpret_cast<uint8_t*>(&data),
                                   sizeof(data), CommandDelay);
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
    *((uint16_t*)buf) = embedded::changeEndianess((uint16_t)SPS30Command::cmdAutocleanInterval);
    *((uint16_t*)(buf + 2)) = converter.tmp[0];
    buf[4] = calculateSps30Crc(buf + 2);
    *((uint16_t*)(buf + 5)) = converter.tmp[1];
    buf[7] = calculateSps30Crc(buf + 5);
    DEBUG_LOG("Sending Sps30I2C comand: " << embedded::BytesView(buf, sizeof(buf)));
    if (!sps30Device.sendSync(buf, sizeof(buf)))
        return Sps30Error::TransportError;

    embedded::delay(FlashWriteDalay);
    return Sps30Error::Success;
}

Sps30Error Sps30I2C::startManualFanCleaning()
{
    if (!sendCommand((uint16_t)SPS30Command::cmdStartManualFanCleaning))
        return Sps30Error::TransportError;

    embedded::delay(CommandDelay);
    return Sps30Error::Success;
}

Sps30Error Sps30I2C::resetSensor()
{
    if (!sendCommand((uint16_t)SPS30Command::cmdReset))
        return Sps30Error::TransportError;
    embedded::delay(200);
    return Sps30Error::Success;
}

Sps30Error Sps30I2C::sleep()
{
    if (readVersion() != Sps30Error::Success || firmwareVersion < 0x200)
        return Sps30Error::UnsupportedCommand;
    if (!sendCommand((uint16_t)SPS30Command::cmdSleep))
        return Sps30Error::TransportError;

    embedded::delay(CommandDelay);
    return Sps30Error::Success;
}

Sps30Error Sps30I2C::wakeUp()
{
    if (firmwareVersion && firmwareVersion < 0x200) //Allow to try if the version is unknown yet
        return Sps30Error::UnsupportedCommand;
    sendCommand((uint16_t)SPS30Command::cmdWakeUp); // workaround of I2C initiate sequence
    if (!sendCommand((uint16_t)SPS30Command::cmdWakeUp))
        return Sps30Error::TransportError;

    embedded::delay(CommandDelay);
    return Sps30Error::Success;
}

std::variant<Sps30Error, uint32_t> Sps30I2C::readDeviceStatusRegister()
{
    if (readVersion() != Sps30Error::Success || firmwareVersion < 0x202)
    {
        return Sps30Error::UnsupportedCommand;
    }

    uint32_t value;
    const auto result = sendCommandGetResponce((uint16_t)SPS30Command::cmdReadDeviceStatusReg,
                                                 reinterpret_cast<uint8_t*>(&value), sizeof(value), CommandDelay);
    if (result == Sps30Error::Success)
    {
        return embedded::changeEndianess(value);
    }
    return result;
}

Sps30Error Sps30I2C::clearDeviceStatusRegister()
{
    if (readVersion() != Sps30Error::Success || firmwareVersion < 0x200)
        return Sps30Error::UnsupportedCommand;
    if (sendCommand((uint16_t)SPS30Command::cmdReadDeviceStatusReg))
    {
        embedded::delay(CommandDelay);
        return Sps30Error::Success;
    }
    return Sps30Error::TransportError;
}

Sps30Error Sps30I2C::readBytesWithCRC(uint8_t* data, uint16_t size)
{
    auto readSize = static_cast<uint16_t>(size + size * crcLength / I2CBlockSize);
    uint8_t buf8[maxReadBufferSize];

    if (!sps30Device.receiveSync(buf8, readSize))
        return Sps30Error::TransportError;
    DEBUG_LOG("Recieved " << (int)readSize << " bytes from I2C:" << embedded::BytesView(buf8, readSize))

    for (int i = 0, j = 0; i < readSize; ++i)
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
    uint16_t bigEndianCmd = embedded::changeEndianess(command);
    DEBUG_LOG("Sending Sps30I2C comand: "
                      << embedded::BytesView(reinterpret_cast<uint8_t*>(&bigEndianCmd), sizeof(bigEndianCmd)))
    return sps30Device.sendSync(reinterpret_cast<const uint8_t*>(&bigEndianCmd), sizeof(bigEndianCmd));
}

Sps30Error Sps30I2C::sendCommandGetResponce(uint16_t cmd, uint8_t* bytes, uint16_t size, uint32_t delay_ms)
{
    if (!sendCommand(cmd))
        return Sps30Error::TransportError;

    if (delay_ms)
    {
        embedded::delay(delay_ms);
    }
    return readBytesWithCRC(bytes, size);
}

}
