#include "BME280.h"
#include "I2CHelper.h"

#include "Delays.h"
#include "PersistentStorage.h"

#include <algorithm>


namespace
{
constexpr uint8_t ptCalibrationBaseAddress = 0x88;
constexpr uint8_t ptCalibrationLastAddress = 0x9F;
constexpr uint8_t humidityCalibrationH1Address = 0xA1;
constexpr uint8_t humidityCalibrationHxBaseAddress = 0xE1;
constexpr uint8_t ptDataBaseAddress = 0xF7;

constexpr uint8_t controlHumidityRegister = 0xF2;
constexpr uint8_t statusRegister = 0xF3;
constexpr uint8_t controlRegister = 0xF4;

constexpr unsigned int maxWaitMilliseconds = 500;

template<typename T>
T square(T value)
{
    return value * value;
}
}

namespace embedded
{

bool BMPE280::readPTCalibrationData()
{
    auto &ptCompensation = calibrationData.ptCompensation;

    union
    {
        std::array<uint8_t, ptCalibrationLastAddress - ptCalibrationBaseAddress + 1> bytes;
        CalibrationData::PTCompensationData data;
    } buf;

    static_assert(sizeof(buf) == sizeof(buf.bytes), "PT compensation data size mismatch with estimation");
    static_assert(sizeof(buf) == sizeof(buf.data), "PT compensation data size mismatch with estimation");
    if (device.readBytes(ptCalibrationBaseAddress, buf.bytes))
    {
        ptCompensation = buf.data;
        return true;
    }

    return false;
}

bool BMPE280::readHumidityCalibrationData()
{
    union
    {
        std::array<uint8_t, (0xE7 - 0xE1 + 1) + 1> bytes; // 0xE7 is the last address of the humidity compensation data
#pragma pack(push, 1)
        struct
        {
            uint8_t H1;
            int16_t H2;
            uint8_t H3;
            int8_t H4high;
            uint8_t H45low;
            int8_t H5high;
            int8_t H6;
        } data;
#pragma pack(pop)
    } buf;
    static_assert(sizeof(buf) == sizeof(buf.bytes), "Humidity compensation data size mismatch with estimation");
    static_assert(sizeof(buf) == sizeof(buf.data), "Humidity compensation data size mismatch with estimation");

    if (device.readByte(humidityCalibrationH1Address, buf.bytes[0])
        && device.readBytes(humidityCalibrationHxBaseAddress, { buf.bytes.begin() + 1, buf.bytes.size() - 1 }))
    {
        const auto compH4 = static_cast<int16_t>(buf.data.H4high << 4 | (buf.data.H45low & 0xf));
        const auto compH5 = static_cast<int16_t>(buf.data.H5high << 4 | (buf.data.H45low >> 4));
        calibrationData.humidityCompensation = {
                buf.data.H1, buf.data.H2, buf.data.H3, compH4, compH5, buf.data.H6
        };
        return true;
    }

    return false;
}

bool BMPE280::reset()
{
    return device.writeByte(0xE0, 0xB6);
}

int BMPE280::init()
{
    uint8_t chipID;
    if (!device.readByte(0xD0, chipID))
    {
        return 1;
    }

    if (chipID != bmp280ChipId && chipID != bme280ChipId)
    {
        return 2;
    }

    if (!reset())
    {
        return 3;
    }

    uint8_t status;
    auto timestamp = embedded::getMillisecondTicks() + maxWaitMilliseconds;
    do
    {
        if (device.readByte(statusRegister, status) && (status & 1) == 0)
        {
            break;
        }
        embedded::delay(10);
    } while (embedded::getMillisecondTicks() < timestamp);

    if (status & 1)
    {
        return 4;
    }

    if (!readPTCalibrationData())
    {
        return 5;
    }

    if (chipID == bme280ChipId && !readHumidityCalibrationData())
    {
        return 6;
    }

    const uint8_t config = (uint8_t(standbyTime) << 5) | (uint8_t(filteringMode) << 2);
    if (!device.writeByte(0xF5, config))
    {
        return 7;
    }

    if (chipID == bme280ChipId)
    {
        if (!device.writeByte(controlHumidityRegister, uint8_t(oversamplingHumidity)))
        {
            return 8;
        }
    }

    const uint8_t ctrl = (uint8_t(oversamplingTemperature) << 5) | (uint8_t(oversamplingPressure) << 2) |
            uint8_t(MeasurementMode::Sleep);
    if (!device.writeByte(controlRegister, ctrl))
    {
        return 9;
    }

    return 0;
}

bool BMPE280::setMeasurementMode(MeasurementMode mode)
{
    uint8_t ctrl;
    if (!device.readByte(controlRegister, ctrl))
    {
        return false;
    }
    ctrl &= ~0b11;  // clear two lower bits
    ctrl |= uint8_t(mode);
    if (!device.writeByte(controlRegister, ctrl))
    {
        return false;
    }
    return true;
}

bool BMPE280::isMeasuring()
{
    uint8_t status;
    if (device.readByte(statusRegister, status))
    {
        if (status & (1 << 3))
        {
            return true;
        }
        if (device.readByte(controlRegister, status))
        {
            return (status & 3) == uint8_t(MeasurementMode::Forced);
        }
    }
    return false;
}

inline int32_t BMPE280::calculateFineTemperature(const int32_t rawTemperature) const
{
    const auto &ptCompensation = calibrationData.ptCompensation;
    // See datasheet for Bosch Sensortec BME280
    const int32_t var1 = ((((rawTemperature >> 3) - ((int32_t)ptCompensation.compT1 << 1)))
                    * (int32_t)ptCompensation.compT2) >> 11;
    const int32_t var2 = ((square((rawTemperature >> 4) - ptCompensation.compT1) >> 12) * ptCompensation.compT3) >> 14;

    return var1 + var2;
}

uint32_t BMPE280::calculateFinePressure(const int32_t rawPressure, const int32_t fineTemperature) const
{
    auto &ptCompensation = calibrationData.ptCompensation;
    // See datasheet for Bosch Sensortec BME280
    int64_t var1 = fineTemperature - 128000;
    const auto var1Squared = square(var1);
    int64_t var2 = var1Squared * ptCompensation.compP2to9[4];
    var2 = var2 + ((var1 * ptCompensation.compP2to9[3]) << 17);
    var2 = var2 + (((int64_t)ptCompensation.compP2to9[2]) << 35);
    var1 = ((var1Squared * ptCompensation.compP2to9[1]) >> 8) + ((var1 * ptCompensation.compP2to9[0]) << 12);
    var1 = (((int64_t)1 << 47) + var1) * ptCompensation.compP1 >> 33;

    if (var1 == 0)
    {
        return 0;  // avoid exception caused by division by zero
    }

    int64_t pressure = 1048576 - rawPressure;
    pressure = (((pressure << 31) - var2) * 3125) / var1;
    var1 = (ptCompensation.compP2to9[7] * square(pressure >> 13)) >> 25;
    var2 = (ptCompensation.compP2to9[6] * pressure) >> 19;

    pressure = ((pressure + var1 + var2) >> 8) + (int64_t(ptCompensation.compP2to9[5]) << 4);
    return pressure;
}

uint32_t BMPE280::calculateFineHumidity(const int32_t rawHumidity, const int32_t fineTemperature) const
{
    auto &humidityCompensation = *calibrationData.humidityCompensation;
    // See datasheet for Bosch Sensortec BME280
    int32_t value = fineTemperature - 76800;
    value = ((((rawHumidity << 14) - (int32_t(humidityCompensation.compH4) << 20)
               - humidityCompensation.compH5 * value) + 16384) >> 15)
            * (((((((value * humidityCompensation.compH6) >> 10) * (((value * humidityCompensation.compH3) >> 11) + 32768))
            >> 10) + 2097152) * humidityCompensation.compH2 + 8192) >> 14);
    value -= ((square(value >> 15) >> 7) * humidityCompensation.compH1) >> 4;
    value = std::max(value, 0);
    value = std::min(value, 419430400);
    return value >> 12;
}

std::optional<BMPE280::MeasurementData> BMPE280::getMeasureData()
{
    constexpr uint8_t readSize = 8;
    std::array<uint8_t, readSize> data;

    if (!device.readBytes(ptDataBaseAddress,
                          {data.begin(), uint16_t(canMeasureHumidity() ? readSize : readSize - 2)}))
    {
        return std::nullopt;
    }

    auto rawPressure = (int32_t(data[0]) << 16 | data[1] << 8 | data[2]) >> 4;
    auto rawTemperature = (int32_t(data[3]) << 16 | data[4] << 8 | data[5]) >> 4;

    auto fineTemperature = calculateFineTemperature(rawTemperature);
    const auto temperature = (fineTemperature * 5 + 128) >> 8;
    const auto pressure = calculateFinePressure(rawPressure, fineTemperature);

    uint32_t humidity = 0;
    if (canMeasureHumidity())
    {
        auto rawHumidity = int32_t(data[6]) << 8 | data[7];
        humidity = calculateFineHumidity(rawHumidity, fineTemperature);
    }

    return MeasurementData{temperature, pressure, humidity};
}

bool BMPE280::loadCalibrationData(PersistentStorage &storage, std::string_view name)
{
    if (const auto storedData = storage.get<decltype(calibrationData)>(name))
    {
        calibrationData = *storedData;
        return true;
    }
    return false;
}

bool BMPE280::saveCalibrationData(PersistentStorage &storage, std::string_view name)
{
    return storage.set(name, calibrationData);
}

}
