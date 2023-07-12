#include "BME280.h"
#include "Delays.h"
#include "Debug.h"
#include "I2CHelper.h"
#include <algorithm>
#include <cstring>

namespace
{
    constexpr uint8_t statusRegister = 0xF3;
    constexpr uint8_t controlRegister = 0xF4;
}

namespace embedded
{

bool BMPE280::readTemperatureAndPressureCalibrationData()
{
    uint16_t buf[12];
    if (device.readBytes(0x88, (uint8_t*)buf, sizeof(buf)))
    {
        temperatureCompensation.T1 = buf[0];
        temperatureCompensation.T2 = (int16_t)buf[1];
        temperatureCompensation.T3 = (int16_t)buf[2];
        pressureCompensation.P1 = buf[3];
        auto* pOthers = (int16_t*)&buf[4];
        for (int i = 0; i < 8; ++i, ++pOthers)
        {
            pressureCompensation.POther[i] = *pOthers;
        }
        return true;
    }

    return false;
}

bool BMPE280::readHumidityCalibrationData()
{
    uint8_t buf[7];

    if (device.readByte(0xa1, humidityCompensation.H1)
        && device.readBytes(0xe1, buf, sizeof(buf)))
    {
        std::memcpy(&humidityCompensation.H2, buf, sizeof(humidityCompensation.H2));
        humidityCompensation.H3 = buf[2];
        humidityCompensation.H4 = ((int16_t)(int8_t)buf[3] << 4) | (buf[4] & 0xf);
        humidityCompensation.H5 = ((int16_t)(int8_t)buf[5] << 4) | (buf[4] >> 4);
        humidityCompensation.H6 = (int8_t)buf[6];

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
    DEBUG_LOG("Initializing BME280")
    if (!device.readByte(0xD0, id))
    {
        return 1;
    }

    DEBUG_LOG("Chip id: " << id);
    if (id != bmp280ChipId && id != bme280ChipId)
    {
        return 2;
    }

    if (!reset())
    {
        return 3;
    }

    uint8_t status;
    auto timestamp = embedded::getMillisecondTicks() + 500;
    do
    {
        if (device.readByte(statusRegister, status) && (status & 1) == 0)
            break;
        embedded::delay(10);
    } while (embedded::getMillisecondTicks() < timestamp);

    if (status & 1)
        return 4;

    if (!readTemperatureAndPressureCalibrationData())
    {
        return 5;
    }

    if (id == bme280ChipId && !readHumidityCalibrationData())
    {
        return 6;
    }

    uint8_t config = (standbyTime << 5) | (filterCoeff << 2);
    if (!device.writeByte(0xF5, config))
    {
        return 7;
    }

    if (id == bme280ChipId)
    {
        // Write crtl hum reg first, only active after write to controlRegister.
        if (!device.writeByte(0xF2, oversamplingHumidity))
        {
            return 8;
        }
    }

    uint8_t ctrl = (oversamplingTemperature << 5) | (oversamplingPressure << 2) | MeasurementMode::Sleep;
    if (!device.writeByte(controlRegister, ctrl))
    {
        return 9;
    }

    return 0;
}

bool BMPE280::setMeasurementMode(MeasurementMode m)
{
    uint8_t ctrl;
    if (!device.readByte(controlRegister, ctrl))
        return false;
    ctrl &= ~0b11;  // clear two lower bits
    ctrl |= m;
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
            return (status & 3) == MeasurementMode::Forced;
        }
    }
    return false;
}

inline int32_t BMPE280::calculateFineTemperature(int32_t rawTemperature) const
{
    int32_t var1 = ((((rawTemperature >> 3) - ((int32_t)temperatureCompensation.T1 << 1)))
                    * (int32_t)temperatureCompensation.T2) >> 11;
    int32_t var2 = (((((rawTemperature >> 4) - (int32_t)temperatureCompensation.T1)
                      * ((rawTemperature >> 4) - (int32_t)temperatureCompensation.T1)) >> 12)
                    * (int32_t)temperatureCompensation.T3) >> 14;

    return var1 + var2;
}

uint32_t BMPE280::calculatePressure(int32_t rawPressure, int32_t fineTemperature) const
{
    int64_t var1 = (int64_t)fineTemperature - 128000;
    int64_t var2 = var1 * var1 * pressureCompensation.POther[4];
    var2 = var2 + ((var1 * pressureCompensation.POther[3]) << 17);
    var2 = var2 + (((int64_t)pressureCompensation.POther[2]) << 35);
    var1 = ((var1 * var1 * pressureCompensation.POther[1]) >> 8) + ((var1 * pressureCompensation.POther[0]) << 12);
    var1 = (((int64_t)1 << 47) + var1) * pressureCompensation.P1 >> 33;

    if (var1 == 0)
    {
        return 0;  // avoid exception caused by division by zero
    }

    int64_t p = 1048576 - rawPressure;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (pressureCompensation.POther[7] * (p >> 13) * (p >> 13)) >> 25;
    var2 = (pressureCompensation.POther[6] * p) >> 19;

    p = ((p + var1 + var2) >> 8) + ((int64_t)pressureCompensation.POther[5] << 4);
    return p;
}

uint32_t BMPE280::calculateHumidity(int32_t rawHumidity, int32_t fineTemp) const
{
    int32_t value = fineTemp - 76800;
    value = ((((rawHumidity << 14) - ((int32_t)humidityCompensation.H4 << 20)
               - humidityCompensation.H5 * value) + 16384) >> 15)
            * (((((((value * humidityCompensation.H6) >> 10) * (((value * humidityCompensation.H3) >> 11) + 32768))
            >> 10) + 2097152) * humidityCompensation.H2 + 8192) >> 14);
    value -= (((((value >> 15) * (value >> 15)) >> 7) * humidityCompensation.H1) >> 4);
    value = std::max(value, (int32_t)0);
    value = std::min(value, (int32_t)419430400);
    return value >> 12;
}

bool BMPE280::getMeasureData(int32_t &temperature, uint32_t &pressure, uint32_t &humidity)
{
    uint8_t data[8];

    size_t size = canMeasureHumidity() ? 8 : 6;
    if (!device.readBytes(0xf7, data, size))
    {
        return false;
    }

    auto rawPressure = (int32_t(data[0]) << 16 | data[1] << 8 | data[2]) >> 4;
    auto rawTemperature = (int32_t(data[3]) << 16 | data[4] << 8 | data[5]) >> 4;

    auto fineTemperature = calculateFineTemperature(rawTemperature);
    temperature = (fineTemperature * 5 + 128) >> 8;
    pressure = calculatePressure(rawPressure, fineTemperature);

    if (canMeasureHumidity())
    {
        auto rawHumidity = int32_t(data[6]) << 8 | data[7];
        humidity = calculateHumidity(rawHumidity, fineTemperature);
    }
    else
    {
        humidity = 0;
    }

    return true;
}

bool BMPE280::getMeasureData(float &temperature, float &pressure, float &humidity)
{
    int32_t fixedTemperature;
    uint32_t fixedPressure;
    uint32_t fixedHumidity;
    if (getMeasureData(fixedTemperature, fixedPressure, fixedHumidity))
    {
        temperature = (float)fixedTemperature / 100.f;
        pressure = (float)fixedPressure / 256.f;
        humidity = (float)fixedHumidity / 1024.f;
        return true;
    }

    return false;
}

}
