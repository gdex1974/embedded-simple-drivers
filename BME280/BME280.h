#pragma once

#include <cstdint>
#include <array>
#include <optional>

namespace embedded
{

class I2CHelper;
class PersistentStorage;

class BMPE280
{
public:
    enum class MeasurementMode : uint8_t
    {
        Sleep = 0
        , Forced
        , Normal = 3
    };

    enum class FilteringMode : uint8_t
    {
        NoFiltering = 0
        , Filter2
        , Filter4
        , Filter8
        , Filter16
    };

    enum class SamplingRate : uint8_t
    {
        NoOversampling = 0
        , OversamplingX1
        , OversamplingX2
        , OversamplingX4
        , OversamplingX8
        , OversamplingX16
    };

    enum class StandbyTime : uint8_t
    {
        Standby500us = 0
        , Standby62500us
        , Standby125ms
        , Standby250ms
        , Standby500ms
        , Standby1s
        , Standby2s
        , Standby4s
    };

    explicit BMPE280(I2CHelper &device,
                     FilteringMode filter = FilteringMode::Filter16,
                     SamplingRate pressureSampling = SamplingRate::OversamplingX16,
                     SamplingRate temperatureSampling = SamplingRate::OversamplingX16,
                     SamplingRate humiditySampling = SamplingRate::OversamplingX16,
                     StandbyTime interval = StandbyTime::Standby250ms)
            : device(device), filteringMode(filter), oversamplingPressure(pressureSampling), oversamplingTemperature(
            temperatureSampling), oversamplingHumidity(humiditySampling), standbyTime(interval)
    {

    }

    // returns zero if successful or a number of failed stage
    int init();
    bool loadCalibrationData(PersistentStorage &storage, std::string_view name);
    bool saveCalibrationData(PersistentStorage &storage, std::string_view name);
    bool reset();

    bool startMeasurement(bool continuous = false)
    {
        return setMeasurementMode(continuous ? MeasurementMode::Normal : MeasurementMode::Forced);
    }

    bool stopMeasurement()
    {
        return setMeasurementMode(MeasurementMode::Sleep);
    }

    bool isMeasuring();

    // temperature in Celsius degree * 100
    // pressure in Pascals * 256
    // humidity in Percents * 1024
    bool getMeasureData(int32_t &temperature, uint32_t &pressure, uint32_t &humidity);
    // temperature in Celsius
    // pressure in Pascals
    // relative humidity in Percents
    bool getMeasureData(float &temperature, float &pressure, float &humidity);

    bool canMeasureHumidity() const { return calibrationData.humidityCompensation.has_value(); }

private:
    int32_t calculateFineTemperature(int32_t rawTemperature) const;
    uint32_t calculateFineHumidity(int32_t rawHumidity, int32_t fineTemperature) const;
    uint32_t calculateFinePressure(int32_t rawPressure, int32_t fineTemperature) const;

    bool readPTCalibrationData();
    bool readHumidityCalibrationData();
    bool setMeasurementMode(MeasurementMode mode);

    static constexpr uint8_t bmp280ChipId = 0x58;
    static constexpr uint8_t bme280ChipId = 0x60;

    struct CalibrationData
    {
        struct PTCompensationData
        {
            uint16_t compT1;
            int16_t compT2;
            int16_t compT3;
            uint16_t compP1;
            std::array<int16_t, 8> compP2to9;
        } ptCompensation {};

        struct HumidityCompensationData
        {
            uint8_t compH1;
            int16_t compH2;
            uint8_t compH3;
            int16_t compH4;
            int16_t compH5;
            int8_t compH6;
        };
        std::optional<HumidityCompensationData> humidityCompensation;
    } calibrationData;

    I2CHelper &device;

    FilteringMode filteringMode;
    SamplingRate oversamplingPressure;
    SamplingRate oversamplingTemperature;
    SamplingRate oversamplingHumidity;
    StandbyTime standbyTime;
};

}
