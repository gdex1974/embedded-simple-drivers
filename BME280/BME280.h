#pragma once

#include <cstdint>

namespace embedded
{

class I2CHelper;

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

    /**
    * Stand by time between measurements in normal mode
    */
    enum class StandbyTime : uint8_t
    {
        Standby500us = 0      /* stand by time 0.5ms */
        , Standby62500us      /* stand by time 62.5ms */
        , Standby125ms     /* stand by time 125ms */
        , Standby250ms     /* stand by time 250ms */
        , Standby500ms     /* stand by time 500ms */
        , Standby1s    /* stand by time 1s */
        , Standby2s    /* stand by time 2s BMP280, 10ms BME280 */
        , Standby4s    /* stand by time 4s BMP280, 20ms BME280 */
    };

    explicit BMPE280(I2CHelper &device,
                     FilteringMode filter = FilteringMode::Filter16,
                     SamplingRate pressureSampling = SamplingRate::OversamplingX16,
                     SamplingRate temperatureSampling = SamplingRate::OversamplingX16,
                     SamplingRate humiditySampling = SamplingRate::OversamplingX16,
                     StandbyTime interval = StandbyTime::Standby250ms)
            : device(device), filterCoeff(filter), oversamplingPressure(pressureSampling), oversamplingTemperature(
            temperatureSampling), oversamplingHumidity(humiditySampling), standbyTime(interval)
    {

    }

    // returns zero if successful or a number of failed stage
    int init();
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

    bool canMeasureHumidity() const { return id == bme280ChipId; }

private:
    int32_t calculateFineTemperature(int32_t rawTemperature) const;
    uint32_t calculateFineHumidity(int32_t rawHumidity, int32_t fineTemperature) const;
    uint32_t calculateFinePressure(int32_t rawPressure, int32_t fineTemperature) const;

    bool readTemperatureAndPressureCalibrationData();
    bool readHumidityCalibrationData();
    bool setMeasurementMode(MeasurementMode mode);

    static constexpr uint8_t bmp280ChipId = 0x58;
    static constexpr uint8_t bme280ChipId = 0x60;

    struct
    {
        uint16_t T1;
        int16_t T2;
        int16_t T3;
    } temperatureCompensation{};

    struct
    {
        uint16_t P1;
        int16_t POther[8];
    } pressureCompensation{};

    struct
    {
        int16_t H2;
        int16_t H4;
        int16_t H5;
        uint8_t H1;
        uint8_t H3;
        int8_t H6;
    } humidityCompensation{};

    uint8_t id{};        /* Chip ID */

    I2CHelper &device;

    FilteringMode filterCoeff;
    SamplingRate oversamplingPressure;
    SamplingRate oversamplingTemperature;
    SamplingRate oversamplingHumidity;
    StandbyTime standbyTime;
};

}
