#pragma once

#include <cstdint>

namespace embedded
{

class I2CHelper;

class BMPE280
{
public:
    enum MeasurementMode
    {
        Sleep = 0, Forced, Normal = 3, };

    enum FilterCoeff
    {
        NoFiltering = 0, Filter2, Filter4, Filter8, Filter16, };

    /**
    * Pressure oversampling settings
    */
    enum SamplingRate
    {
        NoSampling = 0, OversamplingX1, OversamplingX2, OversamplingX4, OversamplingX8, OversamplingX16, };

    /**
    * Stand by time between measurements in normal mode
    */
    enum StandbyTime
    {
        Standby500us = 0,      /* stand by time 0.5ms */
        Standby62500us,      /* stand by time 62.5ms */
        Standby125ms,     /* stand by time 125ms */
        Standby250ms,     /* stand by time 250ms */
        Standby500ms,     /* stand by time 500ms */
        Standby1s,    /* stand by time 1s */
        Standby2s,    /* stand by time 2s BMP280, 10ms BME280 */
        Standby4s,    /* stand by time 4s BMP280, 20ms BME280 */
    };

    explicit BMPE280(I2CHelper &device,
                     FilterCoeff filter = Filter16,
                     SamplingRate pressureSampling = OversamplingX16,
                     SamplingRate temperatureSampling = OversamplingX16,
                     SamplingRate humiditySampling = OversamplingX16,
                     StandbyTime interval = Standby250ms)
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

    // temperature in 1/100th of Celsius degree
    // pressure in format Q24.8
    // humidity in format Q22.10
    bool getMeasureData(int32_t &temperature, uint32_t &pressure, uint32_t &humidity);
    // temperature in Celsius
    // pressure in Pascals
    // relative humidity in percents
    bool getMeasureData(float &temperature, float &pressure, float &humidity);

    bool canMeasureHumidity() const { return id == bme280ChipId; }

private:
    int32_t calculateFineTemperature(int32_t rawTemperature) const;

    // Implements formula from Bosch Sensortec | BME280 Data sheet
    // Returns humidity in %RH as unsigned 32 bit integer in Q22.10 format (22 integer and 10 fractional bits).
    // Output value of “47445” represents 47445/1024 = 46.333 %RH
    uint32_t calculateHumidity(int32_t rawHumidity, int32_t fineTemp) const;

    // Implements formula from Bosch Sensortec | BME280 Data sheet
    // Returns pressure in Pa as unsigned 32 bit integer in Q24.8 format (24 integer bits and 8 fractional bits).
    // Output value of “24674867” represents 24674867/256 = 96386.2 Pa = 963.862 hPa
    uint32_t calculatePressure(int32_t rawPressure, int32_t fineTemperature) const;

    bool readTemperatureAndPressureCalibrationData();
    bool readHumidityCalibrationData();
    bool setMeasurementMode(MeasurementMode m);

    static constexpr uint8_t bmp280ChipId = 0x58;
    static constexpr uint8_t bme280ChipId = 0x60;

    struct
    {
        uint16_t T1;
        int16_t T2;
        int16_t T3;
    } temperatureCompensation;

    struct
    {
        uint16_t P1;
        int16_t POther[8];
    } pressureCompensation;

    /* Humidity compensation for BME280 */
    struct
    {
        int16_t H2;
        int16_t H4;
        int16_t H5;
        uint8_t H1;
        uint8_t H3;
        int8_t H6;
    } humidityCompensation;

    uint8_t id;        /* Chip ID */

    I2CHelper &device;

    FilterCoeff filterCoeff;
    SamplingRate oversamplingPressure;
    SamplingRate oversamplingTemperature;
    SamplingRate oversamplingHumidity;
    StandbyTime standbyTime;
};

}
