#pragma once

#include "ShdlcTransport.h"
#include "Sps30DataTypes.h"

#include <cstdint>
#include <variant>

namespace embedded
{

class Sps30Uart
{
public:
    static constexpr uint8_t maxDeviceInformationlLength = 32;

    explicit Sps30Uart(embedded::PacketUart &uart) : transport(uart) {}

    Sps30Error probe();
    Sps30Error getSerial(Sps30SerialNumber &serial);
    std::variant<Sps30Error, Sps30VersionInformation> getVersion();

    Sps30Error startMeasurement(bool floating = true);
    Sps30Error stopMeasurement();
    std::variant<Sps30Error, Sps30MeasurementData> readMeasurement();

    Sps30Error sleep();
    Sps30Error wakeUp();
    Sps30Error resetSensor();

    std::variant<Sps30Error, uint32_t> getFanAutoCleaningInterval();
    Sps30Error setFanAutoCleaningInterval(uint32_t intervalSeconds);
    Sps30Error startManualFanCleaning();

private:
    ShdlcTransport transport;
};

}
