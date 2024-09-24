#pragma once

#include "IBmeDevice.h"

namespace embedded
{

class GpioDigitalPin;
class SpiDevice;

class SpiHelper : public IBmeDevice
{
public:
    SpiHelper(SpiDevice &spiDevice, GpioDigitalPin &csPin)
    : csPin(csPin)
    , device(spiDevice)
    {}

    bool writeByte(uint8_t memAddress, uint8_t value) const override;

    bool readByte(uint8_t memAddress, uint8_t &value) const override
    {
        return readBytes(memAddress, {&value, 1});
    }

    bool readBytes(uint8_t memAddress, BytesView bytes) const override;

private:
    GpioDigitalPin &csPin;
    SpiDevice &device;
};
}
