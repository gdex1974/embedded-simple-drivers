#pragma once

#include "I2CDevice.h"
#include "MemoryView.h"

namespace embedded
{

class I2CHelper
{
public:
    I2CHelper(I2CBus &bus, uint16_t address)
            : device(bus, address) {}

    bool readByte(uint8_t memAddress, uint8_t &value) const
    {
        return readBytes(memAddress, BytesView(&value, 1));
    }

    bool writeByte(uint8_t memAddress, uint8_t value) const
    {
        uint8_t buffer[2] { memAddress, value };
        return device.sendSync(buffer, sizeof(buffer));
    }

    bool readBytes(uint8_t memAddress, BytesView bytes) const
    {
        return device.sendThenReceive(&memAddress, 1, bytes.begin(), bytes.size());
    }

private:
    I2CDevice device;
};

}
