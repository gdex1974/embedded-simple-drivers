#pragma once

#include "I2CDevice.h"

namespace embedded
{

class I2CHelper
{
public:
    I2CHelper(I2CBus &bus, uint16_t address)
            : device(bus, address) {}

    bool readByte(uint8_t memAddress, uint8_t &value) const
    {
        return readBytes(memAddress, &value, 1);
    }

    bool writeByte(uint8_t memAddress, uint8_t value) const
    {
        uint8_t buffer[2] { memAddress, value };
        return device.sendSync(buffer, sizeof(buffer));
    }

    bool readBytes(uint8_t memAddress, uint8_t* begin, uint16_t len) const
    {
        return device.sendThenReceive(&memAddress, 1, begin, len);
    }

private:
    I2CDevice device;
};

}
