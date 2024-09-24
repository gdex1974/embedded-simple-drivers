#pragma once

#include "MemoryView.h"

#include <cstdint>

namespace embedded
{
class IBmeDevice
{
public:
    virtual bool readByte(uint8_t memAddress, uint8_t& value) const = 0;
    virtual bool writeByte(uint8_t memAddress, uint8_t value) const = 0;
    virtual bool readBytes(uint8_t memAddress, BytesView bytes) const = 0;
};
}
