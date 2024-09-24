#include "SpiHelper.h"

#include "SpiDevice.h"
#include "GpioDigitalPin.h"

#include <cstring>
#include <algorithm>

namespace embedded
{

bool SpiHelper::readBytes(uint8_t memAddress, BytesView bytes) const
{
    std::array<uint8_t, 65> buf;
    size_t startPos = 0;
    while (startPos < bytes.size_bytes())
    {
        std::memset(buf.begin(), 0, buf.size());
        buf[0] = uint8_t(memAddress | 0x80u);
        uint8_t size = std::min(buf.size() - 1, bytes.size_bytes() - startPos);
        ChipSelector sc(csPin);
        if (device.sendAndReceive(buf.begin(), buf.begin(), size + 1))
        {
            for (size_t srcPos = 1, dstPos = 0; dstPos < size;)
            {
                bytes[startPos + dstPos++] = buf[srcPos++];
            }
        }
        else
        {
            return false;
        }
        memAddress += size;
        startPos += size;
    }
    return true;
}

bool SpiHelper::writeByte(uint8_t memAddress, uint8_t value) const
{
    uint8_t buffer[2] { uint8_t(memAddress & 0x7fu), value };
    ChipSelector sc(csPin);
    return device.sendSync(buffer, sizeof(buffer));
}

} // namespace embedded
