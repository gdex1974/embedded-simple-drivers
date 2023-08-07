#include "ShdlcTransport.h"
#include "PacketUart.h"
#include "Debug.h"
#include <numeric>

namespace
{
constexpr int calculateRequiredRxBuffer(const uint8_t payloadSize) { return (2 + (5 + payloadSize) * 2); }

uint8_t calculateCRC(embedded::ConstBytesView bytes)
{
    return ~(uint8_t)std::accumulate(bytes.begin(), bytes.end(), 0);
}

class StuffedBuffer
{
    static constexpr uint8_t MaxPayloadSize = 255;
public:

    void stuffData(const uint8_t addr, const uint8_t cmd, embedded::ConstBytesView bytes)
    {
        addStartStopCode();
        auto startPos= end;
        add(addr);
        add(cmd);
        add(bytes.size());
        add(bytes);
        const auto crc = calculateCRC({ startPos, static_cast<uint16_t>(end - startPos) });
        add(crc);
        addStartStopCode();
    }

    bool unstuffInplace(const uint16_t newSize)
    {
        if (newSize < 7 || *begin() != StartCode)
        {
            return false;
        }
        auto last = begin() + (newSize - 1);
        if (*last != StartCode)
        {
            return false;
        }
        auto sourceIt = begin();
        auto destIt = sourceIt++;
        for (; sourceIt != last; ++sourceIt, ++destIt)
        {
            if (*sourceIt == 0x7e)
            {
                return false;
            }
            else if (*sourceIt == 0x7d)
            {
                if (++sourceIt == last)
                {
                    return false;
                }
                switch (*sourceIt)
                {
                    case 0x31:
                        *destIt = 0x11;
                        break;
                    case 0x33:
                        *destIt = 0x13;
                        break;
                    case 0x5d:
                        *destIt = 0x7d;
                        break;
                    case 0x5e:
                        *destIt = 0x7e;
                        break;
                    default:
                        return false;
                }
            }
            else
            {
                *destIt = *sourceIt;
            }
        }
        end = destIt;
        return (getSize() > 4) && (begin()[3] + 5 == getSize())
               && (calculateCRC({ begin(), (uint16_t)(begin()[3] + 4) }) == *(end - 1));
    }

    uint8_t* begin() { return buffer.begin(); }

    uint16_t getSize() { return (uint16_t)(end - buffer.begin()); }

    uint16_t capacity() { return buffer.size(); }

private:
    static constexpr uint8_t StartCode = 0x7e;

    void addStartStopCode() { *(end++) = StartCode; }

    void add(uint8_t byte)
    {
        switch (byte)
        {
            case 0x11:
            case 0x13:
            case 0x7d:
            case 0x7e:
                // byte stuffing is done by inserting 0x7d and inverting bit 5
                *(end++) = 0x7d;
                *(end++) = byte ^ (1 << 5);
                break;
            default:
                *(end++) = byte;
        }
    }

    void add(embedded::ConstBytesView bytes)
    {
        for (auto byte: bytes)
        {
            add(byte);
        }
    }

    std::array<uint8_t, calculateRequiredRxBuffer(MaxPayloadSize)> buffer;
    decltype(buffer)::iterator end = buffer.begin();
};

}

namespace embedded
{

Sps30Error ShdlcTransport::send(uint8_t addr, uint8_t cmd, ConstBytesView bytes) const
{
    StuffedBuffer buffer;

    buffer.stuffData(addr, cmd, bytes);
    auto len = buffer.getSize();
    DEBUG_LOG("SHDLC::Send sending " << (int)len << " bytes: " << embedded::BytesView(buffer.begin(), len))
    auto ret = uart.Send(buffer.begin(), len);
    if (ret != len)
    {
        return Sps30Error::TransportError;
    }
    return Sps30Error::Success;
}

Sps30Error ShdlcTransport::sendAndReceive(const uint8_t addr,
                                          const uint8_t cmd,
                                          const embedded::ConstBytesView txData,
                                          const embedded::BytesView &bytes)
{
    auto resultView = bytes;
    return sendAndReceive(addr, cmd, txData, resultView);
}

Sps30Error
ShdlcTransport::sendAndReceive(uint8_t addr, uint8_t cmd, embedded::ConstBytesView tx_data, embedded::BytesView &bytes)
{
    auto ret = ShdlcTransport::send(addr, cmd, tx_data);
    if (ret != Sps30Error::Success)
    {
        return ret;
    }

    StuffedBuffer buffer;

    auto received = uart.ReceiveUntil(buffer.begin(), buffer.capacity(), 0x7e, 20);
    DEBUG_LOG("Received " << (int)received << " bytes: " << embedded::BytesView(buffer.begin(), received))

    if (!buffer.unstuffInplace(received))
    {
        DEBUG_LOG("Unstaffing error")
        return Sps30Error::DataError;
    }

    if (bytes.size() < buffer.begin()[3])
    {
        DEBUG_LOG("Insufficient buffer size");
        return Sps30Error::DataError;
    }
    if (buffer.begin()[2] != 0)
    {
        DEBUG_LOG("Unsupported command")
        return Sps30Error::UnsupportedCommand;
    }

    auto pos = buffer.begin() + 4;
    auto data = bytes.begin();
    int counter = 0;
    for (; counter < buffer.begin()[3]; ++counter)
    {
        *(data++) = *(pos++);
    }
    bytes = { bytes.begin(), buffer.begin()[3] };
    DEBUG_LOG("Successfully received payload of " << counter << " bytes");
    return Sps30Error::Success;
}

Sps30Error ShdlcTransport::activateTransport()
{
    const uint8_t data = 0xFF;
    return uart.Send(&data, 1) == 1 ? Sps30Error::Success : Sps30Error::TransportError;
}

}
