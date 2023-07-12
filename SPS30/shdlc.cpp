#include "shdlc.h"
#include "PacketUart.h"
#include "Debug.h"
#include <numeric>

namespace
{

constexpr auto MaxPayloadSize = 255;

constexpr int calculateRequiredRxBuffer(const uint8_t payloadSize) { return (2 + (5 + payloadSize) * 2); }

uint8_t calculateCRC(const uint8_t sum, const uint8_t* bytes, const uint16_t size)
{
    return ~(uint8_t)std::accumulate(bytes, bytes + size, sum + size);
}

uint8_t calculateCRC(embedded::ConstBytesView bytes)
{
    return ~(uint8_t)std::accumulate(bytes.begin(), bytes.end(), 0);
}

class StuffedBuffer
{
public:

    void stuffData(const uint8_t addr, const uint8_t cmd, const uint8_t size, const uint8_t* bytes)
    {
        const auto crc = calculateCRC(addr + cmd, bytes, size);

        addStartStopCode();
        add(addr);
        add(cmd);
        add(size);
        add(bytes, size);
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

    void add(const uint8_t* bytes, const uint8_t size)
    {
        auto last = bytes + size;
        for (auto p = bytes; p != last; ++p)
        {
            add(*p);
        }
    }

    std::array<uint8_t, calculateRequiredRxBuffer(MaxPayloadSize)> buffer;
    decltype(buffer)::iterator end = buffer.begin();
};

}

namespace embedded
{

Sps30Error SHDLC::Send(const uint8_t addr, const uint8_t cmd, const uint8_t size, const uint8_t* bytes)
{
    StuffedBuffer buffer;

    buffer.stuffData(addr, cmd, size, bytes);
    auto len = buffer.getSize();
    DEBUG_LOG("SHDLC::Send sending " << (int)len << " bytes: " << embedded::BytesView(buffer.begin(), len))
    auto ret = uart.Send(buffer.begin(), len);
    if (ret != len)
    {
        return Sps30Error::TransportError;
    }
    return Sps30Error::Success;
}

Sps30Error SHDLC::SendAndReceive(const uint8_t addr,
                                 const uint8_t cmd,
                                 const uint8_t tx_data_len,
                                 const uint8_t* tx_data,
                                 const embedded::BytesView &bytes)
{
    auto resultView = bytes;
    return SendAndReceive(addr, cmd, tx_data_len, tx_data, resultView);
}

Sps30Error SHDLC::SendAndReceive(const uint8_t addr,
                                 const uint8_t cmd,
                                 const uint8_t tx_data_len,
                                 const uint8_t* tx_data,
                                 embedded::BytesView &bytes)
{
    auto ret = SHDLC::Send(addr, cmd, tx_data_len, tx_data);
    if (ret != Sps30Error::Success)
    {
        return ret;
    }

    StuffedBuffer buffer;

    auto received = uart.ReceiveUntil(buffer.begin(), buffer.capacity(), 0x7e, 200);
    DEBUG_LOG("Received " << (int)received << " bytes: " << embedded::BytesView(buffer.begin(), received))

    if (!buffer.unstuffInplace(received) || bytes.size() < buffer.begin()[3])
    {
        DEBUG_LOG("Unstaffing error")
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

Sps30Error SHDLC::ActivateTransport()
{
    const uint8_t data = 0xFF;
    return uart.Send(&data, 1) == 1 ? Sps30Error::Success : Sps30Error::TransportError;
}

}
