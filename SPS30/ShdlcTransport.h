#pragma once

#include <cstdint>
#include "Sps30Error.h"
#include "MemoryView.h"

namespace embedded
{
class PacketUart;

class ShdlcTransport
{
public:
    explicit ShdlcTransport(embedded::PacketUart &device) : uart(device) {}

    Sps30Error send(uint8_t addr, uint8_t cmd, ConstBytesView bytes) const;

    Sps30Error sendAndReceive(uint8_t addr,
                              uint8_t cmd,
                              embedded::ConstBytesView txData,
                              const embedded::BytesView &rxData);

    Sps30Error sendAndReceive(uint8_t addr, uint8_t cmd, embedded::ConstBytesView txData, embedded::BytesView &bytes);

    Sps30Error activateTransport();

private:
    embedded::PacketUart &uart;
};

}
