#pragma once

#include <cstdint>
#include "Sps30Error.h"
#include "MemoryView.h"

namespace embedded
{
class PacketUart;

class SHDLC
{
public:
    explicit SHDLC(embedded::PacketUart &device) : uart(device) {}

    Sps30Error Send(uint8_t addr, uint8_t cmd, uint8_t size, const uint8_t* bytes);

    Sps30Error SendAndReceive(uint8_t addr, uint8_t cmd, uint8_t tx_data_len, const uint8_t* tx_data,
                              const embedded::BytesView &bytes = {});

    Sps30Error SendAndReceive(uint8_t addr, uint8_t cmd, uint8_t tx_data_len, const uint8_t* tx_data,
                              embedded::BytesView &bytes);

    Sps30Error ActivateTransport();

private:
    embedded::PacketUart &uart;
};

}
