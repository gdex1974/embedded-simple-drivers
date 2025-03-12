#pragma once

#include "GpioDigitalPin.h"
#include "SpiDevice.h"
#include "Delays.h"
#include "MemoryView.h"

namespace embedded
{

class SpiDisplayInterface
{
public:
    SpiDisplayInterface(SpiDevice &spiDevice, GpioPinDefinition &resetPin, GpioPinDefinition &dcPin,
    GpioPinDefinition &csPin)
    : rstPin(resetPin)
    , dcPin(dcPin)
    , csPin(csPin)
    , spiDevice(spiDevice) {}

    virtual void initPins() const;

    void spiTransfer(unsigned char data) const
    {
        ChipSelector cs(csPin);
        spiDevice.sendSync(&data, 1);
    }

    void spiTransfer(ConstBytesView buf) const
    {
        ChipSelector cs(csPin);
        spiDevice.sendSync(buf.begin(), buf.size());
    }

    void setDcPin(bool set) const
    {
        dcPin.set(set);
    }

    void reset() const
    {
        delay(20);
        {
            ChipSelector cs(rstPin);
            delayMicroseconds(2000);
        }
        delay(20);
    }

private:
    GpioDigitalPin rstPin;
    GpioDigitalPin dcPin;
    GpioDigitalPin csPin;
    SpiDevice &spiDevice;
};

}
