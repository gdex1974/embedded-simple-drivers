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
        embedded::ChipSelector cs(csPin);
        spiDevice.sendSync(&data, 1);
    }

    void spiTransfer(ConstBytesView buf) const
    {
        embedded::ChipSelector cs(csPin);
        spiDevice.sendSync(buf.begin(), buf.size());
    }

    void setDcPin(bool set) const
    {
        dcPin.set(set);
    }

    void reset() const
    {
        rstPin.set();
        embedded::delay(20);
        {
            embedded::ChipSelector cs(rstPin);
            embedded::delay(2);
        }
        embedded::delay(20);
    }

private:
    embedded::GpioDigitalPin rstPin;
    embedded::GpioDigitalPin dcPin;
    embedded::GpioDigitalPin csPin;
    embedded::SpiDevice &spiDevice;
};

}
