#pragma once

#include "GpioDigitalPin.h"
#include "SpiDevice.h"
#include "Delays.h"
#include "MemoryView.h"

namespace embedded
{

class EpdInterface
{
public:
    EpdInterface(SpiDevice &spiDevice, GpioPinDefinition &resetPin, GpioPinDefinition &dcPin,
                 GpioPinDefinition &csPin, GpioPinDefinition &busyPin)
            : rstPin(resetPin)
              , dcPin(dcPin)
              , csPin(csPin)
              , busyPin(busyPin)
              , spiDevice(spiDevice) {}

    void initPins() const;

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

    bool getBusyState() const
    {
        return busyPin.check();
    }

    void resetEpd() const
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
    embedded::GpioDigitalPin busyPin;
    embedded::SpiDevice &spiDevice;
};

}
