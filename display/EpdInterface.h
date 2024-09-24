#pragma once

#include "SpiDisplayInterface.h"

namespace embedded
{

class EpdInterface : public SpiDisplayInterface
{
public:
    EpdInterface(SpiDevice &spiDevice, GpioPinDefinition &resetPin, GpioPinDefinition &dcPin,
                 GpioPinDefinition &csPin, GpioPinDefinition &busyPin)
            : SpiDisplayInterface(spiDevice, resetPin, dcPin, csPin), busyPin(busyPin) {}

    void initPins() const override;

    bool getBusyState() const
    {
        return busyPin.check();
    }

private:
    embedded::GpioDigitalPin busyPin;
};

}
