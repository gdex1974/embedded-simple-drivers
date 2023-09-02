#include "EpdInterface.h"

namespace embedded
{

void EpdInterface::initPins() const
{
    rstPin.init(GpioDigitalPin::Direction::Output);
    dcPin.init(GpioDigitalPin::Direction::Output);
    busyPin.init(GpioDigitalPin::Direction::Input);
    csPin.init(GpioDigitalPin::Direction::Output);
}

}
