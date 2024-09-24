#include "EpdInterface.h"

namespace embedded
{

void EpdInterface::initPins() const
{
    SpiDisplayInterface::initPins();
    busyPin.init(GpioDigitalPin::Direction::Input);
}

}
