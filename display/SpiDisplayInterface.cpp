#include "SpiDisplayInterface.h"

void embedded::SpiDisplayInterface::initPins() const
{
    rstPin.init(GpioDigitalPin::Direction::Output);
    dcPin.init(GpioDigitalPin::Direction::Output);
    csPin.init(GpioDigitalPin::Direction::Output, embedded::GpioDigitalPin::PullMode::Up);
    csPin.set();
}
