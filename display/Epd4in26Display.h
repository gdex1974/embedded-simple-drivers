#pragma once

#include <GpioDigitalPin.h>

#include "graphics/BaseGeometry.h"
#include "MemoryView.h"

namespace embedded
{
class EpdInterface;
class GpioDigitalPin;

// 4,26" 4-grayscale e-paper display
class Epd4in26Display
{
public:
    static constexpr int displayWidth = 800;
    static constexpr int displayHeight = 480;
    static constexpr embedded::Rect<uint16_t> fullScreenRect =
            { { 0, 0 }
              , { displayWidth, displayHeight } };

    enum class RefreshMode
    {
        FullBW, PartBW, Full4Gray
    };

    Epd4in26Display(embedded::EpdInterface &hal, embedded::GpioDigitalPin &powerPin)
            : hal(hal), powerPin(powerPin) {}

    void powerOn()
    {
        powerPin.set();
    }
    int init() const;
    int init4Gray() const;
    void wakeUp() const;
    void sleep() const;
    void deepSleep() const;
    void reset() const;
    void clear() const;
    void displayFrame(embedded::ConstBytesView image, RefreshMode mode = RefreshMode::FullBW) const;
    void displayWindow(embedded::ConstBytesView image,
                       embedded::Rect<uint16_t> rect,
                       RefreshMode mode = RefreshMode::PartBW) const;
    void waitUntilIdle() const;
    void powerOff()
    {
        powerPin.reset();
    }

private:
    void loadLut() const;
    void sendCommand(uint8_t command) const;
    void sendCommand(uint8_t command, uint8_t arg) const
    {
        sendCommand(command);
        sendData(arg);
    }

    void sendCommand(uint8_t command, embedded::ConstBytesView data) const
    {
        sendCommand(command);
        sendData(data);
    }

    void sendData(unsigned char data) const;
    void sendData(embedded::ConstBytesView data) const;
    void sendStartPoint(embedded::Point<uint16_t> point) const;
    void sendAxisLimits(embedded::Rect<uint16_t> rect) const;
    void prepareToSendScreenData(embedded::Rect<uint16_t> rectToBeSent) const;
    void sendRectDataBW(embedded::ConstBytesView image, const embedded::Rect<uint16_t> &rect) const;
    void sendRectData4Gray(embedded::ConstBytesView image, const embedded::Rect<uint16_t> &rect) const;
    void send4GrayOnePlane(const embedded::ConstBytesView &image, const embedded::Rect<uint16_t> &rect, bool first) const;
    void applyAndWait(RefreshMode mode) const;
    static constexpr Point<uint16_t> mapPoint(const Point<uint16_t> &point)
    {
        return Matrix2x2<uint16_t>(1, 0, 0,-1) * point + Size<uint16_t>{0, displayHeight - 1};
    }

    embedded::EpdInterface &hal;
    embedded::GpioDigitalPin &powerPin;
};

}