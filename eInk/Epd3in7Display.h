#pragma once

#include "graphics/BaseGeometry.h"
#include "MemoryView.h"

namespace embedded
{
class EpdInterface;

// 3,7" 4-grayscale e-paper display
class Epd3in7Display
{
public:
    static constexpr int epdWidth = 280;
    static constexpr int epdHeight = 480;
    static constexpr embedded::Rect<uint16_t> fullScreenRect =
            { { 0, 0 }
              , { epdWidth, epdHeight } };

    enum class RefreshMode
    {
        FullBW, PartBW, Full4Gray
    };

    explicit Epd3in7Display(embedded::EpdInterface &hal)
            : hal(hal) {}

    int init() const;
    void wakeUp() const;
    void sleep() const;
    void reset() const;
    void clear(RefreshMode mode) const;
    void displayFrame(embedded::ConstBytesView image, RefreshMode mode = RefreshMode::FullBW) const;
    void displayWindow(embedded::ConstBytesView image,
                       embedded::Rect<uint16_t> rect,
                       RefreshMode mode = RefreshMode::PartBW) const;


private:
    void waitUntilIdle() const;
    void loadLut(RefreshMode mode) const;
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
    void applyAndWait() const;

    embedded::EpdInterface &hal;
};

}