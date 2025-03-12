#include "Epd4in26Display.h"

#include <Debug.h>

#include "EpdInterface.h"

namespace
{

const uint8_t lutFull4Gray[] =
{
    0x80,	0x48,	0x4A,	0x22,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
    0x0A,	0x48,	0x68,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
    0x88,	0x48,	0x60,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
    0xA8,	0x48,	0x45,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
    0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,	0x00,
    0x07,	0x1E,	0x1C,	0x02,	0x00,   0x05,	0x01,	0x05,	0x01,	0x02,
    0x08,	0x01,	0x01,	0x04,	0x04,   0x00,	0x02,	0x00,	0x02,	0x01,
    0x00,	0x00,	0x00,	0x00,	0x00,   0x00,	0x00,	0x00,	0x00,	0x00,
    0x00,	0x00,	0x00,	0x00,	0x00,   0x00,	0x00,	0x00,	0x00,	0x00,
    0x00,	0x00,	0x00,	0x00,	0x00,   0x00,	0x00,	0x00,	0x00,	0x01,
    0x22,	0x22,	0x22,	0x22,	0x22,   0x17,	0x41,	0xA8,	0x32,	0x30,
};

 const uint8_t gateInitData[] { (embedded::Epd4in26Display::displayHeight - 1)%256,
                                (embedded::Epd4in26Display::displayHeight - 1)/256,
                                0x02 }; // Scan order: all evens first, then all odds
// const uint8_t sourceVoltageData[] { 0x41, 0xA8, 0x32 };
const uint8_t softStartData[] { 0xAE, 0xC7, 0xC3, 0xC0, 0x80 }; // Strength level 2
// const uint8_t displayOptionDataBW[] { 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x4F, 0xFF, 0xFF, 0xFF, 0xFF };

}

namespace embedded
{
int Epd4in26Display::init() const
{
    reset();

    sendCommand(0x18, 0x80); // use the internal temperature sensor

    // setting gate number
    sendCommand(0x0C, softStartData);
    sendCommand(0x01, gateInitData);

    // set border
    sendCommand(0x3C, 0x01); // white

    // set data entry sequence
    sendCommand(0x11, 0x01); // increment X, decrement Y
    waitUntilIdle();

    return 0;
}

int Epd4in26Display::init4Gray() const
{
    init();
    loadLut();
    return 0;
}

void Epd4in26Display::sendCommand(const uint8_t command) const
{
    hal.setDcPin(false);
    hal.spiTransfer(command);
}

void Epd4in26Display::sendData(const unsigned char data) const
{
    hal.setDcPin(true);
    hal.spiTransfer(data);
}

void Epd4in26Display::waitUntilIdle() const
{
    if (hal.getBusyState())
    {
        DEBUG_LOG("Waiting for idle")
        const auto start = getMicrosecondTicks();
        while (hal.getBusyState())
        {
            embedded::delay(1);
        }
        const auto end = getMicrosecondTicks();
        DEBUG_LOG("Idle after " << (end - start) << " us")
    }
    else
    {
        DEBUG_LOG("Idle")
    }
}

void Epd4in26Display::reset() const
{
    hal.reset();
    waitUntilIdle();
    sendCommand(0x12);
    waitUntilIdle();
}

void Epd4in26Display::displayFrame(embedded::ConstBytesView image, RefreshMode mode) const
{
    if (mode != RefreshMode::Full4Gray)
    {
        constexpr uint16_t counter = displayWidth * displayHeight / 8;
        if (image.size() < counter)
        {
            DEBUG_LOG("Invalid image size: " << image.size())
            return;
        }
        prepareToSendScreenData(fullScreenRect);
        sendCommand(0x24);
        sendData({ image.begin(), counter });
    }
    else
    {
        send4GrayOnePlane(image, fullScreenRect, true);
        send4GrayOnePlane(image, fullScreenRect, false);
    }
    applyAndWait(mode);
}

/******************************************************************************
function :  Partial Display
******************************************************************************/
void
Epd4in26Display::displayWindow(embedded::ConstBytesView image, embedded::Rect<uint16_t> rect, RefreshMode mode) const
{
    const auto align = mode != RefreshMode::Full4Gray ? 8 : 4;
    if (auto restX = rect.topLeft.x % align; restX)
    {
        rect.topLeft.x -= restX;
        rect.size.width += restX;
    }
    if (auto restWidth = rect.size.width % align; restWidth)
    {
        rect.size.width += align - restWidth;
    }

    sendRectDataBW(image, rect);
    sendCommand(0x20);
}

void Epd4in26Display::sendRectDataBW(embedded::ConstBytesView image, const embedded::Rect<uint16_t> &rect) const
{
    prepareToSendScreenData(rect);
    sendCommand(0x24);
    const uint16_t bytesWidth = rect.size.width / 8;
    constexpr uint16_t fullBytesWidth = displayWidth / 8;
    auto startBytePtr = image.begin() + rect.topLeft.x / 8 + rect.topLeft.y * fullBytesWidth;
    for (auto colIdx = 0; colIdx < rect.size.height; ++colIdx, startBytePtr += fullBytesWidth)
    {
        sendData({ startBytePtr, bytesWidth });
    }
}

void Epd4in26Display::send4GrayOnePlane(const embedded::ConstBytesView &image,
                                       const embedded::Rect<uint16_t> &rect,
                                       const bool first) const
{
    uint8_t command;
    uint8_t mask;
    if (first)
    {
        command = 0x24;
        mask = 0b01010101;
    }
    else
    {
        command = 0x26;
        mask = 0b10101010;
    }
    prepareToSendScreenData(rect);
    sendCommand(command);
    constexpr uint16_t fullBytesWidth = displayWidth / 4;
    const uint16_t bytesWidth = rect.size.width / 4;
    auto startBytePtr = image.begin() + rect.topLeft.x / 4 + rect.topLeft.y * fullBytesWidth;
    for (auto colIdx = 0; colIdx < rect.size.height; ++colIdx, startBytePtr += fullBytesWidth)
    {
        for (auto pos = 0; pos < bytesWidth; pos += 2)
        {
            uint8_t byte0 = (*(startBytePtr + pos)) & mask;
            uint8_t byte1 = *(startBytePtr + pos + 1) & mask;
            if (first)
            {
                byte0 <<= 1;
                byte1 <<= 1;
            }
            auto target0 = (byte0 & 0x80) | ((byte0 << 1) & 0x40) | ((byte0 << 2) & 0x20) | ((byte0 << 3) & 0x10);
            auto target1 = (byte1 & 0x80) | ((byte1 << 1) & 0x40) | ((byte1 << 2) & 0x20) | ((byte1 << 3) & 0x10);
            sendData((target0 & 0xF0) | (target1 >> 4 & 0x0F));
        }
    }
}

void Epd4in26Display::prepareToSendScreenData(embedded::Rect<uint16_t> rectToBeSent) const
{
    sendAxisLimits(rectToBeSent);
    sendStartPoint(rectToBeSent.topLeft);
}

void Epd4in26Display::sendAxisLimits(embedded::Rect<uint16_t> rect) const
{
    static_assert(mapPoint({0,0}) == Point<uint16_t>{0, displayHeight - 1});
    static_assert(mapPoint({displayWidth - 1, displayHeight - 1}) == Point<uint16_t>{displayWidth - 1, 0});
    const auto topLeft = mapPoint(rect.topLeft);
    const auto bottomRight = mapPoint(rect.bottomRight());
#pragma pack(push, 2)
    union
    {
        struct
        {
            uint16_t start;
            uint16_t end;
        } coords;
        uint8_t data[4];
    } limitsData { .coords = { topLeft.x, bottomRight.x } };
#pragma pack(pop)
    sendCommand(0x44, limitsData.data);
    limitsData.coords = { topLeft.y, bottomRight.y };
    sendCommand(0x45, limitsData.data);
}

void Epd4in26Display::loadLut() const
{
    const auto fullTable = ConstBytesView {lutFull4Gray};
    sendCommand(0x32, fullTable.subspan(0, 105));
    sendCommand(0x03, fullTable[105]);
    sendCommand(0x04, fullTable.subspan(106, 3));
    sendCommand(0x2C, fullTable[109]);
}

void Epd4in26Display::clear() const
{
    constexpr uint16_t imageBytesNumber = displayWidth * displayHeight / 8;

    prepareToSendScreenData(fullScreenRect);

    sendCommand(0x24);
    for (uint16_t i = 0; i < imageBytesNumber; ++i)
    {
        sendData(0xff);
    }
    applyAndWait(RefreshMode::FullBW);
}

void Epd4in26Display::sendStartPoint(embedded::Point<uint16_t> point) const
{
    const auto [x, y] = point;
    union
    {
        struct
        {
            uint16_t coord;
        };
        uint8_t data[2];
    } coordData { .coord = x };
    sendCommand(0x4E, coordData.data);
    coordData.coord = y;
    sendCommand(0x4F, coordData.data);
}

void Epd4in26Display::sleep() const
{
    sendCommand(0X10, 0x00);   //deep sleep
}

void Epd4in26Display::deepSleep() const
{
    sendCommand(0x10, 0x03);
    delay(100);
}


void Epd4in26Display::sendData(embedded::ConstBytesView data) const
{
    hal.setDcPin(true);
    hal.spiTransfer(data);
}

void Epd4in26Display::applyAndWait(RefreshMode mode) const
{
    switch (mode)
    {
        case RefreshMode::Full4Gray:
            sendCommand(0x22, 0xC7);
        break;
        case RefreshMode::FullBW:
            sendCommand(0x22, 0xF7);
        break;
        case RefreshMode::PartBW:
            sendCommand(0x22, 0xFF);
        default:
            break;
    }
    sendCommand(0x20);
    waitUntilIdle();
}

void Epd4in26Display::wakeUp() const
{
    reset();
}

} // namespace embedded
