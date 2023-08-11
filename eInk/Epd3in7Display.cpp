#include "Epd3in7Display.h"
#include "EpdInterface.h"

namespace
{

const uint8_t lutFull4Gray[] =
{
    0x2A, 0x06, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//1
    0x28, 0x06, 0x14, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//2
    0x20, 0x06, 0x10, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//3
    0x14, 0x06, 0x28, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//4
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//5
    0x00, 0x02, 0x02, 0x0A, 0x00, 0x00, 0x00, 0x08, 0x08, 0x02,//6
    0x00, 0x02, 0x02, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//9
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//10
    0x22, 0x22, 0x22, 0x22, 0x22
};

const uint8_t lutFullBW[] =
{
    0x2A, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//1
    0x05, 0x2A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//2
    0x2A, 0x15, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//3
    0x05, 0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//4
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//5
    0x00, 0x02, 0x03, 0x0A, 0x00, 0x02, 0x06, 0x0A, 0x05, 0x00,//6
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//9
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,//10
    0x22, 0x22, 0x22, 0x22, 0x22
};

const uint8_t lutPartBW[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //1
    0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //2
    0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //3
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //4
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //5
    0x00, 0x00, 0x03, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //6
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //8
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //9
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //10
    0x22, 0x22, 0x22, 0x22, 0x22
};

const uint8_t gateInitData[] { 0xDF, 0x01, 0x00 };
const uint8_t sourceVoltageData[] { 0x41, 0xA8, 0x32 };
const uint8_t boosterData[] { 0xAE, 0xC7, 0xC3, 0xC0, 0xC0 };
const uint8_t displayOptionDataBW[] { 0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0x4F, 0xFF, 0xFF, 0xFF, 0xFF };

}

namespace embedded
{
int Epd3in7Display::init() const
{
    reset();
    sendCommand(0x12);
    embedded::delay(10);

    sendCommand(0x46, 0xF7);
    waitUntilIdle();
    sendCommand(0x47, 0xF7);
    waitUntilIdle();

    // setting gate number
    sendCommand(0x01);
    sendData(gateInitData);

    sendCommand(0x03, 0x00);

    // set source voltage
    sendCommand(0x04);
    sendData(sourceVoltageData);

    // set data entry sequence
    sendCommand(0x11, 0x03);

    // set border
    sendCommand(0x3C, 0x01);

    // set booster strength
    sendCommand(0x0C);
    sendData(boosterData);

    // set internal sensor on
    sendCommand(0x18, 0x80);

    // set vcom value
    sendCommand(0x2C, 0x44);

    // set display option, these setting turn on previous function
    sendCommand(0x37);
    sendData(displayOptionDataBW);

    sendAxisLimits({ {   0       , 0 }
                     , { epdWidth, epdHeight } });

    // Display Update Control 2
    sendCommand(0x22, 0xCF);
    return 0;
}

void Epd3in7Display::sendCommand(uint8_t command) const
{
    hal.setDcPin(false);
    hal.spiTransfer(command);
}

void Epd3in7Display::sendData(unsigned char data) const
{
    hal.setDcPin(true);
    hal.spiTransfer(data);
}

void Epd3in7Display::waitUntilIdle() const
{
    if (hal.getBusyState())
    {
        while (hal.getBusyState())
        {
            embedded::delay(1);
        }
    }
}

void Epd3in7Display::reset() const
{
    hal.resetEpd();
}

void Epd3in7Display::displayFrame(embedded::ConstBytesView image, RefreshMode mode) const
{
    const uint16_t counter = epdWidth * epdHeight / (mode != RefreshMode::Full4Gray ? 8 : 4);
    if (image.size() < counter)
    {
        return;
    }

    if (mode != RefreshMode::Full4Gray)
    {
        prepareToSendScreenData(fullScreenRect);
        sendCommand(0x24);
        sendData({ image.begin(), counter });
    }
    else
    {
        sendRectData4Gray(image, fullScreenRect);
    }

    loadLut(mode);
    applyAndWait();
}

/******************************************************************************
function :  Partial Display
******************************************************************************/
void
Epd3in7Display::displayWindow(embedded::ConstBytesView image, embedded::Rect<uint16_t> rect, RefreshMode mode) const
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
    loadLut(mode);
    applyAndWait();
}

void Epd3in7Display::sendRectDataBW(embedded::ConstBytesView image, const embedded::Rect<uint16_t> &rect) const
{
    prepareToSendScreenData(rect);
    sendCommand(0x24);
    const uint16_t bytesWidth = rect.size.width / 8;
    constexpr uint16_t fullBytesWidth = epdWidth / 8;
    auto startBytePtr = image.begin() + rect.topLeft.x / 8 + rect.topLeft.y * fullBytesWidth;
    for (auto colIdx = 0; colIdx < rect.size.height; ++colIdx, startBytePtr += fullBytesWidth)
    {
        sendData({ startBytePtr, bytesWidth });
    }
}

void Epd3in7Display::sendRectData4Gray(embedded::ConstBytesView image, const embedded::Rect<uint16_t> &rect) const
{
    send4GrayOnePlane(image, rect, true);
    send4GrayOnePlane(image, rect, false);
}

void Epd3in7Display::send4GrayOnePlane(const embedded::ConstBytesView &image,
                                       const embedded::Rect<uint16_t> &rect,
                                       bool first) const
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
    constexpr uint16_t fullBytesWidth = epdWidth / 4;
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

void Epd3in7Display::prepareToSendScreenData(embedded::Rect<uint16_t> rectToBeSent) const
{
    sendAxisLimits(rectToBeSent);
    sendStartPoint(rectToBeSent.topLeft);
}

void Epd3in7Display::sendAxisLimits(embedded::Rect<uint16_t> rect) const
{
#pragma pack(push, 2)
    union LimitsData
    {
        struct
        {
            uint16_t start;
            uint16_t end;
        } coords;
        uint8_t data[4];
    };
#pragma pack(pop)
    const auto &bottomRight = rect.bottomRight();
    LimitsData limitsData { .coords = { rect.topLeft.x, bottomRight.x } };
    sendCommand(0x44, limitsData.data);
    limitsData.coords = { rect.topLeft.y, bottomRight.y };
    sendCommand(0x45, limitsData.data);
}

void Epd3in7Display::loadLut(RefreshMode mode) const
{
    sendCommand(0x32);
    switch (mode)
    {
        case RefreshMode::PartBW:
            sendData(lutPartBW);
            break;
        case RefreshMode::FullBW:
            sendData(lutFullBW);
            break;
        case RefreshMode::Full4Gray:
            sendData(lutFull4Gray);
            break;
    }
}

void Epd3in7Display::clear(RefreshMode mode) const
{
    constexpr uint16_t imageBytesNumber = epdWidth * epdHeight / 8;

    prepareToSendScreenData(fullScreenRect);

    sendCommand(0x24);
    for (uint16_t i = 0; i < imageBytesNumber; ++i)
    {
        sendData(0xff);
    }
    if (mode == RefreshMode::Full4Gray)
    {
        sendCommand(0x26);
        for (uint16_t i = 0; i < imageBytesNumber; ++i)
        {
            sendData(0xff);
        }
    }
    loadLut(mode);
    applyAndWait();
}

void Epd3in7Display::sendStartPoint(embedded::Point<uint16_t> point) const
{
    union CoordData
    {
        struct
        {
            uint16_t coord;
        };
        uint8_t data[2];
    };
    CoordData coordData { .coord = point.x };
    sendCommand(0x4E, coordData.data);
    coordData.coord = point.y;
    sendCommand(0x4F, coordData.data);
}

void Epd3in7Display::sleep() const
{
    sendCommand(0X10, 0x03);   //deep sleep
}

void Epd3in7Display::sendData(embedded::ConstBytesView data) const
{
    hal.setDcPin(true);
    hal.spiTransfer(data);
}

void Epd3in7Display::applyAndWait() const
{
    sendCommand(0x20);
    waitUntilIdle();
}

void Epd3in7Display::wakeUp() const
{
    reset();
}

} // namespace embedded
