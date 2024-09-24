#include "ST7302SpiDisplay.h"
#include "SpiDisplayInterface.h"

void embedded::ST7302SpiDisplay::sendCommand(uint8_t command) const
{
    hal.setDcPin(false);
    hal.spiTransfer(command);
}

void embedded::ST7302SpiDisplay::sendData(embedded::ConstBytesView data) const
{
    hal.setDcPin(true);
    hal.spiTransfer(data);
}

void embedded::ST7302SpiDisplay::sendData(unsigned char data) const
{
    hal.setDcPin(true);
    hal.spiTransfer(data);
}

int embedded::ST7302SpiDisplay::init() const
{
    setHighPowerMode(true); // sendCommand(0x38);
    setNvmLoad(true); // sendCommand(0xEB, 0x02); //Enable OTP
    setNvmLoadControl(NvmLoadControlFlags::EnableAll); // sendCommand(0xD7, 0x68); //OTP Load Control
    setBoosterMode(true); // sendCommand(0xD1, 0x01);//Auto Power Control
    setGateVoltageControl(GateHighLevel::VH_12_0, GateLowLevel::VL_5_0); // sendCommand(0xC0, 0x80);//Gate Voltage Setting VGH=12V ; VGL=-5V
    setSourceHighVoltageControl(450, 450, 450, 450, 350, 150); // sendCommand(0xC1, {0x28u, 0x28u, 0x28u, 0x28u, 0x14u, 0x00u});//VSH Setting
    setSourceLowVoltageControl(0, 0, 0, 0);//sendCommand(0xC2, {0x00, 0x00, 0x00, 0x00});//VSL Setting VSL=0
    setVcomH(400);//sendCommand(0xCB, 0x14);//VCOMH Setting
    setGateUpdateControl(GateEqEnableFlag | FirstOutputGateLeftFlag);//sendCommand(0xB4, {0xE5, 0x77, 0xF1, 0xFF, 0xFF, 0x4F, 0xF1, 0xFF, 0xFF, 0x4F});//Gate EQ Setting HPM EQ LPM EQ
    wakeUp();//sendCommand(0x11);//Sleep out
    delay(100);
    enableOSC(); //sendCommand(0xC7, {0xA6, 0xE9});//OSC Setting
    setDuty(256);//sendCommand(0xB0, 0x64);
    setMemoryDataAccessControl(PageDirectionMode, true);//sendCommand(0x36, 0x20); //Memory Data Access Control
    //sendCommand(0x3A, 0x11); //Data Format Select 3 write for 24 bit
    setSourceSettings(false); //sendCommand(0xB9,0x23); //Source Setting
    setPanelSetting(GateScanInterval::Frame, PanelLayoutInterface::OneLine);//sendCommand(0xB8, 0x09); //Panel Setting Frame inversion
    setColumnAddress(0x05, 0x36);//sendCommand(0x2A, {0x05, 0x36}); ////Column Address Setting S61~S182
    setRowAddress(0x00, 0xC7);//sendCommand(0x2B, {0x00, 0xC7}); ////Row Address Setting G1~G250
//    sendCommand(0xD0, 0x1F);
    displayOn();//sendCommand(0x29); //Display on
    clear();
    destressOff();//sendCommand(0x72, 0x00); //Destress OFF
    lowPowerMode();//sendCommand(0x39);//LPM
    return 0;
}

namespace
{
template <int upperBond = 500, int lowerBond = 300>
inline uint8_t normalizeVoltage(int voltage)
{
    return (std::min(upperBond, std::max(voltage, lowerBond)) - lowerBond) / 5;
}
}

void embedded::ST7302SpiDisplay::setSourceHighVoltageControl(int highPowerReflective,
                                                             int highPowerTransmissive,
                                                             int lowPowerReflective,
                                                             int lowPowerTransmissive,
                                                             int gamma1Voltage,
                                                             int gamma2Voltage) const
{
    sendCommand(0xC1, {normalizeVoltage<>(highPowerReflective), normalizeVoltage<>(highPowerTransmissive),
                       normalizeVoltage<>(lowPowerReflective), normalizeVoltage<>(lowPowerTransmissive),
                       normalizeVoltage<450, 250>(gamma1Voltage), normalizeVoltage<350, 150>(gamma2Voltage)});
}

void embedded::ST7302SpiDisplay::setSourceLowVoltageControl(int highPowerReflective,
                                                            int highPowerTransmissive,
                                                            int lowPowerReflective,
                                                            int lowPowerTransmissive) const
{
    sendCommand(0xC2, {normalizeVoltage<250, 0>(highPowerReflective), normalizeVoltage<250, 0>(highPowerTransmissive),
                       normalizeVoltage<250, 0>(lowPowerReflective), normalizeVoltage<250, 0>(lowPowerTransmissive)});
}

void embedded::ST7302SpiDisplay::setGateUpdateControl(uint8_t flags) const
{
    sendCommand(0xB4, {static_cast<uint8_t>(0x25 | flags), 0x77, 0xF1, 0xFF, 0xFF, 0x4F, 0xF1, 0xFF, 0xFF, 0x4F});//Gate EQ Setting HPM EQ LPM EQ
}

void embedded::ST7302SpiDisplay::wakeUp() const
{
    sendCommand(0x11);
    delay(5);
}

void embedded::ST7302SpiDisplay::sleep() const
{
    sendCommand(0x10);
    delay(5);
}

void embedded::ST7302SpiDisplay::reset() const
{
    hal.reset();
}

void embedded::ST7302SpiDisplay::clear() const
{
    setClearRam(true); //sendCommand(0xB9, 0xE3); //enable CLR RAM
    delay(100);
    setClearRam(false);//sendCommand(0xB9, 0x23); //enable CLR RAM
}

void embedded::ST7302SpiDisplay::displayFrame(embedded::ConstBytesView image) const
{
    setColumnAddress(0x19, 0x19 + effectiveHeight / 12);
    setRowAddress(0x00, effectiveWidth / 2 - 1);
    sendCommand(0x2C);
    constexpr int rowBytes = effectiveWidth / 8;
    for (int i = 0; i != effectiveHeight / 12; ++i)
    {
        send12Row(image.subspan(rowBytes * i * 12, rowBytes * 12));
    }
}

void embedded::ST7302SpiDisplay::displayWindow(embedded::ConstBytesView image, embedded::Rect<uint16_t> rect) const
{

}

void embedded::ST7302SpiDisplay::send12Row(ConstBytesView view) const
{
    constexpr int widthByteSize = effectiveWidth / 8;
    for (int i = 0; i != effectiveWidth / 8; ++i) {
        send12RowByte(view[widthByteSize * 0 + i],
                      view[widthByteSize * 1 + i],
                      view[widthByteSize * 2 + i],
                      view[widthByteSize * 3 + i],
                      view[widthByteSize * 4 + i],
                      view[widthByteSize * 5 + i],
                      view[widthByteSize * 6 + i],
                      view[widthByteSize * 7 + i],
                      view[widthByteSize * 8 + i],
                      view[widthByteSize * 9 + i],
                      view[widthByteSize * 10 + i],
                      view[widthByteSize * 11 + i]);
    }
}

void embedded::ST7302SpiDisplay::send12RowByte(const uint8_t byte0, const uint8_t byte1, const uint8_t byte2,
                                               const uint8_t byte3, const uint8_t byte4, const uint8_t byte5,
                                               const uint8_t byte6, const uint8_t byte7, const uint8_t byte8,
                                               const uint8_t byte9, const uint8_t byte10, const uint8_t byte11) const
{
    _send_12_row_bit(0, byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9, byte10, byte11);
    _send_12_row_bit(2, byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9, byte10, byte11);
    _send_12_row_bit(4, byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9, byte10, byte11);
    _send_12_row_bit(6, byte0, byte1, byte2, byte3, byte4, byte5, byte6, byte7, byte8, byte9, byte10, byte11);
}

void embedded::ST7302SpiDisplay::_send_12_row_bit(int line,
                                                  const uint8_t byte0, const uint8_t byte1, const uint8_t byte2,
                                                  const uint8_t byte3, const uint8_t byte4, const uint8_t byte5,
                                                  const uint8_t byte6, const uint8_t byte7, const uint8_t byte8,
                                                  const uint8_t byte9,const uint8_t byte10, const uint8_t byte11) const
{
    uint8_t data = (byte0 << line >> 0 & 0xC0)
                   | (byte1 << line >> 2 & 0x30)
                   | (byte2 << line >> 4 & 0x0C)
                   | (byte3 << line >> 6 & 0x03);
    sendData(data);
    data = (byte4 << line >> 0 & 0xC0)
           | (byte5 << line >> 2 & 0x30)
           | (byte6 << line >> 4 & 0x0C)
           | (byte7 << line >> 6 & 0x03);
    sendData(data);
    data = (byte8 << line >> 0 & 0xC0)
           | (byte9 << line >> 2 & 0x30)
           | (byte10 << line >> 4 & 0x0C)
           | (byte11 << line >> 6 & 0x03);
    sendData(data);
}
