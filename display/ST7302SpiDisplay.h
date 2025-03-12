#pragma once

#include "graphics/BaseGeometry.h"
#include "MemoryView.h"

#include <cmath>

namespace embedded
{
class SpiDisplayInterface;

class ST7302SpiDisplay
{
public:
    static constexpr int displayWidth = 250;
    static constexpr int displayHeight = 122;
    static constexpr int effectiveWidth = displayWidth % 8 == 0 ? displayWidth : (displayWidth/8 + 1) * 8;
    static constexpr int effectiveHeight = displayHeight % 12 == 0 ? displayHeight : (displayHeight/12 + 1) * 12;
    static constexpr embedded::Rect<uint16_t> fullScreenRect =
            { {   0           , 0 }
              , { displayWidth, displayHeight } };
    explicit ST7302SpiDisplay(embedded::SpiDisplayInterface &hal)
            : hal(hal) {}
    int init() const;
    void wakeUp() const;
    void sleep() const;
    void reset() const;
    void softReset() const;
    void clear() const;
    void displayFrame(embedded::ConstBytesView image) const;
    void displayWindow(embedded::ConstBytesView image, embedded::Rect<uint16_t> rect) const;
    void displayOn() const
    {
        sendCommand(0x29);
    }
    void displayOff() const
    {
        sendCommand(0x28);
    }
    void setInversion(bool on = true) const
    {
        sendCommand(on ? 0x21 : 0x20);
    }

private:
    static constexpr uint8_t columnAddressStart = 0x14;
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

    void sendCommand(uint8_t command, std::initializer_list<uint8_t> data) const
    {
        sendCommand(command);
        for (auto d : data)
        {
            sendData(d);
        }
    }

    void sendData(unsigned char data) const;
    void sendData(embedded::ConstBytesView data) const;
    void sendStartPoint(embedded::Point<uint16_t> point) const;
    void sendAxisLimits(embedded::Rect<uint16_t> rect) const;
    void prepareToSendScreenData(embedded::Rect<uint16_t> rectToBeSent) const;
    void setHighPowerMode(bool high) const
    {
        sendCommand(0x38 | (high ? 0 : 1));
    }
    void setNvmLoad(bool enabled) const
    {
        sendCommand(0xEB , enabled ? 2 : 0);
    }
    enum class NvmLoadControlFlags : uint8_t
    {
        EnableAll = 0,
        DisableIDLoad = 1 << 1,
        DisableVoltageControlLoad = 1 << 2,
    };
    void setNvmLoadControl(NvmLoadControlFlags flags) const
    {
        sendCommand(0xD7, 0x68 | static_cast<uint8_t>(flags));
    }
    void setBoosterMode(bool enabled) const
    {
        sendCommand(0xD1, enabled ? 1 : 0);
    }
    enum class GateHighLevel : uint8_t
    {
        VH_8_0 = 0x00,
        VH_8_5 = 0x01,
        VH_9_0 = 0x02,
        VH_9_5 = 0x03,
        VH_10_0 = 0x04,
        VH_10_5 = 0x05,
        VH_11_0 = 0x06,
        VH_11_5 = 0x07,
        VH_12_0 = 0x08,
        VH_12_5 = 0x09,
        VH_13_0 = 0x0A,
        VH_13_5 = 0x0B,
        VH_14_0 = 0x0C,
        VH_14_5 = 0x0D,
        VH_15_0 = 0x0E,
    };
    enum class GateLowLevel : uint8_t
    {
        VL_5_0 = 0x00,
        VL_5_5 = 0x01,
        VL_6_0 = 0x02,
        VL_6_5 = 0x03,
        VL_7_0 = 0x04,
        VL_7_5 = 0x05,
        VL_8_0 = 0x06,
        VL_8_5 = 0x07,
        VL_9_0 = 0x08,
        VL_9_5 = 0x09,
        VL_10_0 = 0x0A,
    };

    void setGateVoltageControl(GateHighLevel vgh, GateLowLevel vgl) const
    {
        sendCommand(0xC0, (static_cast<uint8_t>(vgh) << 4) | static_cast<uint8_t>(vgl));
    }

    //all input parameters are in the hundredths of a volt
    void setSourceHighVoltageControl(int highPowerReflective, int highPowerTransmissive, int lowPowerReflective,
                                     int lowPowerTransmissive, int gamma1Voltage, int gamma2Voltage) const;
    void setSourceLowVoltageControl(int highPowerReflective, int highPowerTransmissive, int lowPowerReflective,
                                    int lowPowerTransmissive) const;
    void setVcomH(int vcomh) const //vcomh in the hundredths of a volt
    {
        sendCommand(0xCB, (std::min(500, std::max(vcomh, 300)) - 300) / 5);
    }
    void setGateDrivingVoltage(int vgh, int vgl) const
    {
        sendCommand(0xB0, (vgh << 4) | vgl);
    }

    static constexpr uint8_t GateEqEnableFlag = 1 << 6;
    static constexpr uint8_t FirstOutputGateLeftFlag = 1 << 7;

    void setGateUpdateControl(uint8_t flags) const;
    void sleepMode() const
    {
        sendCommand(0x10);
    }
    void enableOSC(bool enable = true) const
    {
        sendCommand(0xC7, {static_cast<uint8_t >(0x26 | (enable ? 0x80:0)), 0xE9});
    }
    void setDuty(unsigned duty) const
    {
        sendCommand(0xB0, std::max(std::min(320u, duty) >> 2, 1u));
    }

    enum MemoryDataAccessControlFlags
    {
        PageAddressBottomToTop = 1 << 7,
        ColumnAddressRightToLeft = 1 << 6,
        PageDirectionMode = 1 << 5,
        RefreshBottomToTop = 1 << 4,
        DataOrderRightToLeft = 1 << 3,
        GateScanBottomToTop = 1 << 2,
    };

    void setMemoryDataAccessControl(MemoryDataAccessControlFlags flags, bool mode3Writes) const
    {
        sendCommand(0x36, flags & 0xFC);
        const auto parameter = ((flags & PageAddressBottomToTop) ? 1 << 4 : 0) | ( mode3Writes ? 1 : 0) ;
        sendCommand(0x3A, parameter);
    }

    void setSourceSettings(bool clearRAM) const
    {
        sendCommand(0xB9, clearRAM ? 0xE3 : 0x23);
    }

    enum class GateScanInterval : uint8_t
    {
        TwoLine = 0,
        OneLine = 1,
        Frame = 2,
    };

    enum class PanelLayoutInterface : uint8_t
    {
        TwoLine = 0,
        OneLine = 1,
        None = 2,
    };

    void setPanelSetting(GateScanInterval scanInterval, PanelLayoutInterface layoutInterface) const
    {
        sendCommand(0xB8, (static_cast<uint8_t>(scanInterval) << 2) | static_cast<uint8_t>(layoutInterface));
    }
    void setColumnAddress(uint8_t start, uint8_t end) const
    {
        end = std::min(uint8_t(0x3B), end);
        start = std::min(end, std::max(start, columnAddressStart));
        sendCommand(0x2A, {start, end});
    }
    void setRowAddress(uint8_t start, uint8_t end) const
    {
        end = std::min(uint8_t(0x9F), end);
        start = std::min(end, start);
        sendCommand(0x2B, {start, end});
    }
    void setVcomAndDataIntervalSetting(uint8_t vcom, uint8_t dataInterval) const
    {
        sendCommand(0xD0, (vcom << 4) | dataInterval);
    }
    void setClearRam(bool enable) const
    {
        sendCommand(0xB9, enable ? 0xE3 : 0x23);
    }
    void destressOff() const
    {
        sendCommand(0x72, 0x00);
    }
    void lowPowerMode() const
    {
        sendCommand(0x39);
    }

    embedded::SpiDisplayInterface &hal;
    void send12Row(ConstBytesView view) const;
    void send12RowByte(const uint8_t i,
                       const uint8_t i1,
                       const uint8_t i2,
                       const uint8_t i3,
                       const uint8_t i4,
                       const uint8_t i5,
                       const uint8_t i6,
                       const uint8_t i7,
                       const uint8_t i8,
                       const uint8_t i9,
                       const uint8_t i10,
                       const uint8_t i11) const;
    void _send_12_row_bit(int line,
                          const uint8_t byte0,
                          const uint8_t byte1,
                          const uint8_t byte2,
                          const uint8_t byte3,
                          const uint8_t byte4,
                          const uint8_t byte5,
                          const uint8_t byte6,
                          const uint8_t byte7,
                          const uint8_t byte8,
                          const uint8_t byte9,
                          const uint8_t byte10,
                          const uint8_t byte11) const;
};

}
