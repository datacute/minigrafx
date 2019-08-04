/**
The MIT License (MIT)
Copyright (c) 2019 by Daniel Eichhorn, ThingPulse
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

Please note: We are spending a lot of time to write and maintain open source codes
Please support us by buying our products from https://thingpulse.com/shop/

See more at https://thingpulse.com

Many thanks go to various contributors such as Adafruit, Waveshare.
*/

#ifndef EPD2IN13_H
#define EPD2IN13_H

#include <SPI.h>
#include "DisplayDriver.h"

// Display resolution is 250x122 black/white pixels
// (Neither of which is a multiple of 8)
// Driver has RAM for 296x160 black/white/red pixels
#define EPD_WIDTH       122
#define EPD_HEIGHT      250
#define EPD_WIDTH_MAX   160
#define EPD_HEIGHT_MAX  296

// Pin number to mean that there is no connection
#define EPD_NC          255

// EPD2IN13 commands
#define DRIVER_OUTPUT_CONTROL                       0x01
#define GATE_DRIVING_VOLTAGE_CONTROL                0x03
#define SOURCE_DRIVING_VOLTAGE_CONTROL              0x04
#define BOOSTER_SOFT_START_CONTROL                  0x0C
#define GATE_SCAN_START_POSITION                    0x0F
#define DEEP_SLEEP_MODE                             0x10
#define DATA_ENTRY_MODE_SETTING                     0x11
#define SW_RESET                                    0x12
#define TEMPERATURE_SENSOR_CONTROL                  0x1A
#define MASTER_ACTIVATION                           0x20
#define DISPLAY_UPDATE_CONTROL_1                    0x21
#define DISPLAY_UPDATE_CONTROL_2                    0x22
#define WRITE_RAM                                   0x24
#define WRITE_VCOM_REGISTER                         0x2C
#define WRITE_LUT_REGISTER                          0x32
#define SET_DUMMY_LINE_PERIOD                       0x3A
#define SET_GATE_TIME                               0x3B
#define BORDER_WAVEFORM_CONTROL                     0x3C
#define SET_RAM_X_ADDRESS_START_END_POSITION        0x44
#define SET_RAM_Y_ADDRESS_START_END_POSITION        0x45
#define SET_RAM_X_ADDRESS_COUNTER                   0x4E
#define SET_RAM_Y_ADDRESS_COUNTER                   0x4F
#define SET_ANALOG_BLOCK_CONTROL                    0x74
#define SET_DIGITAL_BLOCK_CONTROL                   0x7E
#define TERMINATE_FRAME_READ_WRITE                  0xFF


const unsigned char lut_full_update[] =
{
    0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00, //LUT0: BB:     VS 0 ~7
    0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00, //LUT1: BW:     VS 0 ~7
    0x80, 0x60, 0x40, 0x00, 0x00, 0x00, 0x00, //LUT2: WB:     VS 0 ~7
    0x10, 0x60, 0x20, 0x00, 0x00, 0x00, 0x00, //LUT3: WW:     VS 0 ~7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //LUT4: VCOM:   VS 0 ~7

    0x03, 0x03, 0x00, 0x00, 0x02, // TP0 A~D RP0
    0x09, 0x09, 0x00, 0x00, 0x02, // TP1 A~D RP1
    0x03, 0x03, 0x00, 0x00, 0x02, // TP2 A~D RP2
    0x00, 0x00, 0x00, 0x00, 0x00, // TP3 A~D RP3
    0x00, 0x00, 0x00, 0x00, 0x00, // TP4 A~D RP4
    0x00, 0x00, 0x00, 0x00, 0x00, // TP5 A~D RP5
    0x00, 0x00, 0x00, 0x00, 0x00, // TP6 A~D RP6

    0x15, //VGH
    0x41, //VSH1
    0xA8, //VSH2
    0x32, //VSL
    0x30, //Frame 1
    0x0A, //Frame 2
};

const unsigned char lut_partial_update[] =
{
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //LUT0: BB:     VS 0 ~7
    0x80, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //LUT1: BW:     VS 0 ~7
    0x40, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //LUT2: WB:     VS 0 ~7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //LUT3: WW:     VS 0 ~7
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, //LUT4: VCOM:   VS 0 ~7

    0x0A, 0x00, 0x00, 0x00, 0x00, // TP0 A~D RP0
    0x00, 0x00, 0x00, 0x00, 0x00, // TP1 A~D RP1
    0x00, 0x00, 0x00, 0x00, 0x00, // TP2 A~D RP2
    0x00, 0x00, 0x00, 0x00, 0x00, // TP3 A~D RP3
    0x00, 0x00, 0x00, 0x00, 0x00, // TP4 A~D RP4
    0x00, 0x00, 0x00, 0x00, 0x00, // TP5 A~D RP5
    0x00, 0x00, 0x00, 0x00, 0x00, // TP6 A~D RP6

    0x15, //VGH
    0x41, //VSH1
    0xA8, //VSH2
    0x32, //VSL
    0x30, //Frame 1
    0x0A, //Frame 2
};

class EPD_LOLIN213 : public DisplayDriver {
public:
    unsigned long width;
    unsigned long height;

    EPD_LOLIN213(uint8_t csPin, uint8_t rstPin, uint8_t dcPin, uint8_t busyPin);
    ~EPD_LOLIN213();

    void setRotation(uint8_t r);
    void init();

    void writeBuffer(BufferInfo *bufferInfo);

    int Init(const unsigned char* lut);

    int  IfInit(void);
    void DigitalWrite(int pin, int value);
    int  DigitalRead(int pin);
    void DelayMs(unsigned int delaytime);
    void SpiTransfer(unsigned char data);

    void SendCommand(unsigned char command);
    void SendData(unsigned char data);
    void WaitUntilIdle(void);
    void Reset(void);

    void DisplayFrame(void);
    void Sleep(void);
    void setFastRefresh(boolean isFastRefreshEnabled);

private:
    uint8_t rotation;
    uint8_t reset_pin;
    uint8_t dc_pin;
    uint8_t cs_pin;
    uint8_t busy_pin;
    const unsigned char* lut;

    uint8_t reverse(uint8_t in);
    uint8_t getPixel(uint8_t *buffer, uint16_t x, uint16_t y, uint16_t bufferWidth, uint16_t bufferHeight);
    void SetLut(const unsigned char* lut);
    void SetMemoryArea(int x_start, int y_start, int x_end, int y_end);
    void SetMemoryPointer(int x, int y);
};

#endif /* EPD2IN13_H */

/* END OF FILE */
