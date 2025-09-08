#pragma once
#include <Arduino.h>

#define IBUS_BAUD 115200
#define IBUS_MAX_CHANNELS 14

class IBusDecoder {
public:
    explicit IBusDecoder(HardwareSerial &uart, uint8_t num_ch = 6);

    void begin(uint32_t baud = IBUS_BAUD);

    // Returns true if a valid frame was decoded into out_ch
    bool poll(uint16_t *out_ch, uint32_t &bad_checksum, uint32_t &lost_sync);

    // Reset state machine
    void reset();

private:
    HardwareSerial &_uart;
    uint8_t  _num_ch;

    uint8_t  _state = 0;
    uint8_t  _buf[28];
    uint8_t  _idx = 0;
    uint32_t _sum = 0;
    uint8_t  _chkL = 0;
};
