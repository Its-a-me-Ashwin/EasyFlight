#include "IBus.h"

IBusDecoder::IBusDecoder(HardwareSerial &uart, uint8_t num_ch)
: _uart(uart), _num_ch(num_ch > IBUS_MAX_CHANNELS ? IBUS_MAX_CHANNELS : num_ch) {
    reset();
}

void IBusDecoder::begin(uint32_t baud) {
    _uart.begin(baud);
}

bool IBusDecoder::poll(uint16_t *out_ch, uint32_t &bad_checksum, uint32_t &lost_sync) {
    while (_uart.available()) {
        uint8_t b = _uart.read();
        switch (_state) {
            case 0: // expect 0x20 (length)
                if (b == 0x20) { _sum = b; _idx = 0; _state = 1; }
                else { lost_sync++; reset(); }
                break;

            case 1: // expect 0x40 (type)
                if (b == 0x40) { _sum += b; _state = 2; }
                else { lost_sync++; reset(); }
                break;

            case 2: // payload
                _buf[_idx++] = b; _sum += b;
                if (_idx >= 28) { _state = 3; }
                break;

            case 3: // checksum LSB
                _chkL = b; _state = 4;
                break;

            case 4: { // checksum MSB
                uint16_t chk_calc = (uint16_t)(0xFFFF - (_sum & 0xFFFF));
                uint16_t chk_recv = (uint16_t)_chkL | ((uint16_t)b << 8);
                if (chk_calc == chk_recv) {
                    for (int i = 0; i < _num_ch; ++i) {
                        int o = i * 2;
                        out_ch[i] = (uint16_t)_buf[o] | ((uint16_t)_buf[o + 1] << 8);
                    }
                    reset();
                    return true;
                } else {
                    bad_checksum++;
                    reset();
                }
            } break;
        }
    }
    return false;
}

void IBusDecoder::reset() {
    _state = 0;
    _idx = 0;
    _sum = 0;
    _chkL = 0;
}
