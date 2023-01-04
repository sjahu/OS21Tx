/*
 * A library for transmitting temperature and humidity data via the Oregon Scientific v2.1 protocol.
 *
 * Requires a 433.92 MHz transmitter connected to a digital pin and a 32 768 Hz crystal oscillator
 * connected to T0 (PB2 on ATtiny85).
 *
 * Assumes that an interrupt waking the CPU from sleep will occur 2 048 times per second. It should
 * be straightforward to change how this interrupt is generated (e.g. to use an oscillator with a
 * different frequency) by modifying the configureTimer() and restoreTimer() functions below.

 * More info here: https://shumphries.ca/blog/2023/01/03/oregon-scientific-attiny85
 *
 * LICENCE
 *
 * Copyright © 2023 Stephen Humphries
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef OS21TX_H
#define OS21TX_H

// Example transmission data
// - Bytes are transmitted in order, small nibble first
// - Nibbles are transmitted LSB-first
// - Nibble descritions in this example are large nibble first, to align with the byte-wise representation
// - Sensor ID 1d20, Channel 1, Rolling ID bb, Battery low, Temperature 22.7°C, Humidity 30%
// uint8_t data[] = {
//   0xff, // Preamble (16 ones (transmitted as 32 bits, alternating 01))
//   0xff, // Preamble
//   0x1a, // Sensor ID (1d20) / Sync (0xa)
//   0x2d, // Sensor ID
//   0x20, // Channel (1=0x1, 2=0x2, 3=0x4) / Sensor ID
//   0xbb, // Rolling ID (randomly generated on startup)
//   0x7c, // Temperature, 10^-1 / Battery low (low is 0x4, not low is 0x0, but both are often OR'd with a 0x8 bit of unknown significance)
//   0x22, // Temperature, 10^1 / Temperature, 10^0
//   0x00, // Humidity, 10^0 / Temperature sign (largest 2 bits, 0x0 for +ve, 0x8 for -ve) | Temperature 10^2 (smallest 2 bits)
//   0x83, // Unknown / Humidity, 10^1
//   0x4a, // Checksum (simple sum)
//   0x55, // Postamble (CRC checksum)
// };

#define SUM_MASK 0xfffe0 // Only some nibbles are included in the checksum and CRC calculations
#define CRC_MASK 0xff3e0
#define CRC_IV 0x42 // ¯\_(ツ)_/¯ (see the blog post for details)
#define CRC_POLY 0x7 // CRC-8-CCITT

#define DATA_LEN 12

#include <avr/sleep.h>

class OS21Tx {
  public:
  const uint8_t pin;

  OS21Tx(uint8_t pin): pin(pin) {}

  void begin(uint8_t channel, uint8_t rollingId) {
    pinMode(pin, OUTPUT);

    setRollingId(rollingId);
    setChannel(channel);
  }

  void transmit(float temperature, float humidity, bool lowBattery = false) {
    setTemperature(temperature);
    setHumidity(humidity);
    setLowBattery(lowBattery);
    setChecksum();
    setCRC();

    sendData(); // Send the message twice
    delay(55); // Pause for a short time between transmissions
    sendData();
  }

  private:

  uint8_t old_TCCR0A;
  uint8_t old_TCCR0B;
  uint8_t old_OCR0A;
  uint8_t old_TIMSK;

  uint8_t data[DATA_LEN] = { // Data frame, initialized with the parts that never change
    0xff,            // Preamble
    0xff,
    0x1a,            // Sync nibble and sensor ID
    0x2d,
    0x00,
    0x00,
    0x08,            // Unknown
    0x00,
    0x00,
    0x80,            // Unknown
    0x00,
    0x00,
  };

  void setRollingId(uint8_t rollingId) {
    data[5] &= 0x00; data[5] |= (rollingId & 0xff);
  }

  void setChannel(uint8_t channel) {
    const uint8_t channelCode = (1 << (channel - 1)); // 1=0x1, 2=0x2, 3=0x4
    data[4] &= 0x0f; data[4] |= ((channelCode << 4) & 0xf0);
  }

  void setTemperature(float t) {
    const uint8_t t_sign = t < 0;
    const uint8_t t_deci = ((int)(t * (t_sign ? -10 : 10)) / 1) % 10;
    const uint8_t t_ones = ((int)(t * (t_sign ? -10 : 10)) / 10) % 10;
    const uint8_t t_tens = ((int)(t * (t_sign ? -10 : 10)) / 100) % 10;
    const uint8_t t_huns = ((int)(t * (t_sign ? -10 : 10)) / 1000) % 10;

    data[6] &= 0x0f; data[6] |= ((t_deci << 4) & 0xf0);
    data[7] &= 0xf0; data[7] |= ((t_ones << 0) & 0x0f);
    data[7] &= 0x0f; data[7] |= ((t_tens << 4) & 0xf0);
    data[8] &= 0xfc; data[8] |= ((t_huns << 0) & 0x03);
    data[8] &= 0xf3; data[8] |= ((t_sign << 3) & 0x0c);
  }

  void setHumidity(float h) {
    h += 0.5; // Round to the nearest one by adding 0.5 then truncating the decimal

    const uint8_t h_ones = ((int)(h * 10) / 10) % 10;
    const uint8_t h_tens = ((int)(h * 10) / 100) % 10;

    data[8] &= 0x0f; data[8] |= ((h_ones << 4) & 0xf0);
    data[9] &= 0xf0; data[9] |= ((h_tens << 0) & 0x0f);
  }

  void setLowBattery(bool b) {
    data[6] &= 0xf8; data[6] |= (b ? 0x4 : 0x0);
  }

  void setChecksum() {
    data[10] &= 0x00; data[10] |= (checksumSimple(data, SUM_MASK) & 0xff);
  }

  void setCRC() {
    data[11] &= 0x00; data[11] |= (checksumCRC(data, CRC_MASK, CRC_IV) & 0xff);
  }

  void sendData() {
    configureTimer();

    for (int i = 0; i < DATA_LEN * 8; ++i) { // Bits are transmitted LSB-first
      sendBit((data[i / 8] >> (i % 8)) & 0x1);
    }
    writeSyncBit(LOW); // Don't leave the transmitter on!

    restoreTimer();
  }

  void sendBit(bool val) {
    if (val) {
      sendZero(); // Recall that each bit is sent twice, inverted first
      sendOne();
    } else {
      sendOne();
      sendZero();
    }
  }

  void sendZero() {
    writeSyncBit(LOW);
    writeSyncBit(HIGH);
  }

  void sendOne() {
    writeSyncBit(HIGH);
    writeSyncBit(LOW);
  }

  static uint8_t checksumSimple(const uint8_t data[], uint64_t mask) {
    uint16_t s = 0x0000;

    for (int i = 0; i < 64; ++i) {
      if (!((mask >> i) & 0x1)) continue; // Skip nibbles that aren't set in the mask

      s += (data[i / 2] >> ((i % 2) * 4)) & 0xf; // Sum data nibble by nibble
      s += (s >> 8) & 0x1; // Add any overflow back into the sum
      s &= 0xff;
    }

    return s;
  }


  static uint8_t checksumCRC(const uint8_t data[], uint64_t mask, uint8_t iv) {
    uint16_t s = iv;

    for (int i = 0; i < 64; ++i) {
      if (!((mask >> i) & 0x1)) continue; // Skip nibbles that aren't set in the mask

      uint8_t nibble = (data[i / 2] >> ((i % 2) * 4)) & 0xf;

      for (int j = 3; j >= 0; --j) {
        uint8_t bit = (nibble >> j) & 0x1;

        s <<= 1;
        s |= bit;

        if (s & 0x100) {
          s ^= CRC_POLY;
        }
      }
    }

    for (int i = 0; i < 8; ++i) {
      s <<= 1;
      if (s & 0x100) {
        s ^= CRC_POLY;
      }
    }

    return s;
  }

  void writeSyncBit(bool val) {
    // Synchronise writes to the 2048 Hz timer by sleeping until the timer interrupt
    // This works so long as there's less than 488 us worth of computation between write calls
    sleep_cpu(); // Sleep right before a pin change (rather than after) to ensure all edges are identically spaced
    digitalWrite(pin, val);
  }

  void configureTimer() {
    old_TCCR0A = TCCR0A; // Save and restore Timer0 config since it's used by Arduino for delay()
    old_TCCR0B = TCCR0B;
    old_OCR0A = OCR0A;
    old_TIMSK = TIMSK;

    cli();
    TCCR0A = (1 << WGM01); // CTC (Clear Timer on Compare Match)
    TCCR0B = (1 << CS02) | (1 << CS01) | (1 << CS00); // External clock source on T0 pin
    OCR0A = 0xf; // Output compare register (32 768 Hz / 16 = 2 048 Hz)
    TIMSK = (1 << OCIE0A); // Interrupt on output compare match
    sei();

    set_sleep_mode(SLEEP_MODE_IDLE);
    sleep_enable();
  }

  void restoreTimer() {
    sleep_disable();

    cli();
    TCCR0A = old_TCCR0A;
    TCCR0B = old_TCCR0B;
    OCR0A = old_OCR0A;
    TIMSK = old_TIMSK;
    sei();
  }
};

ISR(TIMER0_COMPA_vect) {
  // Interrupt handler for TIMER0
  // Do nothing; just return control flow to where it was before sleeping
}

#endif /* OS21TX_H */
