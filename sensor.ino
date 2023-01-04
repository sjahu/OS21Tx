/*
 * ATtiny85-based temperature/humidity sensor compatible with the Oregon Scientific v2.1
 * 433.92 MHz weather sensor protocol.
 *
 * This sketch replicates the behaviour of the Oregon Scientific THGR122NX sensor.
 *
 * Most of the pin assignments defined below are flexible; the only one that isn't is T0,
 * which must be connected to the external oscillator clocking Timer/Counter0. On the
 * ATtiny85, T0 is on PB2.
 *
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

#include <avr/sleep.h>
#include <avr/wdt.h>

#include "DHTWrapper.h"
#define DHT_DATA_PIN 4 // I/O for the temperature/humidity sensor
#define DHT_POWER_PIN 3
DHTWrapper dht = DHTWrapper(DHT_DATA_PIN, DHT_POWER_PIN);

#define T0_PIN 2
#define T0_XO_POWER_PIN 1 // Power for the crystal oscillator clocking Timer0
#include "OS21Tx.h"
#define TX_PIN 0 // Output for the 433.92 Mhz modulator
OS21Tx tx = OS21Tx(TX_PIN);

#include <EEPROM.h>
#define RESET_COUNT_ADDR 0 // Where to store the current reset count (used for seeding RNG and saving channel setting)

#define RESET_PIN 5

#define LOW_BATTERY 2000 // Threshold in mV (2V picked with 2x 1.5V AAA cells in mind. Adjust as required.)

void setup() {
  cli();
  uint8_t _MCUSR = MCUSR;
  MCUSR = 0; // As per the datasheet, if a watchdog timer reset status flag is set, it must be cleared ASAP
  wdt_disable(); // Otherwise, the watchdog timer will start over immediately with the smallest prescale value
  sei();

  ADCSRA = 0; // Disable Analog to Digital Converter (wastes power)

  pinMode(RESET_PIN, INPUT_PULLUP); // Leaving the reset pin floating can trigger random resets

  pinMode(T0_XO_POWER_PIN, OUTPUT);

  uint32_t resetCount;
  EEPROM.get(RESET_COUNT_ADDR, resetCount);
  if (_MCUSR & (1 << EXTRF)) { // Increment the saved channel if an external reset was triggered
    ++resetCount;
    EEPROM.put(RESET_COUNT_ADDR, resetCount);
  }

  uint8_t channel = (resetCount % 3) + 1; // i.e. 1, 2, or 3
  randomSeed(resetCount); // Seed RNG for picking Rolling ID

  dht.begin();
  tx.begin(channel, random(256));
}

void loop() {
  digitalWrite(T0_XO_POWER_PIN, HIGH);
  dht.powerOn();

  // Sleep for 2 seconds to give the DHT22 and crystal oscillator a chance to wake up
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  WDTCR = (1 << WDIE) | (1 << WDCE) | (1 << WDE) | (1 << WDP2) | (1 << WDP1) | (1 << WDP0); // Enable watchdog timer interrupt with 2 second countdown (see ATtiny85 datasheet, section 8.5)
  wdt_reset(); // With the WDE bit set, too, WDIE is cleared when a timeout occurs, putting the watchdog in reset mode
  sleep_cpu(); // So if something in the following sensor or tx code hangs for more than 2s, the watchdog will trigger a chip reset
  sleep_disable();

  float t, h;
  dht.read(t, h);

  if (dht.irrationalReading(t, h)) {
    return; // Try again if we get a known bad reading
  }

  dht.powerOff();

#ifdef LOW_BATTERY
  tx.transmit(t, h, getVcc() < LOW_BATTERY);
#else
  tx.transmit(t, h);
#endif

  digitalWrite(T0_XO_POWER_PIN, LOW);

  // Sleep for 8*5 = 40 seconds (8 seconds is the max for the watchdog timer prescaler)
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  wdt_disable(); // Clear WDE to put watchdog timer back in interrupt-only mode
  WDTCR = (1 << WDIE) | (1 << WDCE) | (1 << WDP3) | (1 << WDP0);
  wdt_reset();
  sleep_cpu(); // This is probably overly cautious, but I'm not using a loop here
  sleep_cpu(); // because if cosmic rays or something disrupted the counter, we
  sleep_cpu(); // could be sleeping for a very long time, since the watchdog timer
  sleep_cpu(); // reset is disabled at this point
  sleep_cpu();
  sleep_disable();
}

ISR(WDT_vect) {
  // Interrupt handler for watchdog timer
  // Do nothing; just return control flow to where it was before sleeping
}

long getVcc() {
  uint8_t _ADCSRA = ADCSRA;

  ADCSRA = (1 << ADEN); // Enable ADC
  ADMUX = (1 << MUX3) | (1 << MUX2); // Vcc as voltage reference; 1.1V bandgap voltage as measurement target

  delay(2); // Allow ADC to settle after switching to internal voltage reference (as per datasheet)

  ADCSRA |= (1 << ADSC); // Start conversion
  while (ADCSRA & (1 << ADSC));

  uint8_t adcl  = ADCL;
  uint8_t adch = ADCH;

  uint16_t result = (adch << 8) | (adcl << 0); // result is 10 bits (max 1023)

  ADCSRA = _ADCSRA;

  return 1125300L / result; // Vcc in mV (1.1 * 1023 * 1000 = 1125300)
}
