/*
 * A fairly dumb wrapper for DHT.h that adds handling for powering a DHT22 sensor
 * on and off via a separate power pin.

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

#ifndef DHTWRAPPER_H
#define DHTWRAPPER_H

#include <DHT.h> // Adafruit's DHT sensor library (https://github.com/adafruit/DHT-sensor-library)

#define SENSOR_TYPE DHT22

class DHTWrapper {
  public:
  const uint8_t dataPin;
  const uint8_t powerPin;
  DHT dht;

  DHTWrapper(uint8_t dataPin,  uint8_t powerPin): dataPin(dataPin), powerPin(powerPin), dht(DHT(dataPin, SENSOR_TYPE)) {}

  void begin() {
    pinMode(powerPin, OUTPUT);
    dht.begin(); // Must call this to set the initial pulltime value (see dht.h/dht.cpp)
    pinMode(dataPin, OUTPUT);
    digitalWrite(dataPin, LOW);
  }

  void powerOn() {
    digitalWrite(powerPin, HIGH);
    // DHT::read() takes care of setting the data pin to the correct state before reading
  }

  void powerOff() {
    digitalWrite(powerPin, LOW);

    pinMode(dataPin, OUTPUT);
    digitalWrite(dataPin, LOW);
  }

  void read(float &t, float &h) {
    // Without force=true, the DHT library only communicates with the seonsor if the last reading was taken more than 2 seconds ago
    // Since power off sleeping stops all the clocks, that 2 seconds would be counting actual CPU run time, which is not helpful for this application
    t = dht.readTemperature(/*fahrenheit*/false, /*force*/true);
    h = dht.readHumidity(/*force*/false);
  }

  bool irrationalReading(float t, float h) {
    return (
      (t == 0.0   && h == 0.0) || // Returned by DHT::read() when the reading times out; rare if the sensor is given long enough to power on, but still possible
      (t == 150.0 && h == 100.0) || // My sensor seems to sometimes return irrational data pairs like this and the next one. Maybe it's a bad part ¯\_(ツ)_/¯
      (t == 50.0  && h == 0.0)
    );
  }
};

#endif /* DHTWRAPPER_H */
