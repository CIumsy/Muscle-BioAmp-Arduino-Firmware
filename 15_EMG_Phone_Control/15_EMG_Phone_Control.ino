// Upside Down Labs invests time and resources providing this open source code,
// please support Upside Down Labs and open-source hardware by purchasing
// products from Upside Down Labs!

// Copyright (c) 2026 Krishnanshu Mittal - krishnanshu@upsidedownlabs.tech
// Copyright (c) 2026 Upside Down Labs - contact@upsidedownlabs.tech

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// At Upside Down Labs, we create open-source DIY neuroscience hardware and software.
// Our mission is to make neuroscience affordable and accessible for everyone.
// By supporting us with your purchase, you help spread innovation and open science.
// Thank you for being part of this journey with us!

#include <Keyboard.h>

// Comment out to stop serial printing
// #define DEBUG

// Samples per second
#define SAMPLE_RATE 500

// Make sure to set the same baud rate on your Serial Monitor/Plotter
#define BAUD_RATE 115200

// Change if your sensors are connected to different analog pins
#define INPUT_PIN1 A0  // Left forearm
#define INPUT_PIN2 A2  // Right forearm

// Envelope buffer size
// High value -> smooth but less responsive
// Low value -> not smooth but responsive
#define BUFFER_SIZE 64

// Envelope level that counts as a clench (tune using Serial Plotter)
#define THRESHOLD1 50
#define THRESHOLD2 50

// Timing in ms
#define LONG_MS 500        // clench longer than this = long clench
#define BOTH_MS 150        // both clenches starting within this = both together
#define MIN_MS 50          // ignore clenches shorter than this
#define REFRACTORY_MS 200  // wait after each key
#define KEY_MS 40          // how long a key stays pressed

// Keys
#define CH1_SHORT_KEY KEY_LEFT_ARROW
#define CH2_SHORT_KEY KEY_RIGHT_ARROW
#define CH1_LONG_KEY KEY_UP_ARROW
#define CH2_LONG_KEY KEY_DOWN_ARROW
#define BOTH_KEY ' '

class Envelope {
  int circular_buffer[BUFFER_SIZE] = {0};
  int data_index = 0, sum = 0;

public:
  // Envelope detection algorithm
  int getEnvelope(int abs_emg) {
    sum -= circular_buffer[data_index];
    sum += abs_emg;
    circular_buffer[data_index] = abs_emg;
    data_index = (data_index + 1) % BUFFER_SIZE;
    return (sum / BUFFER_SIZE) * 2;
  }
};

class EMGFilter {
  float state[4][2] = {{0}};

public:
  // Band-Pass Butterworth IIR digital filter, generated using filter_gen.py.
  // Sampling rate: 500.0 Hz, frequency: [74.5, 149.5] Hz.
  // Filter is order 4, implemented as second-order sections (biquads).
  // Reference:
  // https://docs.scipy.org/doc/scipy/reference/generated/scipy.signal.butter.html
  // https://courses.ideate.cmu.edu/16-223/f2020/Arduino/FilterDemos/filter_gen.py
  float process(float input) {
    float output = input;
    {
      float &z1 = state[0][0], &z2 = state[0][1];  // filter section state
      float x = output - 0.05159732 * z1 - 0.36347401 * z2;
      output = 0.01856301 * x + 0.03712602 * z1 + 0.01856301 * z2;
      z2 = z1;
      z1 = x;
    }
    {
      float &z1 = state[1][0], &z2 = state[1][1];  // filter section state
      float x = output - -0.53945795 * z1 - 0.39764934 * z2;
      output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
      z2 = z1;
      z1 = x;
    }
    {
      float &z1 = state[2][0], &z2 = state[2][1];  // filter section state
      float x = output - 0.47319594 * z1 - 0.70744137 * z2;
      output = 1.00000000 * x + 2.00000000 * z1 + 1.00000000 * z2;
      z2 = z1;
      z1 = x;
    }
    {
      float &z1 = state[3][0], &z2 = state[3][1];  // filter section state
      float x = output - -1.00211112 * z1 - 0.74520226 * z2;
      output = 1.00000000 * x + -2.00000000 * z1 + 1.00000000 * z2;
      z2 = z1;
      z1 = x;
    }
    return output;
  }
};

// Clench state of one channel
struct Channel {
  int threshold;
  uint8_t shortKey;
  uint8_t longKey;
  bool active = false;
  bool used = false;  // true once this clench has sent a key
  unsigned long start = 0;
};

EMGFilter filter1, filter2;
Envelope env1, env2;
Channel ch1 = { THRESHOLD1, CH1_SHORT_KEY, CH1_LONG_KEY };
Channel ch2 = { THRESHOLD2, CH2_SHORT_KEY, CH2_LONG_KEY };

unsigned long lastKeyTime = 0;
unsigned long keyUpTime = 0;
bool keyDown = false;

void sendKey(uint8_t key) {
  if (keyDown) Keyboard.releaseAll();
  Keyboard.press(key);
  digitalWrite(LED_BUILTIN, HIGH);
  keyDown = true;
  keyUpTime = millis() + KEY_MS;
  lastKeyTime = millis();
}

void updateChannel(Channel &ch, int envelope, unsigned long now) {
  if (!ch.active && envelope > ch.threshold && now - lastKeyTime > REFRACTORY_MS) {
    ch.active = true;
    ch.used = false;
    ch.start = now;
  } else if (ch.active && envelope <= ch.threshold) {
    ch.active = false;
    // Short clench sends its key on release
    if (!ch.used && now - ch.start >= MIN_MS) sendKey(ch.shortKey);
  }
}

void checkLong(Channel &ch, unsigned long now) {
  if (ch.active && !ch.used && now - ch.start >= LONG_MS) {
    sendKey(ch.longKey);
    ch.used = true;
  }
}

void setup() {
#ifdef DEBUG
  // Initialize serial communication
  Serial.begin(BAUD_RATE);
#endif
  Keyboard.begin();
  pinMode(LED_BUILTIN, OUTPUT);
}

void loop() {
  // Calculate elapsed time
  static unsigned long past = 0;
  unsigned long present = micros();
  unsigned long interval = present - past;
  past = present;

  // Run timer
  static long timer = 0;
  timer -= interval;

  // Sample and get envelope
  if (timer < 0) {
    timer += 1000000 / SAMPLE_RATE;
    // RAW EMG
    int sensor_value1 = analogRead(INPUT_PIN1);
    int sensor_value2 = analogRead(INPUT_PIN2);

    // Filtered EMG
    int signal1 = filter1.process(sensor_value1);
    int signal2 = filter2.process(sensor_value2);

    // EMG envelope
    int envelope1 = env1.getEnvelope(abs(signal1));
    int envelope2 = env2.getEnvelope(abs(signal2));

#ifdef DEBUG
    Serial.print(envelope1);
    Serial.print(", ");
    Serial.println(envelope2);
#endif

    unsigned long now = millis();
    updateChannel(ch1, envelope1, now);
    updateChannel(ch2, envelope2, now);

    // Both together
    if (ch1.active && ch2.active && !ch1.used && !ch2.used) {
      long gap = (long)ch1.start - (long)ch2.start;
      if (abs(gap) <= BOTH_MS) {
        sendKey(BOTH_KEY);
        ch1.used = true;
        ch2.used = true;
      }
    }

    // Long clench
    checkLong(ch1, now);
    checkLong(ch2, now);
  }

  // Release key after KEY_MS
  if (keyDown && (long)(millis() - keyUpTime) >= 0) {
    Keyboard.releaseAll();
    digitalWrite(LED_BUILTIN, LOW);
    keyDown = false;
  }
}