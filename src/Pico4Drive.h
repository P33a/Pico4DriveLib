/* Copyright (c) 2025  Paulo Costa, Paulo Marques
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.
   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

#ifndef PICO4DRIVE_H
#define PICO4DRIVE_H

#include "Arduino.h"

#ifdef ARDUINO_ARCH_RP2040

typedef enum {
  p2d_drv1 = 0,
  p2d_drv2,
  p2d_drv3,
  p2d_drv4
} driver_num_t;

class pico4drive_t {
  public:
    // number of bits for the PWM
    static constexpr int analogWriteBits = 10;
    // maximum PWM value
    static constexpr int analogWriteMax = (1 << analogWriteBits) - 1;
    // the PWM is internally limited to this value. Can be used to restrict the
    // PWM to lower values. It is initially set to produce 5.5V from a 7.4V
    // source (2x Li-Ion batteries)
    int PWM_limit;

    // current battery voltage
    float battery_voltage;
    // current ON button state
    int button_state;
    // number of times the button has ben pressed. Can be reset by user code
    int button_press_count;

    pico4drive_t();
    void init(uint32_t PWM_freq = 16000);

    uint16_t read_adc(int channel);
    void set_driver_PWM(int new_PWM, int pin_a, int pin_b);
    void set_driver_PWM(int new_PWM, driver_num_t driver_num);
    int voltage_to_PWM(float u);

    int set_driver_voltage(float new_voltage, driver_num_t driver_num);

    void power_off(void);

    void update(void);
};

typedef pico4drive_t Pico4Drive;

#else // ARCH
#error PicoEncoder library requires a PIO peripheral and only works on the RP2040 architecture
#endif

#endif // PICO4DRIVE_H
