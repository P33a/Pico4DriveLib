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

#include "Arduino.h"
#include "Pico4Drive.h"

#define ADC_IN_PIN	28

#define MUXA_PIN	18
#define MUXB_PIN	19
#define MUXC_PIN	20

#define TINY_CTRL_PIN	21

#define DRIVER_1A_PIN 11
#define DRIVER_1B_PIN 10

#define DRIVER_2A_PIN 13
#define DRIVER_2B_PIN 12

#define DRIVER_3A_PIN 15
#define DRIVER_3B_PIN 14

#define DRIVER_4A_PIN 17
#define DRIVER_4B_PIN 16


static const int p4d_pwm_pins[4][2] = {{DRIVER_1A_PIN, DRIVER_1B_PIN},
                                       {DRIVER_2A_PIN, DRIVER_2B_PIN},
                                       {DRIVER_3A_PIN, DRIVER_3B_PIN},
                                       {DRIVER_4A_PIN, DRIVER_4B_PIN}};


// NOPIN definition has changed locations with arduino versions, between
// SerialPIO::NOPIN and just NOPIN but is always 255. So just use the value to
// avoid version lock
SerialPIO SerialTiny(255, 21);

pico4drive_t::pico4drive_t()
{
  battery_voltage = 7.4;
  PWM_limit = analogWriteMax * 5.5 / battery_voltage;
}

void pico4drive_t::init(uint32_t PWM_freq)
{
  // ADC mux pins
  pinMode(MUXA_PIN, OUTPUT);
  pinMode(MUXB_PIN, OUTPUT);
  pinMode(MUXC_PIN, OUTPUT);

  SerialTiny.begin();

  analogWriteResolution(analogWriteBits);
  analogWriteFreq(PWM_freq); //16000

  for (int i = 0; i < 4; i++) {
    pinMode(p4d_pwm_pins[i][0], OUTPUT);
    pinMode(p4d_pwm_pins[i][1], OUTPUT);
  }

  button_press_count = 0;
}

void pico4drive_t::update(void)
{
	int c, bstate;

	while (SerialTiny.available()) {
    // Read battery voltage and On button state from the Tiny controller
    c = SerialTiny.read();

		bstate = c & 1;
		if (bstate && !button_state)
			button_press_count++;
		button_state = bstate;

		battery_voltage = 1e-3 * ((c >> 1) * 50 + 4800);
	}
}

static void adc_set_channel(int channel)
{
	gpio_put_masked(digitalPinToBitMask(MUXA_PIN) | digitalPinToBitMask(MUXB_PIN) | digitalPinToBitMask(MUXC_PIN), channel << MUXA_PIN);
  //digitalWrite(MUXA_PIN, channel & 1);
  //digitalWrite(MUXB_PIN, (channel >> 1) & 1);
  //digitalWrite(MUXC_PIN, (channel >> 2) & 1);
}

uint16_t pico4drive_t::read_adc(int channel)
{
	adc_set_channel(channel); // Switch external MUX to the desired channel
	delayMicroseconds(10);
	return analogRead(A2);    // The mux connects to analog input A2
}

int pico4drive_t::voltage_to_PWM(float u)
{
  return (u / battery_voltage) * analogWriteMax;
}

void pico4drive_t::set_driver_PWM(int new_PWM, int pin_a, int pin_b)
{
  int PWM_max = analogWriteMax;

  if (new_PWM >  PWM_limit) new_PWM =  PWM_limit;
  if (new_PWM < -PWM_limit) new_PWM = -PWM_limit;

  if (new_PWM == 0) {  // Both outputs 0 -> A = H, B = H
    analogWrite(pin_a, PWM_max);
    analogWrite(pin_b, PWM_max);
  } else if (new_PWM > 0) {
    analogWrite(pin_a, PWM_max - new_PWM);
    analogWrite(pin_b, PWM_max);
  } else {
    analogWrite(pin_a, PWM_max);
    analogWrite(pin_b, PWM_max + new_PWM);
  }
}

void pico4drive_t::set_driver_PWM(int new_PWM, driver_num_t driver_num)
{
  set_driver_PWM(new_PWM, p4d_pwm_pins[driver_num][0], p4d_pwm_pins[driver_num][1]);
}

void pico4drive_t::power_off(void)
{
  pinMode(TINY_CTRL_PIN, OUTPUT);
  digitalWrite(TINY_CTRL_PIN, 0);
	while (1)
		;
}


int pico4drive_t::set_driver_voltage(float new_voltage, driver_num_t driver_num)
{
  int pwm = voltage_to_PWM(new_voltage);
  set_driver_PWM(pwm, driver_num);
  return pwm;
}
