# Pico4Drive Arduino Library

An Arduino-compatible C++ library for controlling all peripherals on the **Pico4Drive** development board.

Pico4Drive is designed for small robotics and high-current control applications, combining motor drivers, power management, expanded analog inputs, and safe voltage interfacing in a single board based on the Raspberry Pi Pico.

This library exposes a API matching the Pico4Drive hardware, allowing control of motors, power management, and analog inputs.

---

## Features

- DC Motor Control
  - Up to 4 brushed DC motors
  - Up to 2A continuous (4A peak) per motor
  - High-resolution PWM (10-bit)
  - Voltage-based or raw PWM control

- Power Management (using the AVR Tiny Controller)
  - Software-controlled power-off
  - Battery voltage monitoring
  - ON button state and press counting
  - Ultra-low power off state (~3 µA total board consumption)

- Analog Inputs
  - 10 total analog inputs
    - 2 direct Pico ADC inputs
    - 8 multiplexed inputs
  - multiplexed ADC read function

- 5V Power Support
  - On-board 5V switching regulator (1A)
  - Safe interfacing with 5V (and up to 12V) signals via two sets of resistor dividers

- GPIO Access
  - All 13 header GPIO pins are free for user applications
  - Some Pico pins are reserved for board functions

---

## Supported Hardware / Software

- Pico4Drive board
- Raspberry Pi Pico / Pico W
- Arduino-Pico core (Earle Philhower or compatible)

---

## Installation

### Arduino Library Manager (if published)

1. Open Arduino IDE
2. Sketch → Include Library → Manage Libraries…
3. Search for "Pico4Drive"
4. Install the latest version

### Manual Installation

1. Download or clone this repository
2. Copy the folder into your Arduino libraries directory
3. Restart the Arduino IDE

---

## Driver Enumeration

Motors are identified using the `driver_num_t` enum:

    typedef enum {
      p2d_drv1 = 0,
      p2d_drv2,
      p2d_drv3,
      p2d_drv4
    } driver_num_t;

---

## Getting Started

### Include the Library

    #include <Pico4Drive.h>

### Create the Pico4Drive Object

    pico4drive_t pico4drive;

### Initialize the Board

    void setup() {
        pico4drive.init();          // Default PWM frequency: 16 kHz
    }

You may optionally specify a custom PWM frequency:

    pico4drive.init(20000);         // 20 kHz PWM

---

## PWM Resolution and Limits

- PWM resolution is **10 bits**
- Maximum PWM value:

    pico4drive.analogWriteMax   // 1023

- The library internally limits PWM using:

    pico4drive.PWM_limit;

This value can be adjusted by user code to restrict the maximum motor voltage.
By default, it is set to produce approximately **5.5 V output from a 7.4 V supply** (2 × Li-Ion cells).

---

## Motor Control

### Set Raw PWM Output

Set motor PWM directly (positive or negative values control direction):

    pico4drive.set_driver_PWM(400, p2d_drv1);
    pico4drive.set_driver_PWM(-600, p2d_drv2);

### Set Motor Voltage

Set motor output based on a desired voltage:

    pico4drive.set_driver_voltage(4.5, p2d_drv3);

The voltage is internally converted to PWM using the measured current battery voltage.

---

## Analog Inputs

### Read Any ADC Channel

Reads multiplexed analog inputs:

    uint16_t value = pico4drive.read_adc(0);

---

## Battery and Button State

The following fields are updated by calling `update()`:

    pico4drive.battery_voltage;
    pico4drive.button_state;
    pico4drive.button_press_count;

Example:

    void loop() {
        pico4drive.update();

        if (pico4drive.button_state) {
            // Button currently pressed
        }
    }

`button_press_count` can be reset by user code as needed.

---

## Power Management

### Power Off the Board

Requests the AVR Tiny controller to shut down the board:

    pico4drive.power_off();

This places the board into its ultra-low-power sleep state.

---

## Typical Main Loop Pattern

    void loop() {
        pico4drive.update();

        // Application logic here
    }

Calling `update()` regularly ensures battery voltage and button state remain current.

---

## API Overview

- Constructor:
  - pico4drive_t()

- Initialization:
  - init(PWM_freq = 16000)

- Motor control:
  - set_driver_PWM(new_PWM, driver_num)
  - set_driver_voltage(new_voltage, driver_num)
  - voltage_to_PWM(voltage)

- Analog input:
  - read_adc(channel)

- Power management:
  - power_off()

- State update:
  - update()

- Public state variables:
  - battery_voltage
  - button_state
  - button_press_count
  - PWM_limit
  - analogWriteBits
  - analogWriteMax

---

## Design Notes

- GPIOs used internally for:
  - Motor drivers
  - Analog multiplexer
  - AVR Tiny communication
  are not exposed on the headers.
- All 13 header GPIO pins connect directly to the Pico.
- AVR Tiny communication uses a single Pico GPIO pin.

---

## License

BSD 2-Clause

---

## Contributing

Contributions, bug reports, and feature requests are welcome.
Please open an issue or submit a pull request.
