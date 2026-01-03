#include <pico4drive.h>

pico4drive_t pico4drive;

// ADC channels
const int THROTTLE_ADC = 0;   // Forward / reverse
const int STEERING_ADC = 1;   // Left / right

// Motor drivers
const driver_num_t LEFT_MOTOR  = p2d_drv1;
const driver_num_t RIGHT_MOTOR = p2d_drv2;

// Maximum motor voltage (keep below battery voltage)
const float MAX_MOTOR_VOLTAGE = 5.0;

// Deadzone for joystick center
const int ADC_DEADZONE = 40;

// ADC range (RP2040 ADC is 12-bit, but your abstraction returns uint16_t)
const int ADC_CENTER = 2048;   // Adjust if needed
const int ADC_MAX    = 4095;

float map_adc_to_unit(int value) {
    int delta = value - ADC_CENTER;

    if (abs(delta) < ADC_DEADZONE) {
        return 0.0f;
    }

    return (float)delta / (float)(ADC_MAX / 2);
}

void setup() {
    pico4drive.init();
}

void loop() {
    pico4drive.update();

    // Read joystick
    uint16_t throttle_adc = pico4drive.read_adc(THROTTLE_ADC);
    uint16_t steering_adc = pico4drive.read_adc(STEERING_ADC);

    // Convert ADC values to -1.0 .. +1.0
    float throttle = map_adc_to_unit(throttle_adc);
    float steering = map_adc_to_unit(steering_adc);

    // Tank / differential drive mixing
    float left_cmd  = throttle + steering;
    float right_cmd = throttle - steering;

    // Clamp to -1.0 .. +1.0
    left_cmd  = constrain(left_cmd,  -1.0f, 1.0f);
    right_cmd = constrain(right_cmd, -1.0f, 1.0f);

    // Convert to motor voltages
    float left_voltage  = left_cmd  * MAX_MOTOR_VOLTAGE;
    float right_voltage = right_cmd * MAX_MOTOR_VOLTAGE;

    // Apply motor commands
    pico4drive.set_driver_voltage(left_voltage,  LEFT_MOTOR);
    pico4drive.set_driver_voltage(right_voltage, RIGHT_MOTOR);

    delay(10);
}
