#include <Pico4Drive.h>

// Create Pico4Drive object
pico4drive_t pico4drive;

// Timing for inactivity shutdown
unsigned long last_activity_ms = 0;
const unsigned long INACTIVITY_TIMEOUT_MS = 60000; // 60 seconds

void setup() {
    // Initialize serial port
    Serial.begin(115200);

    Serial.println("Pico4Drive example starting...");

    // Initialize Pico4Drive with default PWM frequency (16 kHz)
    pico4drive.init();

    last_activity_ms = millis();
}

void loop() {
    // Update internal state (battery voltage, button state, etc.)
    pico4drive.update();

    // ---- Button handling ----
    if (pico4drive.button_state) {
    //if (false) {
        last_activity_ms = millis();

        // Drive all motors at 4.0 V while button is pressed
        pico4drive.set_driver_voltage(4.0, p2d_drv1);
        pico4drive.set_driver_voltage(4.0, p2d_drv2);
        pico4drive.set_driver_voltage(4.0, p2d_drv3);
        pico4drive.set_driver_voltage(4.0, p2d_drv4);
    } else {
        // Stop all motors
        pico4drive.set_driver_PWM(0, p2d_drv1);
        pico4drive.set_driver_PWM(0, p2d_drv2);
        pico4drive.set_driver_PWM(0, p2d_drv3);
        pico4drive.set_driver_PWM(0, p2d_drv4);
    }

    // ---- Analog input example ----
    uint16_t adc_value = pico4drive.read_adc(0);

    // ---- Serial output ----
    Serial.print("ADC0: ");
    Serial.print(adc_value);
    Serial.print(" | Battery: ");
    Serial.print(pico4drive.battery_voltage, 2);
    Serial.print(" V | Button presses: ");
    Serial.println(pico4drive.button_press_count);

    // ---- Inactivity power-off ----
    if (millis() - last_activity_ms > INACTIVITY_TIMEOUT_MS) {
        Serial.println("Inactivity timeout, powering off...");
        delay(50); // Allow message to flush
        pico4drive.power_off();
    }

    delay(200);
}
