#include "core_pins.h"
#include "pins_arduino.h"

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
}

void loop()
{
    digitalWrite(LED_BUILTIN, HIGH);
    delay(1000);
    digitalWrite(LED_BUILTIN, LOW);
    delay(1000);
}


/*
#include "HardwareSerial.h"
#include "usb_serial.h"

void setup() {
    Serial.begin(115200);      // USB serial
    Serial1.begin(9600);       // Hardware UART1 (RX=0, TX=1)

    Serial.println("Starting UART test...");
}

void loop() {
    // Send data from USB to Serial1
    if (Serial.available()) {
        char c = Serial.read();
        Serial1.write(c);
    }

    // Send data from Serial1 to USB
    if (Serial1.available()) {
        char c = Serial1.read();
        Serial.write(c);
    }
}
*/
