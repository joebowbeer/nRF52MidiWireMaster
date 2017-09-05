/* nRF52MidiWireMaster
 * 
 * USB MIDI to BLE MIDI converter for Adafruit and SparkFun nRF52 breakout boards.
 * 
 * MIDI messages from a USB MIDI instrument are received by a USB Host Shield
 * attached to another Arduino, fowarded to the nF52 board running this sketch, 
 * and then transmitted as BLE MIDI messages.
 * 
 * Run this sketch on a nRF52 board and run the UsbMidiWireSlave sketch on an 
 * Arduino connected to a USB Host Shield. The two sketches communicate using I2C.
 * 
 * Connections using bidirectional level shifter:
 * 
 * 1. nRF52 SDA/SCL to Arduino SDA/SCL
 * 2. nRF52 RST to Arduino RST
 * 
 * NOTES:
 * 
 * 1. Adafruit Bluefruit nRF52 Feather's SDA(25) and SCL(26) require external 
 * pull-up resistors (4k7)
 *  
 * 2. SparkFun nRF52 Breakout requires SDA(24) and SCL(25) to be redefined in 
 * variants.h
 */

#include <BLEPeripheral.h>
#include <Wire.h>
#include "BleMidiEncoder.h"

#if defined(ARDUINO_FEATHER52)

// Blue LED
#define LED_PIN LED_BLUE
#define LED_ACTIVE LED_STATE_ON

#elif defined(ARDUINO_NRF52_DK)

// LED on pin 7 is active low
#define LED_PIN 7
#define LED_ACTIVE LOW

#else
#error "Unsupported platform." 
#endif

const uint8_t SLAVE_ADDRESS = 0x42;

// BLE MIDI
BLEPeripheral BLE;
BLEService midiSvc("03B80E5A-EDE8-4B33-A751-6CE34EC4C700");
BLECharacteristic midiChar("7772E5DB-3868-4112-A1A9-F2669D106BF3",
    BLEWrite | BLEWriteWithoutResponse | BLENotify | BLERead,
    BLE_MIDI_PACKET_SIZE);

class NordicBleMidiEncoder: public BleMidiEncoder {
  boolean setValue(const unsigned char value[], unsigned char length) {
    return midiChar.setValue(value, length);
  }
};

NordicBleMidiEncoder encoder;
boolean connected;

void setup() {
  connected = false;
  pinMode(LED_PIN, OUTPUT);
  displayConnectionState();
  setupBle();
  Wire.begin();
  delay(1000); // wait for slave setup
}

void loop() {
  BLE.poll();
  connected = !!BLE.central();
  displayConnectionState();

  uint8_t message[3];
  while (!encoder.isFull() && receive(message) && connected) {
    dispatch(message);
  }
  encoder.sendMessages();
}

boolean receive(uint8_t message[]) {
  if (Wire.requestFrom(SLAVE_ADDRESS, 3)) {
    uint8_t status = Wire.read();
    uint8_t byte1 = Wire.read();
    uint8_t byte2 = Wire.read();
    if (status) {
      message[0] = status;
      message[1] = byte1;
      message[2] = byte2;
      return true;
    }
  }
  return false;
}

// returns false if encoder failed due to overflow 
boolean dispatch(uint8_t message[]) {
  switch (message[0] & 0xF0) {
    case 0x80: // Note Off
    case 0x90: // Note On
    case 0xA0: // After Touch Poly
    case 0xB0: // Control Change
    case 0xE0: // Pitch Bend
      return encoder.appendMessage(message[0], message[1], message[2]);

    case 0xC0: // Program Change
    case 0xD0: // After Touch Channel
      return encoder.appendMessage(message[0], message[1]);
  }
  return true;
}

void displayConnectionState() {
  // LED is lit until we're connected
  digitalWrite(LED_PIN, connected ? !LED_ACTIVE : LED_ACTIVE);
}

void setupBle() {
  BLE.setConnectionInterval(6, 12); // 7.5 to 15 millis

  // set the local name peripheral advertises
  BLE.setLocalName("nRFMIDI");

  // set the UUID for the service this peripheral advertises
  BLE.setAdvertisedServiceUuid(midiSvc.uuid());

  // add service and characteristic
  BLE.addAttribute(midiSvc);
  BLE.addAttribute(midiChar);

  // set an initial value for the characteristic
  encoder.sendMessage(0, 0, 0);

  BLE.begin();
}

