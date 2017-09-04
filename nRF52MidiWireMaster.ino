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

#define BLE_PACKET_SIZE 20

// BLE MIDI
BLEPeripheral BLE;
BLEService midiSvc("03B80E5A-EDE8-4B33-A751-6CE34EC4C700");
BLECharacteristic midiChar("7772E5DB-3868-4112-A1A9-F2669D106BF3",
    BLEWrite | BLEWriteWithoutResponse | BLENotify | BLERead, BLE_PACKET_SIZE);

boolean connected;

uint8_t midiData[BLE_PACKET_SIZE];
int byteOffset = 0;
uint8_t lastStatus;
uint16_t lastTime;

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
  while (!isFull() && receive(message) && connected) {
    dispatch(message);
  }
  sendMessages();
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

// returns false if loadMessage failed due to overflow 
boolean dispatch(uint8_t message[]) {
  switch (message[0] & 0xF0) {
    case 0x80: // Note Off
    case 0x90: // Note On
    case 0xA0: // After Touch Poly
    case 0xB0: // Control Change
    case 0xE0: // Pitch Bend
      return loadMessage(3, message[0], message[1], message[2]);

    case 0xC0: // Program Change
    case 0xD0: // After Touch Channel
      return loadMessage(2, message[0], message[1], 0);
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
  sendMessage(3, 0, 0, 0);

  BLE.begin();
}

boolean isEmpty() {
  return byteOffset == 0;
}

boolean isFull() {
  return byteOffset > BLE_PACKET_SIZE - 4;
}

boolean loadMessage(int msglen, uint8_t status, uint8_t byte1, uint8_t byte2) {
  // Assert BLE_PACKET_SIZE > 4
  if (isFull()) return false;
  boolean wasEmpty = isEmpty();
  uint16_t timestamp = (uint16_t) millis();
  uint8_t msgTs = timestamp;
  msgTs |= 1 << 7;  // set the 7th bit
  if (wasEmpty) {
    uint8_t headTs = timestamp >> 7;
    headTs |= 1 << 7;  // set the 7th bit
    headTs &= ~(1 << 6);  // clear the 6th bit
    midiData[byteOffset++] = headTs;
  }
  if (wasEmpty || lastStatus != status) {
    midiData[byteOffset++] = msgTs;
    midiData[byteOffset++] = status;
  } else if (lastTime != timestamp) {
    midiData[byteOffset++] = msgTs;   
  }
  midiData[byteOffset++] = byte1;
  if (msglen == 3) {
    midiData[byteOffset++] = byte2;
  }
  lastStatus = status;
  lastTime = timestamp;
  return true;
}

boolean sendMessage(int msglen, uint8_t status, uint8_t byte1, uint8_t byte2) {
  return loadMessage(msglen, status, byte1, byte2) && sendMessages();
}

boolean sendMessages() {
  if (isEmpty()) return false;
  midiChar.setValue(midiData, byteOffset);
  byteOffset = 0;
  return true;
}

