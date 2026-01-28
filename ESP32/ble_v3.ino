#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

#define SERVICE_UUID                "8c055cbe-b517-4134-bb83-a4c4b05944ba"

// Existing "freeSpaces" characteristic (already in your project)
#define SENSOR_CHARACTERISTIC_UUID  "24b27f5a-8187-467c-81c9-b4480741d580"

// NEW: mode characteristic UUID (generate your own if you prefer)
#define MODE_CHARACTERISTIC_UUID    "3dfb7f69-1a52-4ce8-8b0d-8c2d0f0d7a11"

#define RF_FREQUENCY                915000000
#define TX_OUTPUT_POWER             5
#define LORA_BANDWIDTH              0
#define LORA_SPREADING_FACTOR       7
#define LORA_CODINGRATE             1
#define LORA_PREAMBLE_LENGTH        8
#define LORA_SYMBOL_TIMEOUT         0
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON        false

#define BUFFER_SIZE  32

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED,
                           GEOMETRY_128_64, RST_OLED);

static RadioEvents_t RadioEvents;
bool lora_idle = true;

char rxpacket[BUFFER_SIZE];

// BLE globals
BLEServer* pServer = NULL;
BLECharacteristic* pFreeSpacesCharacteristic = NULL;
BLECharacteristic* pModeCharacteristic = NULL;

bool deviceConnected = false;
bool oldDeviceConnected = false;

int freeSpaces = 0;

// Track current mode for display + BLE
// Allowed values: "IMU" or "DIST"
String currentMode = "IMU";

void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void showStatusOLED() {
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(display.getWidth()/2, 16, "Free Spaces: " + String(freeSpaces));
  display.drawString(display.getWidth()/2, 40, "Mode: " + currentMode);
  display.display();
}

void notifyFreeSpaces() {
  if (deviceConnected && pFreeSpacesCharacteristic) {
    pFreeSpacesCharacteristic->setValue(String(freeSpaces).c_str());
    pFreeSpacesCharacteristic->notify();
  }
}

void notifyMode() {
  if (deviceConnected && pModeCharacteristic) {
    pModeCharacteristic->setValue(currentMode.c_str());
    pModeCharacteristic->notify();
  }
}

// Sends 6-byte packet: 0x42 0xE1 + int32 freeSpaces (big-endian) (your existing design)
void update_all_freeSpaces() {
  uint8_t packet[6];
  packet[0] = 0x42;
  packet[1] = 0xE1;
  packet[2] = (freeSpaces >> 24) & 0xFF;
  packet[3] = (freeSpaces >> 16) & 0xFF;
  packet[4] = (freeSpaces >> 8) & 0xFF;
  packet[5] = (freeSpaces) & 0xFF;

  Radio.Send(packet, 6);
  lora_idle = false;
}

void broadcastModeLoRa() {
  // Broadcast ASCII "MODE:IMU" or "MODE:DIST" to all sensor nodes
  String msg = "MODE:" + currentMode;
  Radio.Send((uint8_t*)msg.c_str(), msg.length());
  lora_idle = false;
}

void OnTxDone(void) { lora_idle = true; }
void OnTxTimeout(void) { Radio.Sleep(); lora_idle = true; }

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer*) override { deviceConnected = true; }
  void onDisconnect(BLEServer*) override { deviceConnected = false; }
};

// Writes to freeSpaces characteristic (existing behavior) [9]
class FreeSpacesCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue();
    if (!value.length()) return;

    String rxStr(value.c_str());
    rxStr.trim();

    if (rxStr.equalsIgnoreCase("inc")) {
      freeSpaces++;
      showStatusOLED();
      notifyFreeSpaces();
      update_all_freeSpaces();
      return;
    }

    if (rxStr.equalsIgnoreCase("dec")) {
      freeSpaces--;
      if (freeSpaces < 0) freeSpaces = 0;
      showStatusOLED();
      notifyFreeSpaces();
      update_all_freeSpaces();
      return;
    }

    // Numeric set
    if (rxStr.length() > 0 && rxStr.toInt() == rxStr.toFloat()) {
      freeSpaces = rxStr.toInt();
      if (freeSpaces < 0) freeSpaces = 0;
      showStatusOLED();
      notifyFreeSpaces();
      update_all_freeSpaces();
      return;
    }
  }
};

// Writes to NEW mode characteristic
class ModeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue();
    if (!value.length()) return;

    String rxStr(value.c_str());
    rxStr.trim();
    rxStr.toUpperCase();

    // Accept either "IMU"/"DIST" or "MODE:IMU"/"MODE:DIST"
    if (rxStr == "MODE:IMU" || rxStr == "IMU") {
      currentMode = "IMU";
      showStatusOLED();
      notifyMode();
      broadcastModeLoRa();
      return;
    }

    if (rxStr == "MODE:DIST" || rxStr == "DIST") {
      currentMode = "DIST";
      showStatusOLED();
      notifyMode();
      broadcastModeLoRa();
      return;
    }
  }
};

// LoRa RX still handles inc/dec and updates all displays [9]
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  uint16_t n = (size < (BUFFER_SIZE - 1)) ? size : (BUFFER_SIZE - 1);
  memcpy(rxpacket, payload, n);
  rxpacket[n] = '\0';

  if (strcmp(rxpacket, "inc") == 0) {
    freeSpaces++;
    showStatusOLED();
    notifyFreeSpaces();
    update_all_freeSpaces();
  } else if (strcmp(rxpacket, "dec") == 0) {
    freeSpaces--;
    if (freeSpaces < 0) freeSpaces = 0;
    showStatusOLED();
    notifyFreeSpaces();
    update_all_freeSpaces();
  }

  Radio.Sleep();
  lora_idle = true;
}

void setup() {
  Serial.begin(115200);
  VextON();
  delay(100);

  display.init();
  display.clear();
  display.display();

  // LoRa init pattern matches Heltec examples (RxDone + IrqProcess + Rx(0)) [2]
  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);
  RadioEvents.RxDone = OnRxDone;
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);

  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
                    LORA_SPREADING_FACTOR, LORA_CODINGRATE,
                    LORA_PREAMBLE_LENGTH, LORA_FIX_LENGTH_PAYLOAD_ON,
                    true, 0, 0, LORA_IQ_INVERSION_ON, 3000);

  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
                    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
                    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
                    0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  // BLE init and service/characteristics
  BLEDevice::init("ESP32");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Existing FreeSpaces characteristic (read/write/notify) [9]
  pFreeSpacesCharacteristic = pService->createCharacteristic(
    SENSOR_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_INDICATE
  );
  pFreeSpacesCharacteristic->addDescriptor(new BLE2902());
  pFreeSpacesCharacteristic->setValue(String(freeSpaces).c_str());
  pFreeSpacesCharacteristic->setCallbacks(new FreeSpacesCallbacks());

  // NEW Mode characteristic (read/write/notify)
  pModeCharacteristic = pService->createCharacteristic(
    MODE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_INDICATE
  );
  pModeCharacteristic->addDescriptor(new BLE2902());
  pModeCharacteristic->setValue(currentMode.c_str());
  pModeCharacteristic->setCallbacks(new ModeCallbacks());

  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();

  showStatusOLED();
  notifyFreeSpaces();
  notifyMode();
}

void loop() {
  if (lora_idle) {
    Radio.Rx(0);
    lora_idle = false;
  }
  Radio.IrqProcess();

  // connection bookkeeping (same pattern you already use) [9]
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    // push current values immediately after connect
    notifyFreeSpaces();
    notifyMode();
  }
}
