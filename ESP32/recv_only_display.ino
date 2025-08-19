#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

#define SERVICE_UUID                "8c055cbe-b517-4134-bb83-a4c4b05944ba"
#define SENSOR_CHARACTERISTIC_UUID  "24b27f5a-8187-467c-81c9-b4480741d580"

#define RF_FREQUENCY                915000000
#define LORA_BANDWIDTH              0
#define LORA_SPREADING_FACTOR       7
#define LORA_CODINGRATE             1
#define LORA_PREAMBLE_LENGTH        8
#define LORA_SYMBOL_TIMEOUT         0
#define LORA_FIX_LENGTH_PAYLOAD_ON  false
#define LORA_IQ_INVERSION_ON        false
#define BUFFER_SIZE                 8

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
char rxpacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;
bool lora_idle = true;

BLEServer* pServer = NULL;
BLECharacteristic* pSensorCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

int freeSpaces = 0;

// =========== LoRa packet sending function =============
void update_all_freeSpaces() {
    // LoRa packet: 0x42e1 prefix (2 bytes), then freeSpaces as int32_t (big endian)
    uint8_t packet[6];
    packet[0] = 0x42;   // prefix
    packet[1] = 0xe1;
    packet[2] = (freeSpaces >> 24) & 0xFF;
    packet[3] = (freeSpaces >> 16) & 0xFF;
    packet[4] = (freeSpaces >> 8) & 0xFF;
    packet[5] = (freeSpaces) & 0xFF;
    Radio.Send(packet, 6);
    Serial.printf("LoRa: Sent update 0x42e1 %d\n", freeSpaces);
}

// ========== OLED + BLE update ===========
void showCounter() {
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  String outStr = "Free Spaces: " + String(freeSpaces);
  display.drawString(display.getWidth()/2, display.getHeight()/2-8, outStr);
  display.display();
  if (deviceConnected && pSensorCharacteristic != nullptr) {
    pSensorCharacteristic->setValue(String(freeSpaces).c_str());
    pSensorCharacteristic->notify();
  }
}

void VextON(void) {
  pinMode(Vext,OUTPUT); digitalWrite(Vext, LOW);
}

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer*)    { deviceConnected = true; }
  void onDisconnect(BLEServer*) { deviceConnected = false; }
};

// Accepts "inc", "dec", or an integer string via BLE writes
class MySensorCharacteristicCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue();
    if (value.length() > 0) {
      String rxStr(value.c_str());
      rxStr.trim();
      if (rxStr.equalsIgnoreCase("inc")) {
        freeSpaces++;
        showCounter();
        update_all_freeSpaces();
      } else if (rxStr.equalsIgnoreCase("dec")) {
        freeSpaces--;
        if (freeSpaces < 0) {
          freeSpaces = 0;
        }
        showCounter();
        update_all_freeSpaces();
      } else if (rxStr.length() > 0 && rxStr.toInt() == rxStr.toFloat()) {
        // Accept only integer, reject non-numeric (`toInt`/`toFloat` trick)
        freeSpaces = rxStr.toInt();
        showCounter();
        update_all_freeSpaces();
      }
    }
  }
};

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memcpy(rxpacket, payload, size); rxpacket[size] = '\0';
  Serial.printf("\r\nreceived \"%s\" rssi %d, len %d\r\n",rxpacket,rssi,size);
  if (strcmp(rxpacket, "inc") == 0)      { freeSpaces++; showCounter(); update_all_freeSpaces();}
  else if (strcmp(rxpacket, "dec") == 0) { 
      freeSpaces--; 
      if (freeSpaces < 0) {
          freeSpaces = 0;
      }
      showCounter(); 
      update_all_freeSpaces();}
  Radio.Sleep();
  lora_idle = true;
}

void setup() {
  Serial.begin(115200);
  VextON(); delay(100);

  display.init(); display.clear(); display.display();
  display.setFont(ArialMT_Plain_16); display.setTextAlignment(TEXT_ALIGN_CENTER);

  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,
    LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);

  BLEDevice::init("ESP32");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);

  pSensorCharacteristic = pService->createCharacteristic(
    SENSOR_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_INDICATE
  );

  pSensorCharacteristic->addDescriptor(new BLE2902());
  pSensorCharacteristic->setValue(String(freeSpaces).c_str());
  pSensorCharacteristic->setCallbacks(new MySensorCharacteristicCallbacks());

  pService->start();

  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);
  BLEDevice::startAdvertising();
  Serial.println("BLE + LoRa receiver ready. Waiting for connections...");

  showCounter();
}

void loop() {
  if (lora_idle) {
    Radio.Rx(0); lora_idle = false;
  }
  Radio.IrqProcess();

  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    Serial.println("Start advertising");
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    Serial.println("Device Connected");
    if (pSensorCharacteristic) {
      pSensorCharacteristic->setValue(String(freeSpaces).c_str());
      pSensorCharacteristic->notify();
    }
  }
}
