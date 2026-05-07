#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

#define SERVICE_UUID                "8c055cbe-b517-4134-bb83-a4c4b05944ba"
#define SENSOR_CHARACTERISTIC_UUID  "24b27f5a-8187-467c-81c9-b4480741d580"
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
#define BUFFER_SIZE                 32

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// ===== Radio FSM (matches v5_enter pattern) =====
static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr);

typedef enum {
  LOWPOWER,
  STATE_RX,
  STATE_TX_FREESPACES,  // TX state for sending freeSpaces update
  STATE_TX_MODE         // TX state for sending mode broadcast
} radio_states_t;
radio_states_t radio_state;

char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

// BLE globals
BLEServer* pServer = NULL;
BLECharacteristic* pFreeSpacesCharacteristic = NULL;
BLECharacteristic* pModeCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;

int freeSpaces = 0;
String currentMode = "IMU";

// Flags set by BLE callbacks, acted on in loop() FSM
bool pendingFreeSpacesUpdate = false;
bool pendingModeUpdate = false;

// 5 Minute rebroadcasting mode in case a sensor node is rebooted.
unsigned long lastModeBroadcast = 0;
const unsigned long MODE_BROADCAST_INTERVAL_MS = 5UL * 60UL * 1000UL; // 5 minutes

void VextON(void) {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void showStatusOLED() {
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(display.getWidth()/2, 16, "Free: " + String(freeSpaces));
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

// ===== BLE Callbacks =====
class MyServerCallbacks : public BLEServerCallbacks {
  void onConnect(BLEServer*) override    { deviceConnected = true; }
  void onDisconnect(BLEServer*) override { deviceConnected = false; }
};

class FreeSpacesCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue();
    if (!value.length()) return;

    String rxStr(value.c_str());
    rxStr.trim();

    if (rxStr.equalsIgnoreCase("inc")) {
      freeSpaces++;
    } else if (rxStr.equalsIgnoreCase("dec")) {
      freeSpaces--;
      if (freeSpaces < 0) freeSpaces = 0;
    } else if (rxStr.length() > 0 && rxStr.toInt() == rxStr.toFloat()) {
      freeSpaces = rxStr.toInt();
      if (freeSpaces < 0) freeSpaces = 0;
    } else {
      return;
    }

    showStatusOLED();
    notifyFreeSpaces();
    // Set flag - FSM will handle TX in loop()
    pendingFreeSpacesUpdate = true;
  }
};

class ModeCallbacks : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* pCharacteristic) override {
    String value = pCharacteristic->getValue();
    if (!value.length()) return;

    String rxStr(value.c_str());
    rxStr.trim();
    rxStr.toUpperCase();

    if (rxStr == "MODE:IMU" || rxStr == "IMU") {
      currentMode = "IMU";
    } else if (rxStr == "MODE:DIST" || rxStr == "DIST") {
      currentMode = "DIST";
    } else {
      return;
    }

    showStatusOLED();
    notifyMode();
    // Set flag - FSM will handle TX in loop()
    pendingModeUpdate = true;
  }
};

// ===== Radio Callbacks =====
void OnTxDone(void) {
  Serial.println("TX Done");
  radio_state = STATE_RX;  // Always return to RX after TX
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("TX Timeout");
  radio_state = STATE_RX;  // Return to RX on timeout too
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  uint16_t n = (size < (BUFFER_SIZE - 1)) ? size : (BUFFER_SIZE - 1);
  memcpy(rxpacket, payload, n);
  rxpacket[n] = '\0';

  Serial.printf("RX: \"%s\" RSSI:%d\n", rxpacket, rssi);

  // Ignore reflected MODE packets (sent by this program)
  if (strncmp(rxpacket, "MODE:", 5) == 0) {
    Serial.println("Ignoring reflected MODE packet");
    Radio.Sleep();
    radio_state = STATE_RX;
    return;
  }

  // Ignore reflected freeSpaces update packets (0x42E1 prefix, sent by us)
  if (n >= 2 && (uint8_t)rxpacket[0] == 0x42 && (uint8_t)rxpacket[1] == 0xE1) {
    Serial.println("Ignoring reflected freeSpaces packet");
    Radio.Sleep();
    radio_state = STATE_RX;
    return;
  }

  // Handle inc/dec from sensor nodes
  if (strcmp(rxpacket, "inc") == 0) {
    freeSpaces++;
    Serial.printf("inc received, freeSpaces=%d\n", freeSpaces);
    showStatusOLED();
    notifyFreeSpaces();
    pendingFreeSpacesUpdate = true;  // FSM will send LoRa update
  } else if (strcmp(rxpacket, "dec") == 0) {
    freeSpaces--;
    if (freeSpaces < 0) freeSpaces = 0;
    Serial.printf("dec received, freeSpaces=%d\n", freeSpaces);
    showStatusOLED();
    notifyFreeSpaces();
    pendingFreeSpacesUpdate = true;  // FSM will send LoRa update
  }

  Radio.Sleep();
  radio_state = STATE_RX;
}

void setup() {
  Serial.begin(115200);
  VextON();
  delay(100);

  display.init();
  display.clear();
  display.display();

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.TxDone    = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone    = OnRxDone;

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

  // Start in RX mode (matches v5_enter pattern)
  radio_state = STATE_RX;

  // BLE init
  BLEDevice::init("ESP32");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);

  pFreeSpacesCharacteristic = pService->createCharacteristic(
    SENSOR_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
    BLECharacteristic::PROPERTY_NOTIFY |
    BLECharacteristic::PROPERTY_INDICATE
  );
  pFreeSpacesCharacteristic->addDescriptor(new BLE2902());
  pFreeSpacesCharacteristic->setValue(String(freeSpaces).c_str());
  pFreeSpacesCharacteristic->setCallbacks(new FreeSpacesCallbacks());

  pModeCharacteristic = pService->createCharacteristic(
    MODE_CHARACTERISTIC_UUID,
    BLECharacteristic::PROPERTY_READ   |
    BLECharacteristic::PROPERTY_WRITE  |
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
  // Check for pending TX requests from BLE callbacks
  // MODE update takes priority over freeSpaces update
  if (radio_state == STATE_RX) {
    if (pendingModeUpdate) {
      pendingModeUpdate = false;
      radio_state = STATE_TX_MODE;
    } else if (pendingFreeSpacesUpdate) {
      pendingFreeSpacesUpdate = false;
      radio_state = STATE_TX_FREESPACES;
    }
  }

  // Radio FSM - matches v5_enter pattern
  switch(radio_state) {
    case STATE_TX_FREESPACES: {
      uint8_t packet[6];
      packet[0] = 0x42;
      packet[1] = 0xE1;
      packet[2] = (freeSpaces >> 24) & 0xFF;
      packet[3] = (freeSpaces >> 16) & 0xFF;
      packet[4] = (freeSpaces >> 8)  & 0xFF;
      packet[5] = (freeSpaces)       & 0xFF;
      Serial.printf("TX freeSpaces update: %d\n", freeSpaces);
      Radio.Send(packet, 6);
      radio_state = LOWPOWER;  // Wait for OnTxDone
      break;
    }
    case STATE_TX_MODE: {
      String msg = "MODE:" + currentMode;
      memcpy(txpacket, msg.c_str(), msg.length());
      Serial.printf("TX mode broadcast: %s\n", txpacket);
      Radio.Send((uint8_t*)txpacket, msg.length());
      radio_state = LOWPOWER;  // Wait for OnTxDone
      // Prevents spamming on manual switches 
      lastModeBroadcast = millis();
      break;
    }
    case STATE_RX:
      Serial.println("Entering RX mode");
      Radio.Rx(0);
      radio_state = LOWPOWER;
      break;
    case LOWPOWER:
      // AMAZING BUG FIX: YOU NEED THIS to effectively flag poll
      // this is because BLE callbacks raise a flag and don't actually 
      // directly manipulate the radio state machine.
      // You basically need a flag polling step that runs in every idle state 
      if (pendingModeUpdate) {
        pendingModeUpdate = false;
        Radio.Standby();
        radio_state = STATE_TX_MODE;
      } 
      else if (pendingFreeSpacesUpdate) {
          pendingFreeSpacesUpdate = false;
          Radio.Standby();
          radio_state = STATE_TX_FREESPACES;
      }
      // Periodic rebroadcast every ~ 5 minutes
      else if (millis() - lastModeBroadcast > MODE_BROADCAST_INTERVAL_MS) {
        Radio.Standby();
        radio_state = STATE_TX_MODE;
        lastModeBroadcast = millis();
      }
      Radio.IrqProcess();
      break;
    default:
      break;
  }

  // BLE connection bookkeeping
  if (!deviceConnected && oldDeviceConnected) {
    delay(500);
    pServer->startAdvertising();
    oldDeviceConnected = deviceConnected;
  }
  if (deviceConnected && !oldDeviceConnected) {
    oldDeviceConnected = deviceConnected;
    notifyFreeSpaces();
    notifyMode();
  }
}