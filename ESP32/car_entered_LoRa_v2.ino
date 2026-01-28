/*
  car_entered_LoRa.ino
  --------------------
  Sensor node (ENTRY): sends LoRa payload "dec" when a vehicle enters.

  Same mode behavior and mode commands as car_exit_LoRa.ino:
    "MODE:IMU"  -> IMU detection
    "MODE:DIST" -> distance detection
*/

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <NewPing.h>

#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

// ==================== LoRa config ====================
#define RF_FREQUENCY          915000000
#define TX_OUTPUT_POWER       5
#define LORA_BANDWIDTH        0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE       1
#define LORA_PREAMBLE_LENGTH  8
#define LORA_SYMBOL_TIMEOUT   0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON       false

#define RX_BUF_SIZE  64
static char rxBuf[RX_BUF_SIZE];

// ==================== OLED ====================
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED,
                           GEOMETRY_128_64, RST_OLED);

void VextON() {
  pinMode(Vext, OUTPUT);
  digitalWrite(Vext, LOW);
}

void showStatus(const String &line1, const String &line2 = "") {
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  display.drawString(display.getWidth()/2, 16, line1);
  if (line2.length()) display.drawString(display.getWidth()/2, 38, line2);
  display.display();
}

// ==================== Detection mode ====================
enum DetectMode { MODE_IMU, MODE_DIST };
volatile DetectMode mode = MODE_IMU;

// ==================== IMU ====================
Adafruit_MPU6050 mpu;

static const float Z_POS_THRESH = +3.0f;   // m/s^2
static const float Z_NEG_THRESH = -3.0f;   // m/s^2
static const uint32_t IMU_UPDOWN_TIMEOUT_MS = 3000;
static const uint32_t IMU_COOLDOWN_MS = 2000;

enum ImuState { IMU_WAIT_UP, IMU_WAIT_DOWN, IMU_COOLDOWN };
ImuState imuState = IMU_WAIT_UP;
uint32_t imuStateStartMs = 0;
uint32_t imuCooldownStartMs = 0;

// ==================== HC-SR04 ====================
#define TRIGGER_PIN   7
#define ECHO_PIN      6
#define MAX_DISTANCE  300
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

static const float TARGET_DIST_M = 1.0;
static const uint32_t REQUIRED_MS = 1000;
bool prevInRange = false;
uint32_t inRangeStartMs = 0;
bool distTriggered = false;

// ==================== LoRa radio events ====================
static RadioEvents_t RadioEvents;
bool lora_idle = true;

void OnTxDone(void) { lora_idle = true; }
void OnTxTimeout(void) { Radio.Sleep(); lora_idle = true; }

void sendDecPacket() {
  if (!lora_idle) return;

  const char *payload = "dec";
  Radio.Send((uint8_t*)payload, strlen(payload));
  lora_idle = false;

  showStatus("car has entered", "sent dec");
}

void handleModeCommand(const char *cmd) {
  if (strcmp(cmd, "MODE:IMU") == 0) {
    mode = MODE_IMU;
    imuState = IMU_WAIT_UP;
    imuStateStartMs = millis();
    distTriggered = false;
    prevInRange = false;
    showStatus("Mode: IMU", "Waiting for gate");
  } else if (strcmp(cmd, "MODE:DIST") == 0) {
    mode = MODE_DIST;
    imuState = IMU_WAIT_UP;
    imuStateStartMs = millis();
    distTriggered = false;
    prevInRange = false;
    showStatus("Mode: DIST", "Waiting for car");
  }
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  uint16_t n = (size < (RX_BUF_SIZE - 1)) ? size : (RX_BUF_SIZE - 1);
  memcpy(rxBuf, payload, n);
  rxBuf[n] = '\0';

  if (strncmp(rxBuf, "MODE:", 5) == 0) {
    Serial.printf("RX cmd: %s\n", rxBuf);
    handleModeCommand(rxBuf);
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

  showStatus("Booting...", "entry node");

  Wire.begin();
  if (!mpu.begin()) {
    showStatus("MPU6050 FAIL");
    while (1) delay(100);
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Mcu.begin(HELTEC_BOARD, SLOW_CLK_TPYE);

  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

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

  imuStateStartMs = millis();
  showStatus("Mode: IMU", "Waiting for gate");
}

void loop() {
  if (lora_idle) {
    Radio.Rx(0);
    lora_idle = false;
  }
  Radio.IrqProcess();

  const uint32_t now = millis();

  if (mode == MODE_IMU) {
    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);
    float z = a.acceleration.z;

    switch (imuState) {
      case IMU_WAIT_UP:
        if (z > Z_POS_THRESH) {
          imuState = IMU_WAIT_DOWN;
          imuStateStartMs = now;
          showStatus("Gate UP detected", "waiting DOWN");
        }
        break;

      case IMU_WAIT_DOWN:
        if (now - imuStateStartMs > IMU_UPDOWN_TIMEOUT_MS) {
          imuState = IMU_WAIT_UP;
          showStatus("Gate timeout", "waiting UP");
        } else if (z < Z_NEG_THRESH) {
          sendDecPacket();
          imuState = IMU_COOLDOWN;
          imuCooldownStartMs = now;
        }
        break;

      case IMU_COOLDOWN:
        if (now - imuCooldownStartMs > IMU_COOLDOWN_MS) {
          imuState = IMU_WAIT_UP;
          showStatus("Mode: IMU", "Waiting for gate");
        }
        break;
    }

    delay(50);
  } else {
    float dist_cm = sonar.ping_cm();
    float dist_m = dist_cm / 100.0f;
    bool inRange = (dist_cm > 0 && dist_m < TARGET_DIST_M);

    if (inRange && !prevInRange) {
      inRangeStartMs = now;
      distTriggered = false;
    }

    if (inRange && !distTriggered) {
      if (now - inRangeStartMs >= REQUIRED_MS) {
        sendDecPacket();
        distTriggered = true;
      }
    }

    if (!inRange) {
      distTriggered = false;
    }

    prevInRange = inRange;

    if (!inRange) {
      showStatus("Mode: DIST", "Waiting for car");
    }

    delay(50);
  }
}
