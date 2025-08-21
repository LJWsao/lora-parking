#include <NewPing.h>
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

#define TRIGGER_PIN   7
#define ECHO_PIN      6
#define MAX_DISTANCE  300
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE);

#define RF_FREQUENCY          915000000
#define TX_OUTPUT_POWER       5
#define LORA_BANDWIDTH        0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE       1
#define LORA_PREAMBLE_LENGTH  8
#define LORA_SYMBOL_TIMEOUT   0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON       false
#define BUFFER_SIZE           8
char txpacket[BUFFER_SIZE];
bool lora_idle = true;

static RadioEvents_t RadioEvents;
void OnTxDone(void)        { lora_idle = true; }
void OnTxTimeout(void)     { Radio.Sleep(); lora_idle = true; }

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);
void VextON()  { pinMode(Vext,OUTPUT); digitalWrite(Vext, LOW); }

const float TARGET_DIST_M = 1.0;
const uint32_t REQUIRED_MS = 1000;
// State flags
bool prevInRange = false;      // Was an object previously in range
unsigned long inRangeStart = 0;
bool sentEvent = false;        // True if we've already sent dec for current detection
bool showEventMessage = false;
unsigned long eventMessageStart = 0;

void showWaiting() {
  display.clear();
  display.drawString(display.getWidth()/2, display.getHeight()/2-8, "Waiting for car");
  display.display();
}

void showCarExited() {
  display.clear();
  display.drawString(display.getWidth()/2, display.getHeight()/2-8, "car has exited");
  display.display();
}

void setup() {
  Serial.begin(115200);
  VextON(); delay(100);
  display.init(); display.clear(); display.display();
  display.setFont(ArialMT_Plain_16); display.setTextAlignment(TEXT_ALIGN_CENTER);
  showWaiting();

  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
    LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
}

void loop() {
  float dist_cm = sonar.ping_cm();
  float dist_m  = dist_cm / 100.0;
  unsigned long now = millis();

  // Display event message for 3 seconds, then return to waiting
  if (showEventMessage) {
    if (now - eventMessageStart >= 3000) {
      showEventMessage = false;
      showWaiting();
    }
    Radio.IrqProcess();
    delay(50);
    return;
  }

  bool inRange = (dist_cm > 0 && dist_m < TARGET_DIST_M);

  if (inRange && !prevInRange) {
    // Transition: Out of range --> In range
    inRangeStart = now;
    sentEvent = false;
    // Do not send yet; need >1s in this state
  }

  if (inRange) {
    if (!sentEvent && (now - inRangeStart >= REQUIRED_MS) && lora_idle) {
      // Object has stayed in range for required time, event was not yet sent
      strcpy(txpacket, "inc");
      Serial.printf("[HC-SR04] Object for >1s, sending \"%s\"\n", txpacket);
      Radio.Send((uint8_t *)txpacket, strlen(txpacket));
      showCarExited();
      showEventMessage = true;
      eventMessageStart = millis();
      lora_idle = false;
      sentEvent = true;
    }
    // Do nothing else while object in range and event has already been sent
  } else {
    // Not in range: reset state so we can detect next car after leaving
    sentEvent = false;
  }

  prevInRange = inRange;

  Radio.IrqProcess();
  delay(50);
}
