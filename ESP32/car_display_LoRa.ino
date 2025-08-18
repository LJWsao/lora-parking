#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

#define RF_FREQUENCY          915000000
#define LORA_BANDWIDTH        0
#define LORA_SPREADING_FACTOR 7
#define LORA_CODINGRATE       1
#define LORA_PREAMBLE_LENGTH  8
#define LORA_SYMBOL_TIMEOUT   0
#define LORA_FIX_LENGTH_PAYLOAD_ON false
#define LORA_IQ_INVERSION_ON       false
#define BUFFER_SIZE        8

static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

char rxpacket[BUFFER_SIZE];
static RadioEvents_t RadioEvents;
bool lora_idle = true;

int freeSpaces = 0;

void VextON(void) {
  pinMode(Vext,OUTPUT); digitalWrite(Vext, LOW);
}

void showCounter() {
  display.clear();
  display.setFont(ArialMT_Plain_16);
  display.setTextAlignment(TEXT_ALIGN_CENTER);
  String outStr = "Free Spaces: " + String(freeSpaces);
  display.drawString(display.getWidth()/2, display.getHeight()/2-8, outStr);
  display.display();
}

void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr) {
  memcpy(rxpacket, payload, size); rxpacket[size] = '\0';
  Serial.printf("\r\nreceived \"%s\" rssi %d, len %d\r\n",rxpacket,rssi,size);

  if (strcmp(rxpacket, "inc") == 0)      { freeSpaces++; showCounter(); }
  else if (strcmp(rxpacket, "dec") == 0) { freeSpaces--; showCounter(); }
  Radio.Sleep();
  lora_idle = true;
}

void setup() {
  Serial.begin(115200);
  VextON(); delay(100);
  display.init(); display.clear(); display.display();
  display.setFont(ArialMT_Plain_16); display.setTextAlignment(TEXT_ALIGN_CENTER);
  showCounter();

  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  RadioEvents.RxDone = OnRxDone;
  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH, LORA_SYMBOL_TIMEOUT,
    LORA_FIX_LENGTH_PAYLOAD_ON, 0, true, 0, 0, LORA_IQ_INVERSION_ON, true);
}

void loop() {
  if (lora_idle) {
    Serial.println("into RX mode");
    Radio.Rx(0); lora_idle = false;
  }
  Radio.IrqProcess();
}
