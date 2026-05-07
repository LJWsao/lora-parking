/* 
car exit code 

EXIT sensor node: sends LoRa payload "inc" when a vehicle exits.

*/

#include <NewPing.h>
#include "LoRaWan_APP.h"
#include "Arduino.h"
#include "HT_SSD1306Wire.h"

#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

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
#define BUFFER_SIZE           16
char txpacket[BUFFER_SIZE];
char rxpacket[BUFFER_SIZE];

// ====== Radio definitions =======
static RadioEvents_t RadioEvents;
void OnTxDone(void);
void OnTxTimeout(void);
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr );
// Custom function defs
void switchSensorMode(const char *mode);

typedef enum
{
    LOWPOWER,
    STATE_RX,
    STATE_TX
}radio_states_t;
radio_states_t radio_state;

typedef enum
{
    DIST,
    IMU,
}sensor_states_t;
sensor_states_t sens_state;

// ======= IMU =======
Adafruit_MPU6050 mpu;

static const float RAISED_THRESH = 60; // raised limit in degrees
static const float LOW_THRESH = 10; //lowered limit in degrees
// Callibration constants
static const float ACCEL_X_OFFSET = 0;
static const float ACCEL_Y_OFFSET = 0;
static const float ACCEL_Z_OFFSET = 0;

static const int MPU_SDA = 41;
static const int MPU_SCL = 42;

// Tracking whether sensor has been triggered (either distance or )
bool sens_start = false; 

// Define OLED screen
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

// Define GPIO
void VextON()  { pinMode(Vext,OUTPUT); digitalWrite(Vext, LOW); }

const float TARGET_DIST_M = 0.3;
// State flags
bool showEventMessage = false;
unsigned long eventMessageStart = 0;

void showDistWaiting() {
  display.clear();
  display.drawString(display.getWidth()/2, display.getHeight()/2-8, "DIST: Waiting");
  display.display();
}

void showIMUWaiting() {
  display.clear();
  display.drawString(display.getWidth()/2, display.getHeight()/2-8, "IMU: Waiting");
  display.display();
}

void showCarExit() {
  display.clear();
  display.drawString(display.getWidth()/2, display.getHeight()/2-8, "car has exited");
  display.display();
}

void setup() {
  Serial.begin(115200);
  // GPIO setup
  VextON(); delay(100);
  // OLED setup
  display.init(); display.clear(); display.display();
  display.setFont(ArialMT_Plain_16); display.setTextAlignment(TEXT_ALIGN_CENTER);

  Mcu.begin(HELTEC_BOARD,SLOW_CLK_TPYE);
  RadioEvents.TxDone = OnTxDone;
  RadioEvents.TxTimeout = OnTxTimeout;
  RadioEvents.RxDone = OnRxDone;

  Radio.Init(&RadioEvents);
  Radio.SetChannel(RF_FREQUENCY);
  Radio.SetTxConfig(MODEM_LORA, TX_OUTPUT_POWER, 0, LORA_BANDWIDTH,
    LORA_SPREADING_FACTOR, LORA_CODINGRATE, LORA_PREAMBLE_LENGTH,
    LORA_FIX_LENGTH_PAYLOAD_ON, true, 0, 0, LORA_IQ_INVERSION_ON, 3000);
  Radio.SetRxConfig(MODEM_LORA, LORA_BANDWIDTH, LORA_SPREADING_FACTOR,
    LORA_CODINGRATE, 0, LORA_PREAMBLE_LENGTH,
    LORA_SYMBOL_TIMEOUT, LORA_FIX_LENGTH_PAYLOAD_ON,
    0, true, 0, 0, LORA_IQ_INVERSION_ON, true );

  // Define default state for radio + sensor
  radio_state=STATE_RX;
  sens_state=IMU;
  showIMUWaiting(); // Set OLED display

  // ==== IMU initialization ====
  Wire1.begin(MPU_SDA, MPU_SCL);
  Wire1.setClock(100000);

  // Try to initialize!
  if (!mpu.begin(0x68, &Wire1)) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Radio.Rx(0);
  delay(50);
  radio_state = LOWPOWER;
}

void loop() {
  // current time variable
  unsigned long now = millis();

  // Debug: see current states ever 5 seconds
  static unsigned long lastDebug = 0;
  if (now - lastDebug > 3000) {
    Serial.printf("Radio: %d, Sensor: %d\n", radio_state, sens_state);
    lastDebug = now;
  }

  // FSM for radio state
  switch(radio_state) {
    // Radio methods only ever called from within this switch case
    case STATE_TX: 
      strcpy(txpacket, "inc");
      Serial.printf("Sensor triggered, sending \"%s\"\n", txpacket);
      Radio.Send((uint8_t *)txpacket, strlen(txpacket));
      // Change to LOWPOWER after packet sent - so IrqProcess run (onTxDone called)
      radio_state = LOWPOWER;
      break;
    case STATE_RX: 
      // Change to LOWPOWER after packet sent - so IrqProcess run (onRxDone called)
      Serial.println("RX mode");
      Radio.Rx(0);
      radio_state = LOWPOWER;
      break;
    case LOWPOWER:
      // Check for interrupts
      break;
    default:
      break;
  }

  Radio.IrqProcess();
  

  // Sensor FSM - distance sensor vs. IMU
  switch(sens_state) {
    case DIST: { // Add this for local scope
      // Check the distance sensor
      float dist_cm = sonar.ping_cm();

      // Ignore 0 readings, use last valid reading (for some reason every other reading wants to be 0)
      static float lastValidDist = 999.0;
      if (dist_cm > 0) {
        lastValidDist = dist_cm;
      }
      float dist_m = lastValidDist / 100.0;
      //Serial.printf("Distance (m): %f\n", dist_m);

      // Check if the distance sensor measures below the threshold
      if (!sens_start && (dist_m < TARGET_DIST_M)) {
        sens_start = true;
      } // Only send if the distance sensor has been trigered and the car has then moved out of the way
      else if (sens_start && (dist_m > TARGET_DIST_M)) {
        sens_start = false;
        showEventMessage = true;
        eventMessageStart = now;
        showCarExit();
        // Packet will now be sent on the next loop by setting state to STATE_TX
        radio_state = STATE_TX;
      }
      
      // Display event message for 2 seconds, then return to waiting
      if (showEventMessage) {
        if (now - eventMessageStart >= 2000) {
          showEventMessage = false;
          showDistWaiting();
        }
      }
      break;
    } // case DIST closing brace
    case IMU: {
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);
      float ax = a.acceleration.x + ACCEL_X_OFFSET;
      float ay = a.acceleration.y + ACCEL_Y_OFFSET;
      float az = a.acceleration.z + ACCEL_Z_OFFSET;
      float roll_ang = atan(ay/sqrt(ax*ax + az*az))*(180/3.14159);
      float pitch_ang = atan(-ax/sqrt(ay*ay + az*az))*(180/3.14159);
      //Serial.printf("Roll angle deg: %f Pitch angle deg: %f\n", roll_ang, pitch_ang);

      // Check if IMU goes above a certain angle
      if (!sens_start && (roll_ang > RAISED_THRESH)) {
        sens_start = true;
      } // Check if IMU goes below a the threshold angle only after it was seen to go above
      else if (sens_start && (roll_ang < LOW_THRESH)) {
        sens_start = false;
        showEventMessage = true;
        eventMessageStart = now;
        showCarExit();
        // Packet will now be sent on the next loop by setting state to STATE_TX
        radio_state = STATE_TX;
      }

      // Handle message display INSIDE IMU case
      if (showEventMessage && (now - eventMessageStart >= 2000)) {
        showEventMessage = false;
        showIMUWaiting();  // ← Show IMU waiting, not DIST!
      }
      break;
    }
    default:
      break;
  }

}

void switchSensorMode(const char *mode) {
  if (strcmp(mode, "MODE:IMU") == 0) {
    sens_state = IMU;
    sens_start = false; // Reset shared mode flag
    showIMUWaiting();
  } 
  else if (strcmp(mode, "MODE:DIST") == 0) {
    sens_state = DIST;
    sens_start = false; // Reset shared mode flag
    showDistWaiting();
  }
}

// called by Radio.Irqprocess
void OnTxDone(void) { 
  Serial.println("onTXDone");
  radio_state = STATE_RX; // Set back to default mode of RX after packet sent
}

void OnTxTimeout(void) {
  Radio.Sleep();
  Serial.println("TX Timeout");
  radio_state = STATE_TX;
}
// called by Radio.Irqprocess
void OnRxDone(uint8_t *payload, uint16_t size, int16_t rssi, int8_t snr ) {
  // Copy to buffer and null-terminate (safe for strcmp)
  uint16_t n = (size < (BUFFER_SIZE - 1)) ? size : (BUFFER_SIZE - 1);
  memcpy(rxpacket, payload, n);
  rxpacket[n] = '\0';

  Serial.printf(">>> OnRxDone raw: size=%d RSSI=%d\n", size, rssi);  // Add this

  // Filter out freeSpaces update packets - (0x42E1 prefix)
  if (n >= 2 && (uint8_t)rxpacket[0] == 0x42 && (uint8_t)rxpacket[1] == 0xE1) {
    Serial.println("Ignoring freeSpaces packet");
    radio_state = STATE_RX;
    return;
  }

  // Check if a switch mode command is received
  if (strncmp(rxpacket, "MODE:", 5) == 0) {
    Serial.printf("RX cmd: %s\n", rxpacket);
    // If so switch the sensor state
    switchSensorMode(rxpacket);
  }
  //Recommended to remove this, still need to test...
  //Radio.Sleep();
  radio_state = STATE_RX;
}