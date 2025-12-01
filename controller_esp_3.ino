// controller_esp_3.ino
// ESP WROOM 3 – kontroler s tipkalima
// Crvena = poen CRVENI
// Plava  = poen PLAVI
// Žuta   = UNDO
// Zelena = FINISH

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>

// ================== MAC ADRESE ==================

// Scoreboard ESP_1 (ESP WROOM 1)
uint8_t scoreboardMac[] = {0x10, 0x06, 0x1C, 0x81, 0xDA, 0xA0};

// ================== KOMANDA STRUKTURA ==================

typedef struct {
  uint8_t command;   // 1 = RED_POINT, 2 = BLUE_POINT, 3 = UNDO, 4 = FINISH
} CommandMessage;

CommandMessage cmdMsg;

const uint8_t CMD_RED_POINT = 1;
const uint8_t CMD_BLUE_POINT = 2;
const uint8_t CMD_UNDO = 3;
const uint8_t CMD_FINISH = 4;
const uint8_t CMD_RESET_ALL = 5;

esp_now_peer_info_t peerInfo;

// ================== TIPKE ==================

const int BTN_GREEN_PIN  = 4;   // finish
const int BTN_RED_PIN    = 5;   // poen za crvene
const int BTN_BLUE_PIN   = 18;  // poen za plave
const int BTN_YELLOW_PIN = 19;  // undo

// Debounce
const unsigned long DEBOUNCE_DELAY = 50;  // ms
const unsigned long LONG_PRESS_MS = 4000; // 4 sekunde za long press


int lastStableGreen  = HIGH;
int lastStableRed    = HIGH;
int lastStableBlue   = HIGH;
int lastStableYellow = HIGH;

int lastReadingGreen  = HIGH;
int lastReadingRed    = HIGH;
int lastReadingBlue   = HIGH;
int lastReadingYellow = HIGH;

unsigned long lastChangeTimeGreen  = 0;
unsigned long lastChangeTimeRed    = 0;
unsigned long lastChangeTimeBlue   = 0;
unsigned long lastChangeTimeYellow = 0;

unsigned long pressStartGreen  = 0;
unsigned long pressStartRed    = 0;
unsigned long pressStartBlue   = 0;
unsigned long pressStartYellow = 0;


// ================== ESP-NOW CALLBACK ==================

void OnDataSent(const wifi_tx_info_t *info, esp_now_send_status_t status) {
  Serial.print("Last Packet Send Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

// ================== SLANJE KOMANDE ==================

void sendCommand(uint8_t command) {
  cmdMsg.command = command;
  esp_err_t result = esp_now_send(scoreboardMac, (uint8_t*)&cmdMsg, sizeof(cmdMsg));
  if (result == ESP_OK) {
    Serial.print("Poslana komanda: ");
    Serial.println(command);
  } else {
    Serial.print("Greska slanja komande: ");
    Serial.println(result);
  }
}

// ================== TIPKE ==================

void handleButton(
  const char* name,
  int pin,
  int &lastReading,
  int &lastStable,
  unsigned long &lastChangeTime,
  unsigned long &pressStartTime,
  void (*onShortRelease)(),
  void (*onLongRelease)()
) {
  int reading = digitalRead(pin);
  unsigned long now = millis();

  // Debounce
  if (reading != lastReading) {
    lastChangeTime = now;
    lastReading = reading;
  }

  if ((now - lastChangeTime) > DEBOUNCE_DELAY) {
    if (reading != lastStable) {

      // INPUT_PULLUP: HIGH = pušteno, LOW = stisnuto

      // Trenutak PRITISKA (HIGH -> LOW)
      if (lastStable == HIGH && reading == LOW) {
        pressStartTime = now;
      }

      // Trenutak PUŠTANJA (LOW -> HIGH)
      if (lastStable == LOW && reading == HIGH) {
        unsigned long pressDuration = now - pressStartTime;

        Serial.print("Tipka PUSTENA: ");
        Serial.print(name);
        Serial.print(" (");
        Serial.print(pressDuration);
        Serial.println(" ms)");

        if (pressDuration >= LONG_PRESS_MS) {
          if (onLongRelease) onLongRelease();
        } else {
          if (onShortRelease) onShortRelease();
        }
      }

      lastStable = reading;
    }
  }
}


// Callback za pojedine tipke
// ZELENA
void onGreenShort() {
  // kratki pritisak zelene = ništa
}

void onGreenLong() {
  // dugi pritisak zelene = FINISH meča
  sendCommand(CMD_FINISH);
}

// CRVENA
void onRedShort() {
  // uvijek kratki klik = poen CRVENI
  sendCommand(CMD_RED_POINT);
}

// PLAVA
void onBlueShort() {
  // uvijek kratki klik = poen PLAVI
  sendCommand(CMD_BLUE_POINT);
}

// ŽUTA
void onYellowShort() {
  // kratki klik = UNDO
  sendCommand(CMD_UNDO);
}

void onYellowLong() {
  // dugi klik = FULL RESET
  sendCommand(CMD_RESET_ALL);
}


// ================== SETUP / LOOP ==================

void setup() {
  Serial.begin(115200);

  pinMode(BTN_GREEN_PIN,  INPUT_PULLUP);
  pinMode(BTN_RED_PIN,    INPUT_PULLUP);
  pinMode(BTN_BLUE_PIN,   INPUT_PULLUP);
  pinMode(BTN_YELLOW_PIN, INPUT_PULLUP);

  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_register_send_cb(OnDataSent);

  memcpy(peerInfo.peer_addr, scoreboardMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  Serial.println("controller_esp_3 spreman. Pritisni tipke...");
}

void loop() {
  handleButton("ZELENA", BTN_GREEN_PIN,
             lastReadingGreen, lastStableGreen, lastChangeTimeGreen, pressStartGreen,
             onGreenShort, onGreenLong);

  handleButton("CRVENA", BTN_RED_PIN,
              lastReadingRed, lastStableRed, lastChangeTimeRed, pressStartRed,
              onRedShort, nullptr);          // nema long akcije

  handleButton("PLAVA", BTN_BLUE_PIN,
              lastReadingBlue, lastStableBlue, lastChangeTimeBlue, pressStartBlue,
              onBlueShort, nullptr);         // nema long akcije

  handleButton("ZUTA", BTN_YELLOW_PIN,
              lastReadingYellow, lastStableYellow, lastChangeTimeYellow, pressStartYellow,
              onYellowShort, onYellowLong);
}
