#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <WiFi.h>
#include <esp_now.h>

// ================== MCP EXPANDER ==================
Adafruit_MCP23X17 mcp;

// ================== PINOVI MOTORA ==================
//
// Motor 11 → MCP A0,A1,A2,A3  (SET Plavi)
// Motor 12 → ESP32 32,33,25,26 (GEM Plavi)
// Motor 13 → ESP32 27,14,12,13 (POENI Plavi)
//
// Motor 21 → MCP A4,A5,A6,A7  (SET Crveni)
// Motor 22 → MCP B0,B1,B2,B3  (GEM Crveni)
// Motor 23 → ESP32 4,16,17,5  (POENI Crveni)

// Motor 11 (MCP A0–A3)
const uint8_t M11_IN1 = 0;  // A0
const uint8_t M11_IN2 = 1;  // A1
const uint8_t M11_IN3 = 2;  // A2
const uint8_t M11_IN4 = 3;  // A3

// Motor 12 (ESP32)
const int M12_IN1 = 32;
const int M12_IN2 = 33;
const int M12_IN3 = 25;
const int M12_IN4 = 26;

// Motor 13 (ESP32) – poeni PLAVI
const int M13_IN1 = 27;
const int M13_IN2 = 14;
const int M13_IN3 = 12;
const int M13_IN4 = 13;

// Motor 21 (MCP A4–A7)
const uint8_t M21_IN1 = 4;  // A4
const uint8_t M21_IN2 = 5;  // A5
const uint8_t M21_IN3 = 6;  // A6
const uint8_t M21_IN4 = 7;  // A7

// Motor 22 (MCP B0–B3)
const uint8_t M22_IN1 = 8;   // B0
const uint8_t M22_IN2 = 9;   // B1
const uint8_t M22_IN3 = 10;  // B2
const uint8_t M22_IN4 = 11;  // B3

// Motor 23 (ESP32) – poeni CRVENI
const int M23_IN1 = 4;
const int M23_IN2 = 16;
const int M23_IN3 = 17;
const int M23_IN4 = 5;

// ================== HALL SENZORI ==================
const int HALL1_PIN = 34;
const int HALL2_PIN = 35;
const int HALL3_PIN = 36;
const int HALL4_PIN = 39;
const int HALL5_PIN = 18;
const int HALL6_PIN = 19;

// Većina hall modula ide na LOW kad je magnet blizu.
const int HALL_ACTIVE_STATE = LOW;

// ================== OSTALO ==================
const int BUZZER_PIN = 2;  // Buzzer kao u scoreboard_esp

// ================== KONFIGURACIJA MOTORA ==================

// Half-step sekvenca za 28BYJ-48
const int STEPS = 8;
const uint8_t seq[STEPS][4] = {
  {1, 0, 0, 0},
  {1, 1, 0, 0},
  {0, 1, 0, 0},
  {0, 1, 1, 0},
  {0, 0, 1, 0},
  {0, 0, 1, 1},
  {0, 0, 0, 1},
  {1, 0, 0, 1}
};

// 🔧 BRZINA MOTORA (koristi se samo za izračun delay-a)
float RPM = 25.0;
const int NOMINAL_STEPS_PER_REV = 4096;

int STEP_DELAY_MS = 0;     // izračuna se u setup()
int stepIndex = 0;         // globalni indeks half-step sekvence (0–7)

// 📌 KALIBRIRANI KORACI PO KRUGU ZA SVAKI MOTOR
const long STEPS_PER_REV_11 = 4097;
const long STEPS_PER_REV_12 = 4096;
const long STEPS_PER_REV_13 = 4096;
const long STEPS_PER_REV_21 = 4096;
const long STEPS_PER_REV_22 = 4095;
const long STEPS_PER_REV_23 = 4096;

// 📌 OFFSET DO "NIŠTA" (u stupnjevima) – za init
float OFFSET_TO_BLANK_DEG = 281.0f;

// 📌 KORAK ZA JEDAN DISPLAY KORAK (10° = jedno polje)
const float POINT_STEP_DEG = 10.0f;
const int TOTAL_POSITIONS = 36;

// ================== MAPIRANJE DISKA ==================
//
// 0: prazno
// 1: "0"
// 2: "15"
// 3: "30"
// 4: "40"
// 5: "AD"
// 6..19: "1".."14"
// 20..33: "16".."29"
// 34: "31"
// 35: "32"

const int DISC_INDEX_BLANK = 0;
const int DISC_INDEX_0     = 1;
const int DISC_INDEX_15    = 2;
const int DISC_INDEX_30    = 3;
const int DISC_INDEX_40    = 4;
const int DISC_INDEX_AD    = 5;
const int DISC_INDEX_FIRST_NUM = 6;  // "1"

// ================== TRENUTNI INDEX SVAKOG MOTORA ==================
//
// SVI kreću sa "0" nakon initAllMotors + setInitialTennisLayout,
// dakle index = DISC_INDEX_0.

int idxM11 = DISC_INDEX_0;  // SET Plavi
int idxM21 = DISC_INDEX_0;  // SET Crveni
int idxM12 = DISC_INDEX_0;  // GEM Plavi
int idxM22 = DISC_INDEX_0;  // GEM Crveni
int idxM13 = DISC_INDEX_0;  // POENI Plavi
int idxM23 = DISC_INDEX_0;  // POENI Crveni

// ================== FUNKCIJE ZA MAPIRANJE VRIJEDNOSTI -> INDEX DISKA ==================

int padelPointsToDiscIndex(int padelPoints) {
  // 0,15,30,40
  switch (padelPoints) {
    case 0:  return DISC_INDEX_0;
    case 15: return DISC_INDEX_15;
    case 30: return DISC_INDEX_30;
    case 40: return DISC_INDEX_40;
    default:
      // sigurnosno – ako nešto pođe po zlu, prikaži 0
      return DISC_INDEX_0;
  }
}

// Za gemove, setove i tiebreak: brojke 0,1,2,3,...
int numberToDiscIndex(int n) {
  if (n <= 0) {
    return DISC_INDEX_0; // "0"
  }
  // 1..14
  if (n >= 1 && n <= 14) {
    return DISC_INDEX_FIRST_NUM + (n - 1);  // 1 -> 6, 14 -> 19
  }
  // 15 se u tvojoj sekvenci ne pojavljuje kao "15" (jer je zauzet padel),
  // ali tiebreak realno rijetko ide toliko daleko.
  // Za 16..29:
  if (n >= 16 && n <= 29) {
    // 16 je na indexu 20
    int offsetFrom16 = n - 16;
    return 20 + offsetFrom16; // 16->20, 29->33
  }
  if (n == 31) return 34;
  if (n == 32) return 35;

  // Ako je stvarno nešto iznad, wrapaj na 0 iz sigurnosti
  return DISC_INDEX_0;
}

// ================== HELPER FUNKCIJE ZA MOTORE ==================

// Pretvaranje stupnjeva u broj koraka za KONKRETAN motor
int degreesToSteps(float deg, long stepsPerRev) {
  return (int)((stepsPerRev * deg / 360.0f) + 0.5f);
}

// STEP preko ESP32 pinova
void stepMotorESP(int in1, int in2, int in3, int in4, int idx) {
  digitalWrite(in1, seq[idx][0]);
  digitalWrite(in2, seq[idx][1]);
  digitalWrite(in3, seq[idx][2]);
  digitalWrite(in4, seq[idx][3]);
}

// RELEASE preko ESP32 pinova
void releaseMotorESP(int in1, int in2, int in3, int in4) {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// STEP preko MCP23017
void stepMotorMCP(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4, int idx) {
  mcp.digitalWrite(in1, seq[idx][0]);
  mcp.digitalWrite(in2, seq[idx][1]);
  mcp.digitalWrite(in3, seq[idx][2]);
  mcp.digitalWrite(in4, seq[idx][3]);
}

// RELEASE preko MCP23017
void releaseMotorMCP(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4) {
  mcp.digitalWrite(in1, LOW);
  mcp.digitalWrite(in2, LOW);
  mcp.digitalWrite(in3, LOW);
  mcp.digitalWrite(in4, LOW);
}

// Release SVIH motora (safety)
void releaseAllMotors() {
  // ESP motori
  releaseMotorESP(M12_IN1, M12_IN2, M12_IN3, M12_IN4);
  releaseMotorESP(M13_IN1, M13_IN2, M13_IN3, M13_IN4);
  releaseMotorESP(M23_IN1, M23_IN2, M23_IN3, M23_IN4);

  // MCP motori
  releaseMotorMCP(M11_IN1, M11_IN2, M11_IN3, M11_IN4);
  releaseMotorMCP(M21_IN1, M21_IN2, M21_IN3, M21_IN4);
  releaseMotorMCP(M22_IN1, M22_IN2, M22_IN3, M22_IN4);
}

// ================== ROTACIJA JEDNOG MOTORA ZA ZADANI KUT (DEG) ==================
//
// dir = -1 → tvoj dosadašnji smjer (stepIndex--)
// dir = +1 → suprotni smjer (stepIndex++)

void rotateESPByDegreesDir(int in1, int in2, int in3, int in4,
                           int motorNumber, float deg, long stepsPerRev,
                           int dir) {
  int steps = degreesToSteps(deg, stepsPerRev);

  Serial.print("Motor ");
  Serial.print(motorNumber);
  Serial.print(" (ESP) - rotacija ");
  Serial.print(deg);
  Serial.print("° -> ");
  Serial.print(steps);
  Serial.print(" koraka u smjeru ");
  Serial.println(dir < 0 ? "-1" : "+1");

  for (int i = 0; i < steps; i++) {
    stepMotorESP(in1, in2, in3, in4, stepIndex);

    if (dir < 0) {
      stepIndex--;
      if (stepIndex < 0) stepIndex = STEPS - 1;
    } else {
      stepIndex++;
      if (stepIndex >= STEPS) stepIndex = 0;
    }

    delay(STEP_DELAY_MS);
  }

  releaseMotorESP(in1, in2, in3, in4);
}

// Stari API – default smjer -1 (kao dosad)
void rotateESPByDegrees(int in1, int in2, int in3, int in4,
                        int motorNumber, float deg, long stepsPerRev) {
  rotateESPByDegreesDir(in1, in2, in3, in4, motorNumber, deg, stepsPerRev, -1);
}

// Nova funkcija – smjer +1 (nazad)
void rotateESPByDegreesPlus(int in1, int in2, int in3, int in4,
                            int motorNumber, float deg, long stepsPerRev) {
  rotateESPByDegreesDir(in1, in2, in3, in4, motorNumber, deg, stepsPerRev, +1);
}

void rotateMCPByDegreesDir(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                           int motorNumber, float deg, long stepsPerRev,
                           int dir) {
  int steps = degreesToSteps(deg, stepsPerRev);

  Serial.print("Motor ");
  Serial.print(motorNumber);
  Serial.print(" (MCP) - rotacija ");
  Serial.print(deg);
  Serial.print("° -> ");
  Serial.print(steps);
  Serial.print(" koraka u smjeru ");
  Serial.println(dir < 0 ? "-1" : "+1");

  for (int i = 0; i < steps; i++) {
    stepMotorMCP(in1, in2, in3, in4, stepIndex);

    if (dir < 0) {
      stepIndex--;
      if (stepIndex < 0) stepIndex = STEPS - 1;
    } else {
      stepIndex++;
      if (stepIndex >= STEPS) stepIndex = 0;
    }

    delay(STEP_DELAY_MS);
  }

  releaseMotorMCP(in1, in2, in3, in4);
}

// Stari API – default smjer -1
void rotateMCPByDegrees(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                        int motorNumber, float deg, long stepsPerRev) {
  rotateMCPByDegreesDir(in1, in2, in3, in4, motorNumber, deg, stepsPerRev, -1);
}

// Nova funkcija – smjer +1
void rotateMCPByDegreesPlus(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                            int motorNumber, float deg, long stepsPerRev) {
  rotateMCPByDegreesDir(in1, in2, in3, in4, motorNumber, deg, stepsPerRev, +1);
}

// Jedan "shake" korak = +15° u smjeru -1 pa -5° u smjeru +1
// Neto pomak = 10° u tvom glavnom smjeru (-1)

void shakeStepESP(int in1, int in2, int in3, int in4,
                  int motorNumber, long stepsPerRev) {
  float forwardDeg = 15.0f;
  float backDeg    = 5.0f;

  // naprijed (glavni smjer, -1)
  rotateESPByDegrees(in1, in2, in3, in4,
                     motorNumber, forwardDeg, stepsPerRev);

  // malo nazad (+1)
  rotateESPByDegreesPlus(in1, in2, in3, in4,
                         motorNumber, backDeg, stepsPerRev);
}

void shakeStepMCP(uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                  int motorNumber, long stepsPerRev) {
  float forwardDeg = 15.0f;
  float backDeg    = 5.0f;

  rotateMCPByDegrees(in1, in2, in3, in4,
                     motorNumber, forwardDeg, stepsPerRev);
  rotateMCPByDegreesPlus(in1, in2, in3, in4,
                         motorNumber, backDeg, stepsPerRev);
}

// ================== HOMING FUNKCIJE (EDGE METODA) ==================

bool homeMotorESP(
  int in1, int in2, int in3, int in4,
  int hallPin,
  int motorNumber,
  unsigned long timeoutMs
) {
  unsigned long start = millis();
  bool found = false;

  Serial.println("--------------------------------------");
  Serial.print("Motor ");
  Serial.print(motorNumber);
  Serial.println(" - homing EDGE metoda (ESP)...");

  // 1) FAZA – ako smo već na Hallu, izađi iz zone
  while (millis() - start < timeoutMs &&
         digitalRead(hallPin) == HALL_ACTIVE_STATE) {

    stepMotorESP(in1, in2, in3, in4, stepIndex);
    stepIndex--;
    if (stepIndex < 0) stepIndex = STEPS - 1;
    delay(STEP_DELAY_MS);
  }

  // 2) FAZA – traži prvi ulazak u Hall zonu (NEAKTIVAN -> AKTIVAN)
  while (millis() - start < timeoutMs) {
    int hallVal = digitalRead(hallPin);

    if (hallVal == HALL_ACTIVE_STATE) {
      found = true;
      Serial.print("Motor ");
      Serial.print(motorNumber);
      Serial.println(" - prvi ulazak u Hall zonu (HOME = \"3\")!");
      break;
    }

    stepMotorESP(in1, in2, in3, in4, stepIndex);
    stepIndex--;
    if (stepIndex < 0) stepIndex = STEPS - 1;
    delay(STEP_DELAY_MS);
  }

  releaseMotorESP(in1, in2, in3, in4);

  if (!found) {
    Serial.print("Motor ");
    Serial.print(motorNumber);
    Serial.println(" - TIMEOUT, Hall edge nije nadjen!");
  }

  return found;
}

bool homeMotorMCP(
  uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
  int hallPin,
  int motorNumber,
  unsigned long timeoutMs
) {
  unsigned long start = millis();
  bool found = false;

  Serial.println("--------------------------------------");
  Serial.print("Motor ");
  Serial.print(motorNumber);
  Serial.println(" - homing EDGE metoda (MCP)...");

  while (millis() - start < timeoutMs &&
         digitalRead(hallPin) == HALL_ACTIVE_STATE) {

    stepMotorMCP(in1, in2, in3, in4, stepIndex);
    stepIndex--;
    if (stepIndex < 0) stepIndex = STEPS - 1;
    delay(STEP_DELAY_MS);
  }

  while (millis() - start < timeoutMs) {
    int hallVal = digitalRead(hallPin);

    if (hallVal == HALL_ACTIVE_STATE) {
      found = true;
      Serial.print("Motor ");
      Serial.print(motorNumber);
      Serial.println(" - prvi ulazak u Hall zonu (HOME = \"3\")!");
      break;
    }

    stepMotorMCP(in1, in2, in3, in4, stepIndex);
    stepIndex--;
    if (stepIndex < 0) stepIndex = STEPS - 1;
    delay(STEP_DELAY_MS);
  }

  releaseMotorMCP(in1, in2, in3, in4);

  if (!found) {
    Serial.print("Motor ");
    Serial.print(motorNumber);
    Serial.println(" - TIMEOUT, Hall edge nije nadjen!");
  }

  return found;
}

// ================== TENISKI LAYOUT (SVI NA "0") ==================

void setInitialTennisLayout() {
  Serial.println("Postavljam pocetni teniski layout: SVI na 0 (0:0 gemovi, 0:0 setovi)...");

  // 11 – setovi gornji: "0"
  rotateMCPByDegrees(M11_IN1, M11_IN2, M11_IN3, M11_IN4,
                     11, 10.0f, STEPS_PER_REV_11);
  delay(500);

  // 21 – setovi donji: "0"
  rotateMCPByDegrees(M21_IN1, M21_IN2, M21_IN3, M21_IN4,
                     21, 10.0f, STEPS_PER_REV_21);
  delay(500);

  // 12 – gemovi gornji: "0"
  rotateESPByDegrees(M12_IN1, M12_IN2, M12_IN3, M12_IN4,
                     12, 10.0f, STEPS_PER_REV_12);
  delay(500);

  // 22 – gemovi donji: "0"
  rotateMCPByDegrees(M22_IN1, M22_IN2, M22_IN3, M22_IN4,
                     22, 10.0f, STEPS_PER_REV_22);
  delay(500);

  // 13 – poeni gornji: "0"
  rotateESPByDegrees(M13_IN1, M13_IN2, M13_IN3, M13_IN4,
                     13, 10.0f, STEPS_PER_REV_13);
  delay(500);

  // 23 – poeni donji: "0"
  rotateESPByDegrees(M23_IN1, M23_IN2, M23_IN3, M23_IN4,
                     23, 10.0f, STEPS_PER_REV_23);
  delay(500);

  // logički indexi – svi na "0"
  idxM11 = idxM21 = idxM12 = idxM22 = idxM13 = idxM23 = DISC_INDEX_0;

  Serial.println("Teniski layout postavljen: SVI NA 0.");
}


// Mali "kick" svih motora za 5° u -1 smjeru nakon hominga+offseta
void nudgeAllMotors5deg() {
  float nudgeDeg = 5.0f;

  rotateMCPByDegrees(M11_IN1, M11_IN2, M11_IN3, M11_IN4,
                     11, nudgeDeg, STEPS_PER_REV_11);
  delay(200);

  rotateESPByDegrees(M12_IN1, M12_IN2, M12_IN3, M12_IN4,
                     12, nudgeDeg, STEPS_PER_REV_12);
  delay(200);

  rotateESPByDegrees(M13_IN1, M13_IN2, M13_IN3, M13_IN4,
                     13, nudgeDeg, STEPS_PER_REV_13);
  delay(200);

  rotateMCPByDegrees(M21_IN1, M21_IN2, M21_IN3, M21_IN4,
                     21, nudgeDeg, STEPS_PER_REV_21);
  delay(200);

  rotateMCPByDegrees(M22_IN1, M22_IN2, M22_IN3, M22_IN4,
                     22, nudgeDeg, STEPS_PER_REV_22);
  delay(200);

  rotateESPByDegrees(M23_IN1, M23_IN2, M23_IN3, M23_IN4,
                     23, nudgeDeg, STEPS_PER_REV_23);
  delay(200);
}

// ================== SCOREBOARD LOGIKA (TVOJ scoreboard_esp) ==================

// ------- ESP-NOW KOMANDA -------

typedef struct {
  uint8_t command;   // 1 = RED_POINT, 2 = BLUE_POINT, 3 = UNDO, 4 = FINISH, 5 = RESET
} CommandMessage;

const uint8_t CMD_RED_POINT   = 1;
const uint8_t CMD_BLUE_POINT  = 2;
const uint8_t CMD_UNDO        = 3;
const uint8_t CMD_FINISH      = 4;
const uint8_t CMD_RESET_ALL   = 5;

// ------- STANJE REZULTATA -------

const int MAX_SETS = 3;

int pointsRedIdx  = 0;
int pointsBlueIdx = 0;

int gemoviCrveni[MAX_SETS] = {0, 0, 0};
int gemoviPlavi [MAX_SETS] = {0, 0, 0};

int tiebreakCrveni[MAX_SETS] = {0, 0, 0};
int tiebreakPlavi [MAX_SETS] = {0, 0, 0};

int trenutniSet = 0;
int setsRed  = 0;
int setsBlue = 0;

bool matchFinished   = false;
bool tieBreakActive  = false;

int  lastTieBreakSet       = -1;
bool lastTieBreakWonByRed  = false;

// ------- POVIJEST ZA UNDO -------

const int MAX_HISTORY = 50;

struct ScoreState {
  int pointsRedIdx;
  int pointsBlueIdx;

  int gemoviCrveni[MAX_SETS];
  int gemoviPlavi[MAX_SETS];

  int tiebreakCrveni[MAX_SETS];
  int tiebreakPlavi[MAX_SETS];

  int trenutniSet;
  int setsRed;
  int setsBlue;

  bool matchFinished;
  bool tieBreakActive;

  int  lastTieBreakSet;
  bool lastTieBreakWonByRed;
};

ScoreState historyStack[MAX_HISTORY];
int historyTop = -1;

// PROTOTIPI
void printScore();
void updatePhysicalDisplay();

// ------- POMOĆNE FUNKCIJE SCOREBOARD -------

void saveStateToHistory() {
  if (historyTop >= MAX_HISTORY - 1) {
    for (int i = 1; i < MAX_HISTORY; i++) {
      historyStack[i - 1] = historyStack[i];
    }
    historyTop = MAX_HISTORY - 2;
  }

  historyTop++;
  ScoreState &s = historyStack[historyTop];

  s.pointsRedIdx  = pointsRedIdx;
  s.pointsBlueIdx = pointsBlueIdx;

  for (int i = 0; i < MAX_SETS; i++) {
    s.gemoviCrveni[i]   = gemoviCrveni[i];
    s.gemoviPlavi[i]    = gemoviPlavi[i];
    s.tiebreakCrveni[i] = tiebreakCrveni[i];
    s.tiebreakPlavi[i]  = tiebreakPlavi[i];
  }

  s.trenutniSet        = trenutniSet;
  s.setsRed            = setsRed;
  s.setsBlue           = setsBlue;
  s.matchFinished      = matchFinished;
  s.tieBreakActive     = tieBreakActive;
  s.lastTieBreakSet    = lastTieBreakSet;
  s.lastTieBreakWonByRed = lastTieBreakWonByRed;
}

bool undoLastState() {
  if (historyTop < 0) {
    Serial.println("UNDO: nema povijesti stanja.");
    return false;
  }

  ScoreState &s = historyStack[historyTop];

  pointsRedIdx  = s.pointsRedIdx;
  pointsBlueIdx = s.pointsBlueIdx;

  for (int i = 0; i < MAX_SETS; i++) {
    gemoviCrveni[i]   = s.gemoviCrveni[i];
    gemoviPlavi[i]    = s.gemoviPlavi[i];
    tiebreakCrveni[i] = s.tiebreakCrveni[i];
    tiebreakPlavi[i]  = s.tiebreakPlavi[i];
  }

  trenutniSet        = s.trenutniSet;
  setsRed            = s.setsRed;
  setsBlue           = s.setsBlue;
  matchFinished      = s.matchFinished;
  tieBreakActive     = s.tieBreakActive;
  lastTieBreakSet    = s.lastTieBreakSet;
  lastTieBreakWonByRed = s.lastTieBreakWonByRed;

  historyTop--;
  Serial.println("UNDO: vraćeno jedno stanje unazad.");
  return true;
}

int pointsIndexToPadel(int idx) {
  const int MAP[4] = {0, 15, 30, 40};
  if (idx < 0) idx = 0;
  if (idx > 3) idx = 3;
  return MAP[idx];
}

bool isMatchOver() {
  return (setsRed >= 2 || setsBlue >= 2);
}

void resetPoints() {
  pointsRedIdx  = 0;
  pointsBlueIdx = 0;
}

void resetAll() {
  Serial.println("RESET_ALL: vraćam sve na početno stanje.");

  pointsRedIdx  = 0;
  pointsBlueIdx = 0;

  for (int i = 0; i < MAX_SETS; i++) {
    gemoviCrveni[i]   = 0;
    gemoviPlavi[i]    = 0;
    tiebreakCrveni[i] = 0;
    tiebreakPlavi[i]  = 0;
  }

  trenutniSet = 0;
  setsRed  = 0;
  setsBlue = 0;

  matchFinished  = false;
  tieBreakActive = false;

  lastTieBreakSet      = -1;
  lastTieBreakWonByRed = false;

  historyTop = -1;

  printScore();
  updatePhysicalDisplay();
}

void beepShort() {
  digitalWrite(BUZZER_PIN, HIGH);
  delay(50);
  digitalWrite(BUZZER_PIN, LOW);
}

void beepMulti(int count) {
  for (int i = 0; i < count; i++) {
    beepShort();
    delay(100);
  }
}

void maybeStartTieBreak() {
  if (matchFinished)   return;
  if (tieBreakActive)  return;

  if (gemoviCrveni[trenutniSet] == 6 &&
      gemoviPlavi[trenutniSet]  == 6) {
    tieBreakActive = true;
    Serial.println("Krece tiebrake do 7 poena na 2 razlike.");
    resetPoints();
  }
}

// ------- ZAVRŠETAK SETA -------

void finishSetNormal(bool redWon) {
  if (redWon) {
    setsRed++;
    Serial.print(">>> CRVENI OSVOJILI SET #");
  } else {
    setsBlue++;
    Serial.print(">>> PLAVI OSVOJILI SET #");
  }
  Serial.println(trenutniSet + 1);

  if (isMatchOver()) {
    matchFinished = true;
    Serial.println(redWon ? ">>> MEČ ZAVRŠEN! (CRVENI)"
                          : ">>> MEČ ZAVRŠEN! (PLAVI)");
  } else {
    trenutniSet++;
    if (trenutniSet >= MAX_SETS) {
      matchFinished = true;
      Serial.println(">>> MEČ ZAVRŠEN (MAX SETOVA)!");
    } else {
      resetPoints();
      tieBreakActive = false;
    }
  }

  updatePhysicalDisplay();
}

void finishSetTieBreak(bool redWon) {
  if (redWon) {
    gemoviCrveni[trenutniSet] += 1;
    setsRed++;
  } else {
    gemoviPlavi[trenutniSet]  += 1;
    setsBlue++;
  }

  Serial.print(">>> ");
  Serial.print(redWon ? "CRVENI" : "PLAVI");
  Serial.print(" DOBILI TIEBREAK U SETU #");
  Serial.println(trenutniSet + 1);

  lastTieBreakSet      = trenutniSet;
  lastTieBreakWonByRed = redWon;

  if (isMatchOver()) {
    matchFinished = true;
    Serial.println(redWon ? ">>> MEČ ZAVRŠEN! (CRVENI)"
                          : ">>> MEČ ZAVRŠEN! (PLAVI)");
  } else {
    trenutniSet++;
    if (trenutniSet >= MAX_SETS) {
      matchFinished = true;
      Serial.println(">>> MEČ ZAVRŠEN (MAX SETOVA)!");
    } else {
      resetPoints();
      tieBreakActive = false;
    }
  }

  updatePhysicalDisplay();
}

// ------- LOGIKA POENA -------

void winGameRed() {
  if (matchFinished) return;

  gemoviCrveni[trenutniSet]++;
  Serial.println(">>> CRVENI OSVOJILI GEM!");

  resetPoints();

  int diff = gemoviCrveni[trenutniSet] - gemoviPlavi[trenutniSet];
  if (gemoviCrveni[trenutniSet] >= 6 && diff >= 2) {
    finishSetNormal(true);
  } else {
    maybeStartTieBreak();
    updatePhysicalDisplay();
  }

  printScore();
}

void winGameBlue() {
  if (matchFinished) return;

  gemoviPlavi[trenutniSet]++;
  Serial.println(">>> PLAVI OSVOJILI GEM!");

  resetPoints();

  int diff = gemoviPlavi[trenutniSet] - gemoviCrveni[trenutniSet];
  if (gemoviPlavi[trenutniSet] >= 6 && diff >= 2) {
    finishSetNormal(false);
  } else {
    maybeStartTieBreak();
    updatePhysicalDisplay();
  }

  printScore();
}

void checkTieBreakEnd() {
  int r = tiebreakCrveni[trenutniSet];
  int b = tiebreakPlavi[trenutniSet];

  if ( (r >= 7 || b >= 7) && (abs(r - b) >= 2) ) {
    if (r > b) {
      finishSetTieBreak(true);
    } else {
      finishSetTieBreak(false);
    }
  } else {
    updatePhysicalDisplay();
  }

  printScore();
}

void addPointRed() {
  if (matchFinished) {
    Serial.println("Meč je već završen. Ignoriram poen.");
    return;
  }

  saveStateToHistory();

  if (!tieBreakActive) {
    if (pointsRedIdx < 3) {
      pointsRedIdx++;
      printScore();
      updatePhysicalDisplay();
    } else {
      winGameRed();
    }
  } else {
    tiebreakCrveni[trenutniSet]++;
    Serial.print("Tie-break poen za CRVENE: ");
    Serial.print(tiebreakCrveni[trenutniSet]);
    Serial.print(" : ");
    Serial.println(tiebreakPlavi[trenutniSet]);
    checkTieBreakEnd();
  }
}

void addPointBlue() {
  if (matchFinished) {
    Serial.println("Meč je već završen. Ignoriram poen.");
    return;
  }

  saveStateToHistory();

  if (!tieBreakActive) {
    if (pointsBlueIdx < 3) {
      pointsBlueIdx++;
      printScore();
      updatePhysicalDisplay();
    } else {
      winGameBlue();
    }
  } else {
    tiebreakPlavi[trenutniSet]++;
    Serial.print("Tie-break poen za PLAVE: ");
    Serial.print(tiebreakCrveni[trenutniSet]);
    Serial.print(" : ");
    Serial.println(tiebreakPlavi[trenutniSet]);
    checkTieBreakEnd();
  }
}

// ------- ISPIS SCOREBOARD -------

void printScore() {
  Serial.println(F("================= REZULTAT ================="));
  Serial.println(F(" TIM      POENI   SET-3  SET-2  SET-1   SET"));
  Serial.println(F("-------------------------------------------------"));

  int set_1_red  = gemoviCrveni[0];
  int set_2_red  = gemoviCrveni[1];
  int set_3_red  = gemoviCrveni[2];

  int set_1_blue = gemoviPlavi[0];
  int set_2_blue = gemoviPlavi[1];
  int set_3_blue = gemoviPlavi[2];

  int displayPointsRed  = tieBreakActive ? tiebreakCrveni[trenutniSet]
                                         : pointsIndexToPadel(pointsRedIdx);
  int displayPointsBlue = tieBreakActive ? tiebreakPlavi[trenutniSet]
                                         : pointsIndexToPadel(pointsBlueIdx);

  bool markSet1 = (lastTieBreakSet == 0);
  bool markSet2 = (lastTieBreakSet == 1);
  bool markSet3 = (lastTieBreakSet == 2);

  Serial.print(F(" CRVENI    "));
  Serial.print(displayPointsRed);
  Serial.print(F("       "));

  Serial.print(markSet3 ? "*" : " ");
  Serial.print(set_3_red);
  Serial.print(F("      "));

  Serial.print(markSet2 ? "*" : " ");
  Serial.print(set_2_red);
  Serial.print(F("      "));

  Serial.print(markSet1 ? "*" : " ");
  Serial.print(set_1_red);
  Serial.print(F("      "));

  Serial.println(setsRed);

  Serial.print(F(" PLAVI     "));
  Serial.print(displayPointsBlue);
  Serial.print(F("       "));

  Serial.print(markSet3 ? "*" : " ");
  Serial.print(set_3_blue);
  Serial.print(F("      "));

  Serial.print(markSet2 ? "*" : " ");
  Serial.print(set_2_blue);
  Serial.print(F("      "));

  Serial.print(markSet1 ? "*" : " ");
  Serial.print(set_1_blue);
  Serial.print(F("      "));

  Serial.println(setsBlue);

  Serial.println(F("============================================"));

  if (lastTieBreakSet >= 0) {
    int s = lastTieBreakSet;
    Serial.print(F("* - SET "));
    Serial.print(s + 1);
    Serial.print(F(" - "));
    Serial.print(tiebreakCrveni[s]);
    Serial.print(F(":"));
    Serial.print(tiebreakPlavi[s]);
    Serial.print(F(" tiebrake za "));
    Serial.println(lastTieBreakWonByRed ? F("crvene") : F("plave"));
  }

  Serial.println();
}

// ================== POMAK NA TARGET INDEX ZA MOTORE ==================

void moveESPToIndex(int &currentIdx, int targetIdx,
                    int in1, int in2, int in3, int in4,
                    int motorNumber, long stepsPerRev) {
  if (targetIdx < 0 || targetIdx >= TOTAL_POSITIONS) return;

  int diff = (targetIdx - currentIdx + TOTAL_POSITIONS) % TOTAL_POSITIONS;
  if (diff == 0) return; // već smo tamo

  float deg = diff * POINT_STEP_DEG;
  rotateESPByDegrees(in1, in2, in3, in4, motorNumber, deg, stepsPerRev);
  currentIdx = targetIdx;
}

void moveMCPToIndex(int &currentIdx, int targetIdx,
                    uint8_t in1, uint8_t in2, uint8_t in3, uint8_t in4,
                    int motorNumber, long stepsPerRev) {
  if (targetIdx < 0 || targetIdx >= TOTAL_POSITIONS) return;

  int diff = (targetIdx - currentIdx + TOTAL_POSITIONS) % TOTAL_POSITIONS;
  if (diff == 0) return;

  float deg = diff * POINT_STEP_DEG;
  rotateMCPByDegrees(in1, in2, in3, in4, motorNumber, deg, stepsPerRev);
  currentIdx = targetIdx;
}
// ================== POMAK VIŠE MOTORA ISTOVREMENO ==================
//
// Pomakne sve motore koji trebaju promjenu u jednoj petlji,
// da se maksimalno vrte paralelno (štedi vrijeme).
//
// Redoslijed argumenata je:
//   M11 (set PLAVI), M21 (set CRVENI),
//   M12 (gem PLAVI), M22 (gem CRVENI),
//   M13 (poeni PLAVI), M23 (poeni CRVENI).

void moveAllMotorsToTargets(
  int targetIdxM11, int targetIdxM21,
  int targetIdxM12, int targetIdxM22,
  int targetIdxM13, int targetIdxM23
) {
  auto computeSteps = [&](int currentIdx, int targetIdx, long stepsPerRev) -> long {
    if (targetIdx < 0 || targetIdx >= TOTAL_POSITIONS) return 0;
    int diff = (targetIdx - currentIdx + TOTAL_POSITIONS) % TOTAL_POSITIONS;
    if (diff == 0) return 0;
    float deg = diff * POINT_STEP_DEG;
    return degreesToSteps(deg, stepsPerRev);
  };

  // Koliko koraka treba za svaki motor
  long stepsM11 = computeSteps(idxM11, targetIdxM11, STEPS_PER_REV_11);
  long stepsM21 = computeSteps(idxM21, targetIdxM21, STEPS_PER_REV_21);
  long stepsM12 = computeSteps(idxM12, targetIdxM12, STEPS_PER_REV_12);
  long stepsM22 = computeSteps(idxM22, targetIdxM22, STEPS_PER_REV_22);
  long stepsM13 = computeSteps(idxM13, targetIdxM13, STEPS_PER_REV_13);
  long stepsM23 = computeSteps(idxM23, targetIdxM23, STEPS_PER_REV_23);

  // Nađi maksimalan broj koraka koji itko treba odraditi
  long maxSteps = stepsM11;
  if (stepsM21 > maxSteps) maxSteps = stepsM21;
  if (stepsM12 > maxSteps) maxSteps = stepsM12;
  if (stepsM22 > maxSteps) maxSteps = stepsM22;
  if (stepsM13 > maxSteps) maxSteps = stepsM13;
  if (stepsM23 > maxSteps) maxSteps = stepsM23;

  // Ako nema ništa za vrtiti, samo izađi
  if (maxSteps == 0) return;

  Serial.print("Paralelni pomak motora, maxSteps = ");
  Serial.println(maxSteps);

  for (long i = 0; i < maxSteps; ++i) {
    // SET PLAVI (M11)
    if (i < stepsM11) {
      stepMotorMCP(M11_IN1, M11_IN2, M11_IN3, M11_IN4, stepIndex);
    }
    // SET CRVENI (M21)
    if (i < stepsM21) {
      stepMotorMCP(M21_IN1, M21_IN2, M21_IN3, M21_IN4, stepIndex);
    }
    // GEM PLAVI (M12)
    if (i < stepsM12) {
      stepMotorESP(M12_IN1, M12_IN2, M12_IN3, M12_IN4, stepIndex);
    }
    // GEM CRVENI (M22)
    if (i < stepsM22) {
      stepMotorMCP(M22_IN1, M22_IN2, M22_IN3, M22_IN4, stepIndex);
    }
    // POENI PLAVI (M13)
    if (i < stepsM13) {
      stepMotorESP(M13_IN1, M13_IN2, M13_IN3, M13_IN4, stepIndex);
    }
    // POENI CRVENI (M23)
    if (i < stepsM23) {
      stepMotorESP(M23_IN1, M23_IN2, M23_IN3, M23_IN4, stepIndex);
    }

    // Globalni half-step index – vrtimo u glavnom smjeru (-1)
    stepIndex--;
    if (stepIndex < 0) stepIndex = STEPS - 1;

    delay(STEP_DELAY_MS);
  }

  // Pusti motore koji su se kretali
  if (stepsM11 > 0) releaseMotorMCP(M11_IN1, M11_IN2, M11_IN3, M11_IN4);
  if (stepsM21 > 0) releaseMotorMCP(M21_IN1, M21_IN2, M21_IN3, M21_IN4);
  if (stepsM12 > 0) releaseMotorESP(M12_IN1, M12_IN2, M12_IN3, M12_IN4);
  if (stepsM22 > 0) releaseMotorMCP(M22_IN1, M22_IN2, M22_IN3, M22_IN4);
  if (stepsM13 > 0) releaseMotorESP(M13_IN1, M13_IN2, M13_IN3, M13_IN4);
  if (stepsM23 > 0) releaseMotorESP(M23_IN1, M23_IN2, M23_IN3, M23_IN4);

  // Update-aj logičke indexe
  if (stepsM11 > 0) idxM11 = targetIdxM11;
  if (stepsM21 > 0) idxM21 = targetIdxM21;
  if (stepsM12 > 0) idxM12 = targetIdxM12;
  if (stepsM22 > 0) idxM22 = targetIdxM22;
  if (stepsM13 > 0) idxM13 = targetIdxM13;
  if (stepsM23 > 0) idxM23 = targetIdxM23;
}

// ================== UPDATE FIZIČKOG DISPLAYA IZ SCOREBOARD STANJA ==================

void updatePhysicalDisplay() {
  // 1) POENI (M13 = PLAVI, M23 = CRVENI)
  int displayPointsRed  = tieBreakActive ? tiebreakCrveni[trenutniSet]
                                         : pointsIndexToPadel(pointsRedIdx);
  int displayPointsBlue = tieBreakActive ? tiebreakPlavi[trenutniSet]
                                         : pointsIndexToPadel(pointsBlueIdx);

  int targetIdxRedPoints;
  int targetIdxBluePoints;

  if (tieBreakActive) {
    targetIdxRedPoints  = numberToDiscIndex(displayPointsRed);
    targetIdxBluePoints = numberToDiscIndex(displayPointsBlue);
  } else {
    targetIdxRedPoints  = padelPointsToDiscIndex(displayPointsRed);
    targetIdxBluePoints = padelPointsToDiscIndex(displayPointsBlue);
  }

  // 2) GEMOVI (trenutni set) – M22 = CRVENI, M12 = PLAVI
  int currentGemRed  = gemoviCrveni[trenutniSet];
  int currentGemBlue = gemoviPlavi[trenutniSet];

  int targetIdxRedGem  = numberToDiscIndex(currentGemRed);
  int targetIdxBlueGem = numberToDiscIndex(currentGemBlue);

  // 3) SETOVI – M21 = CRVENI, M11 = PLAVI
  int targetIdxRedSet  = numberToDiscIndex(setsRed);
  int targetIdxBlueSet = numberToDiscIndex(setsBlue);

  // SVE motore vrtimo paralelno:
  //
  //   M11 → targetIdxBlueSet   (set PLAVI)
  //   M21 → targetIdxRedSet    (set CRVENI)
  //   M12 → targetIdxBlueGem   (gem PLAVI)
  //   M22 → targetIdxRedGem    (gem CRVENI)
  //   M13 → targetIdxBluePoints (poeni PLAVI)
  //   M23 → targetIdxRedPoints  (poeni CRVENI)

  moveAllMotorsToTargets(
    targetIdxBlueSet,    // M11
    targetIdxRedSet,     // M21
    targetIdxBlueGem,    // M12
    targetIdxRedGem,     // M22
    targetIdxBluePoints, // M13
    targetIdxRedPoints   // M23
  );
}

// ================== INIT SVIH MOTORA ==================

void initAllMotors() {
  Serial.println("======================================");
  Serial.println("   POCETNA INICIJALIZACIJA MOTORA");
  Serial.println("   (HOME na \"3\" + OFFSET na \"NIŠTA\")");
  Serial.println("======================================");

  unsigned long timeoutPerMotor = 10000; // 10 s po motoru

  // Mali beep na pocetku
  digitalWrite(BUZZER_PIN, HIGH);
  delay(100);
  digitalWrite(BUZZER_PIN, LOW);

  float offsetDeg = OFFSET_TO_BLANK_DEG;

  // 11
  if (homeMotorMCP(M11_IN1, M11_IN2, M11_IN3, M11_IN4,
                   HALL1_PIN, 11, timeoutPerMotor)) {
    delay(1000);
    rotateMCPByDegrees(M11_IN1, M11_IN2, M11_IN3, M11_IN4,
                       11, offsetDeg, STEPS_PER_REV_11);
  }
  delay(500);

  // 12
  if (homeMotorESP(M12_IN1, M12_IN2, M12_IN3, M12_IN4,
                   HALL2_PIN, 12, timeoutPerMotor)) {
    delay(1000);
    rotateESPByDegrees(M12_IN1, M12_IN2, M12_IN3, M12_IN4,
                       12, offsetDeg, STEPS_PER_REV_12);
  }
  delay(500);

  // 13
  if (homeMotorESP(M13_IN1, M13_IN2, M13_IN3, M13_IN4,
                   HALL3_PIN, 13, timeoutPerMotor)) {
    delay(1000);
    rotateESPByDegrees(M13_IN1, M13_IN2, M13_IN3, M13_IN4,
                       13, offsetDeg, STEPS_PER_REV_13);
  }
  delay(500);

  // 21
  if (homeMotorMCP(M21_IN1, M21_IN2, M21_IN3, M21_IN4,
                   HALL4_PIN, 21, timeoutPerMotor)) {
    delay(1000);
    rotateMCPByDegrees(M21_IN1, M21_IN2, M21_IN3, M21_IN4,
                       21, offsetDeg, STEPS_PER_REV_21);
  }
  delay(500);

  // 22
  if (homeMotorMCP(M22_IN1, M22_IN2, M22_IN3, M22_IN4,
                   HALL5_PIN, 22, timeoutPerMotor)) {
    delay(1000);
    rotateMCPByDegrees(M22_IN1, M22_IN2, M22_IN3, M22_IN4,
                       22, offsetDeg, STEPS_PER_REV_22);
  }
  delay(500);

  // 23
  if (homeMotorESP(M23_IN1, M23_IN2, M23_IN3, M23_IN4,
                   HALL6_PIN, 23, timeoutPerMotor)) {
    delay(1000);
    rotateESPByDegrees(M23_IN1, M23_IN2, M23_IN3, M23_IN4,
                       23, offsetDeg, STEPS_PER_REV_23);
  }
  delay(500);

  // Nakon offseta na "NIŠTA", postavi početni layout (trenutno svi na 0)
  setInitialTennisLayout();


  Serial.println("======================================");
  Serial.println("   INICIJALIZACIJA ZAVRSENA");
  Serial.println("======================================");

  // Buzzer pattern
  for (int i = 0; i < 3; i++) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(100);
    digitalWrite(BUZZER_PIN, LOW);
    delay(150);
  }
}

// ================== ESP-NOW RECEIVE ==================

void OnDataRecv(const esp_now_recv_info * info, const uint8_t *incomingData, int len) {
  if (len < sizeof(CommandMessage)) {
    Serial.println("Primljen premali paket.");
    return;
  }

  CommandMessage cmd;
  memcpy(&cmd, incomingData, sizeof(cmd));

  Serial.print("Primljena komanda: ");
  Serial.println(cmd.command);

  bool changed = false;

  switch (cmd.command) {
    case CMD_RED_POINT:
      addPointRed();      // CRVENI -> motori 23,22,21
      changed = true;
      break;

    case CMD_BLUE_POINT:
      addPointBlue();     // PLAVI -> motori 13,12,11
      changed = true;
      break;

    case CMD_UNDO:
      if (undoLastState()) {
        printScore();
        updatePhysicalDisplay();
        changed = true;
      }
      break;

    case CMD_FINISH:
      if (!matchFinished) {
        saveStateToHistory();
        matchFinished = true;
        Serial.println(">>> MEČ MANUALNO ZAKLJUČAN (FINISH)!");
        printScore();
        changed = true;
      } else {
        Serial.println("Meč je već zaključen.");
      }
      break;

    case CMD_RESET_ALL:
      resetAll();
      changed = true;
      break;

    default:
      Serial.println("Nepoznata komanda.");
      break;
  }

  if (changed) {
    if (cmd.command == CMD_FINISH) {
      beepMulti(5);   // kraj meča
    } else if (cmd.command == CMD_RESET_ALL) {
      beepMulti(2);   // full reset
    } else {
      beepShort();    // ostale promjene
    }
  }
}

// ================== SETUP / LOOP ==================

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("Bootam ESP32, inicijaliziram MCP, pinove i ESP-NOW...");

  // Izračun brzine (delay po koraku)
  STEP_DELAY_MS = (int)(60000.0 / (RPM * NOMINAL_STEPS_PER_REV) + 0.5);
  Serial.print("RPM = ");
  Serial.print(RPM);
  Serial.print("  -> STEP_DELAY_MS = ");
  Serial.println(STEP_DELAY_MS);

  // MCP / I2C
  Wire.begin(21, 22);
  Wire.setClock(400000);

  if (!mcp.begin_I2C(0x27)) {
    Serial.println("Nije pronadjen MCP23X17 na adresi 0x27! STOP.");
    while (1) { delay(10); }
  }

  // ESP motori kao izlazi
  pinMode(M12_IN1, OUTPUT);
  pinMode(M12_IN2, OUTPUT);
  pinMode(M12_IN3, OUTPUT);
  pinMode(M12_IN4, OUTPUT);

  pinMode(M13_IN1, OUTPUT);
  pinMode(M13_IN2, OUTPUT);
  pinMode(M13_IN3, OUTPUT);
  pinMode(M13_IN4, OUTPUT);

  pinMode(M23_IN1, OUTPUT);
  pinMode(M23_IN2, OUTPUT);
  pinMode(M23_IN3, OUTPUT);
  pinMode(M23_IN4, OUTPUT);

  // MCP pinovi
  uint8_t mcpPins[] = {
    M11_IN1, M11_IN2, M11_IN3, M11_IN4,
    M21_IN1, M21_IN2, M21_IN3, M21_IN4,
    M22_IN1, M22_IN2, M22_IN3, M22_IN4
  };
  for (uint8_t i = 0; i < sizeof(mcpPins) / sizeof(mcpPins[0]); i++) {
    mcp.pinMode(mcpPins[i], OUTPUT);
    mcp.digitalWrite(mcpPins[i], LOW);
  }

  // Hall senzori
  pinMode(HALL1_PIN, INPUT);
  pinMode(HALL2_PIN, INPUT);
  pinMode(HALL3_PIN, INPUT);
  pinMode(HALL4_PIN, INPUT);
  pinMode(HALL5_PIN, INPUT);
  pinMode(HALL6_PIN, INPUT);

  // Buzzer
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  // Safety
  releaseAllMotors();

  // Scoreboard init (logika)
  resetAll();

  // ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(OnDataRecv);

  Serial.println("Pokrecem homing + offset na \"NIŠTA\" + teniski layout...");
  initAllMotors();         // nakon ovoga svi motori pokazuju "0"

  updatePhysicalDisplay(); // da se sigurno uskladi sa stanjem scorea

  Serial.println("scoreboard + splitflap spreman. Čeka komande...");
}

void loop() {
  // Sve se rješava preko ESP-NOW callbacka
}
