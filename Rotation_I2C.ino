// =====================================================================
//  Rotationsentwicklung – Steuerung (Arduino Uno)
//  Hardware wie bisher: BTS7960, KY-040 (Drehdrück-Encoder), LCD 1602 parallel
//
//  Bedienung:
//   - Nach dem Einschalten: Drehen = Entwicklungszeit (Standard 10:00, 10-s-Schritte)
//   - Kurz drücken          = Start / Stopp (jederzeit)
//   - Gestoppt (PAUSE):     kurz drücken = weiterlaufen, drehen = zurück zur Zeiteinstellung
//   - Länger als 2 s halten = Menü Geschwindigkeit (90..110 %), auch während des Laufs
//                             im Menü: drehen = Geschwindigkeit, kurz drücken = Menü verlassen
//
//  Rotation: ca. 65 U/min am Tank (Übersetzung Rolle Ø35 -> Tank Ø95 = 2,71 -> ca. 176 U/min Motor), Richtungswechsel alle 3,5 s (≈17 Wechsel/min),
//            sanfte Rampen beim Anlaufen und Abbremsen, kurze Pause beim Wechsel.
// =====================================================================

#include <Wire.h>
#include <LiquidCrystal_I2C.h>   // Bibliothek "LiquidCrystal I2C" von Frank de Brabander (Bibliotheksverwalter)

// ================= PINS (aus dem bisherigen Skript) =================
const uint8_t RPWM  = 9;     // BTS7960 RPWM  -> Richtung VOR
const uint8_t LPWM  = 10;    // BTS7960 LPWM  -> Richtung ZURÜCK
const uint8_t REN   = 8;     // BTS7960 R_EN
const uint8_t LEN   = 7;     // BTS7960 L_EN

const uint8_t BTN   = 2;     // KY-040 SW
const uint8_t ENC_A = 3;     // KY-040 CLK (Interrupt)
const uint8_t ENC_B = 4;     // KY-040 DT

// LCD1602 mit I2C-Adapter (PCF8574): SDA -> A4, SCL -> A5, VCC -> 5V, GND -> GND
// Adresse meist 0x27, bei manchen Adaptern 0x3F (PCF8574A) -> hier anpassen
const uint8_t LCD_ADDR = 0x27;
LiquidCrystal_I2C lcd(LCD_ADDR, 16, 2);

// ================= ÜBERSETZUNG & KALIBRIERUNG =================
// Antrieb: Motor treibt direkt die Welle mit den TPU-Rollen (Ø35), die Rollen treiben den Tank (Ø95 an der Lauffläche).
//   Übersetzung i = D_TANK / D_ROLLE = 95 / 35 = 2,71  ->  Rolle (= Motor) muss 2,71x schneller drehen als der Tank.
//   65 U/min am Tank  ->  ca. 176 U/min am Motor (200er-Motor: max. ca. 74 U/min am Tank).
const float D_ROLLE_MM        = 35.0;   // Außen-Ø TPU-Ring auf der Antriebsscheibe
const float D_TANK_MM         = 95.0;   // Ø1520 an der Lauffläche
const float TANK_RPM_SOLL     = 65.0;   // Ziel bei 100 % (200er-Motor: max. ca. 74 U/min am Tank, Reserve für +10 %)
const float MOTOR_RPM_12V     = 200;    // Leerlaufdrehzahl GA37 bei 12 V laut Angebot (0 = unbekannt)
const int   BASE_PWM_FALLBACK = 40;     // gilt, solange MOTOR_RPM_12V = 0 ist (Wert aus altem Skript)
const float PWM_KORREKTUR     = 1.00;   // Feinabgleich nach Messung: gemessene Tank-U/min zu niedrig -> größer

const float UEBERSETZUNG  = D_TANK_MM / D_ROLLE_MM;
const float MOTOR_RPM_SOLL= TANK_RPM_SOLL * UEBERSETZUNG;
// Näherung: Drehzahl ~ proportional zur PWM (Last + Reibung -> PWM_KORREKTUR nach Messung anpassen)
const int   BASE_PWM = (MOTOR_RPM_12V > 0)
                       ? (int)(255.0 * MOTOR_RPM_SOLL / MOTOR_RPM_12V * PWM_KORREKTUR + 0.5)
                       : (int)(BASE_PWM_FALLBACK * PWM_KORREKTUR + 0.5);
const int   START_PWM_MIN     = 22;    // Mindest-PWM beim Anlauf ("Wumms", damit das Getriebe sicher losläuft)
const int   COUNTS_PER_DETENT = 2;     // Encoder-Impulse pro Raste (KY-040 meist 2; bei 1 Raste = 2 Schritte -> anpassen)

// ================= PARAMETER =================
const uint16_t DEFAULT_TIME_S = 600;   // 10:00
const uint16_t TIME_STEP_S    = 10;
const uint16_t MIN_TIME_S     = 30;
const uint16_t MAX_TIME_S     = 3600;

const int SPEED_MIN_PCT = 90;
const int SPEED_MAX_PCT = 110;

// Materialschonend: weniger Richtungswechsel (~12/min, ~3,9 Tank-Umdrehungen je Richtung bei 100 %),
// weiche Rampen und Stillstandspause vor dem Umkehren (Getriebespiel setzt sich, kein Umkehren unter Last).
const unsigned long DIR_INTERVAL_MS = 4500;  // Laufzeit je Richtung inkl. Rampen
const unsigned long RAMP_UP_MS      = 1000;
const unsigned long RAMP_DOWN_MS    = 800;
const unsigned long REVERSE_PAUSE_MS= 300;

const unsigned long LONG_PRESS_MS   = 2000;
const unsigned long DEBOUNCE_MS     = 30;

// ================= ZUSTÄNDE =================
enum Mode  { SET_TIME, RUNNING, PAUSED, FINISHED };
enum Motor { M_OFF, M_RAMP_UP, M_HOLD, M_RAMP_DOWN, M_PAUSE };

Mode  mode       = SET_TIME;
Motor motorState = M_OFF;
bool  speedMenu  = false;

uint16_t setTimeS    = DEFAULT_TIME_S;
long     remainingMs = 0;            // Restzeit in ms
int      speedPct    = 100;

bool directionForward = true;
int  currentPWM       = 0;
unsigned long phaseStart   = 0;      // Beginn der aktuellen Motorphase
unsigned long dirStart     = 0;      // Beginn der aktuellen Richtung
unsigned long lastTick     = 0;      // für Countdown
int  rampFromPWM = 0;
bool stopAfterRamp = false;          // nach dem Abbremsen stehen bleiben (Stopp / Ende)

// ================= ENCODER =================
volatile int encCount = 0;
volatile unsigned long encLastUs = 0;

void isrEncoder() {
  unsigned long t = micros();
  if (t - encLastUs < 800) return;   // Prellen unterdrücken
  encLastUs = t;
  if (digitalRead(ENC_A) == digitalRead(ENC_B)) encCount++;
  else encCount--;
}

int readEncoderSteps() {
  noInterrupts();
  int c = encCount;
  int steps = c / COUNTS_PER_DETENT;
  encCount = c - steps * COUNTS_PER_DETENT;
  interrupts();
  return steps;
}

// ================= MOTOR =================
int targetPWM() {
  return min(255L, (long)BASE_PWM * speedPct / 100);
}

void writeMotor(int pwm) {
  pwm = constrain(pwm, 0, 255);
  currentPWM = pwm;
  if (directionForward) { analogWrite(LPWM, 0); analogWrite(RPWM, pwm); }
  else                  { analogWrite(RPWM, 0); analogWrite(LPWM, pwm); }
}

void motorOff() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  currentPWM = 0;
  motorState = M_OFF;
}

void beginRampUp(unsigned long now) {
  motorState = M_RAMP_UP;
  phaseStart = now;
  dirStart   = now;
}

void beginRampDown(unsigned long now, bool stopAfter) {
  if (motorState == M_OFF) return;
  rampFromPWM   = currentPWM;
  stopAfterRamp = stopAfter;
  motorState    = M_RAMP_DOWN;
  phaseStart    = now;
}

void updateMotor(unsigned long now) {
  unsigned long t = now - phaseStart;
  int tgt = targetPWM();
  int startPWM = max(START_PWM_MIN, (int)(tgt * 6 / 10));
  if (startPWM > tgt) startPWM = tgt;

  switch (motorState) {
    case M_OFF:
      break;

    case M_RAMP_UP:
      if (t >= RAMP_UP_MS) { writeMotor(tgt); motorState = M_HOLD; }
      else writeMotor(startPWM + (long)(tgt - startPWM) * t / RAMP_UP_MS);
      break;

    case M_HOLD:
      if (currentPWM != tgt) writeMotor(tgt);            // Geschwindigkeit live übernehmen
      if (now - dirStart >= DIR_INTERVAL_MS - RAMP_DOWN_MS) beginRampDown(now, false);
      break;

    case M_RAMP_DOWN:
      if (t >= RAMP_DOWN_MS) {
        motorOff();
        if (!stopAfterRamp) { motorState = M_PAUSE; phaseStart = now; }
      } else {
        writeMotor(rampFromPWM - (long)rampFromPWM * t / RAMP_DOWN_MS);
      }
      break;

    case M_PAUSE:
      if (t >= REVERSE_PAUSE_MS) {
        directionForward = !directionForward;
        beginRampUp(now);
      }
      break;
  }
}

// ================= ABLAUF =================
void startRun() {
  unsigned long now = millis();
  if (mode != PAUSED) {               // neuer Durchgang
    remainingMs = (long)setTimeS * 1000L;
    directionForward = true;
  }
  mode = RUNNING;
  lastTick = now;
  beginRampUp(now);
  updateLCD(true);
}

void stopRun() {
  mode = PAUSED;
  beginRampDown(millis(), true);
  updateLCD(true);
}

void finishRun() {
  mode = FINISHED;
  remainingMs = 0;
  beginRampDown(millis(), true);
  updateLCD(true);
}

// ================= TASTER =================
bool btnStable = HIGH, btnLastRead = HIGH;
unsigned long btnChangeT = 0, btnPressT = 0;
bool longHandled = false;

void handleButton(unsigned long now) {
  bool r = digitalRead(BTN);
  if (r != btnLastRead) { btnLastRead = r; btnChangeT = now; }
  if (now - btnChangeT < DEBOUNCE_MS) return;

  if (r != btnStable) {
    btnStable = r;
    if (btnStable == LOW) {                 // gedrückt
      btnPressT = now;
      longHandled = false;
    } else if (!longHandled) {              // kurz losgelassen
      onShortPress();
    }
  }
  // langer Druck: sofort nach 2 s auslösen, nicht erst beim Loslassen
  if (btnStable == LOW && !longHandled && now - btnPressT >= LONG_PRESS_MS) {
    longHandled = true;
    speedMenu = true;
    updateLCD(true);
  }
}

void onShortPress() {
  if (speedMenu) { speedMenu = false; updateLCD(true); return; }
  switch (mode) {
    case SET_TIME: startRun(); break;
    case RUNNING:  stopRun();  break;
    case PAUSED:   startRun(); break;
    case FINISHED: mode = SET_TIME; updateLCD(true); break;
  }
}

// ================= LCD =================
void printTime(long ms) {
  long s = (ms + 999) / 1000;          // aufrunden, damit 0:00 erst am Ende erscheint
  int m = s / 60, sec = s % 60;
  if (m < 10) lcd.print('0');
  lcd.print(m);
  lcd.print(':');
  if (sec < 10) lcd.print('0');
  lcd.print(sec);
}

void printLine(uint8_t row, const char* text) {
  lcd.setCursor(0, row);
  lcd.print(text);
  for (uint8_t i = strlen(text); i < 16; i++) lcd.print(' ');
}

void updateLCD(bool force) {
  static unsigned long lastDraw = 0;
  unsigned long now = millis();
  if (!force && now - lastDraw < 200) return;
  lastDraw = now;

  if (speedMenu) {
    printLine(0, "Geschwindigkeit");
    lcd.setCursor(0, 1);
    char buf[17];
    snprintf(buf, sizeof(buf), "%3d%%  %2dU/m P%3d", speedPct, (int)(TANK_RPM_SOLL * speedPct / 100 + 0.5), targetPWM());
    lcd.print(buf);
    return;
  }

  lcd.setCursor(0, 0);
  switch (mode) {
    case SET_TIME:
      lcd.print("Zeit:     "); printTime((long)setTimeS * 1000L); lcd.print(' ');
      printLine(1, "Druck = Start");
      break;

    case RUNNING:
      lcd.print("Rest:     "); printTime(remainingMs); lcd.print(' ');
      lcd.setCursor(0, 1);
      if (directionForward) lcd.print(">> VOR      ");
      else                  lcd.print("<< ZUR\xF5" "CK   ");
      {
        char buf[5]; snprintf(buf, sizeof(buf), "%3d%%", speedPct); lcd.print(buf);
      }
      break;

    case PAUSED:
      lcd.print("Rest:     "); printTime(remainingMs); lcd.print(' ');
      printLine(1, "PAUSE Druck=Lauf");
      break;

    case FINISHED:
      printLine(0, "    FERTIG!");
      printLine(1, "dr\xF5" "cken = zur\xF5" "ck");
      break;
  }
}

// ================= SETUP / LOOP =================
void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  motorOff();
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  pinMode(BTN, INPUT_PULLUP);
  pinMode(ENC_A, INPUT_PULLUP);
  pinMode(ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(ENC_A), isrEncoder, CHANGE);

  lcd.init();
  lcd.backlight();
  printLine(0, "Rotation");
  printLine(1, "65 U/min  3,5s");
  delay(1200);
  updateLCD(true);
}

void loop() {
  unsigned long now = millis();

  handleButton(now);

  // ----- Encoder -----
  int steps = readEncoderSteps();
  if (steps != 0) {
    if (speedMenu) {
      speedPct = constrain(speedPct + steps, SPEED_MIN_PCT, SPEED_MAX_PCT);
    } else if (mode == SET_TIME) {
      long t = (long)setTimeS + (long)steps * TIME_STEP_S;
      setTimeS = constrain(t, (long)MIN_TIME_S, (long)MAX_TIME_S);
    } else if (mode == PAUSED && motorState == M_OFF) {
      mode = SET_TIME;                      // aus der Pause zurück zur Zeiteinstellung
    }
    updateLCD(true);
  }

  // ----- Countdown -----
  if (mode == RUNNING) {
    remainingMs -= (long)(now - lastTick);
    lastTick = now;
    if (remainingMs <= 0) finishRun();
  }

  updateMotor(now);
  updateLCD(false);
}
