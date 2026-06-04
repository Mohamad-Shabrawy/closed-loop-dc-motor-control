#define BLYNK_TEMPLATE_ID "TMPL2SzwRzWPA"
#define BLYNK_TEMPLATE_NAME "LED ESP32"
#define BLYNK_AUTH_TOKEN "YOUR_BLYNK_AUTH_TOKEN"

#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <Keypad.h>

// -----------------------------------------------------------------------------
// WiFi / Blynk
// -----------------------------------------------------------------------------
char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "YOUR_WIFI_SSID";
char pass[] = "YOUR_WIFI_PASSWORD";

BlynkTimer timer;

// -----------------------------------------------------------------------------
// Physical LCD
// -----------------------------------------------------------------------------
LiquidCrystal_I2C lcd(0x27, 16, 2);
void lcdPrintLine(uint8_t row, const String &text) {
  static String last[2] = {"", ""};
  String s = text;
  if (s.length() > 16) s.remove(16);
  while (s.length() < 16) s += ' ';
  if (row > 1 || last[row] == s) return;
  last[row] = s;
  lcd.setCursor(0, row);
  lcd.print(s);
}

// Blynk LCD Widget
WidgetLCD lcdApp(V4);
void showLCD(uint8_t row, String text) {
  lcdPrintLine(row, text);
  if (text.length() > 16) text.remove(16);
  while (text.length() < 16) text += " ";
  lcdApp.print(0, row, text);
}

// -----------------------------------------------------------------------------
// Keypad
// -----------------------------------------------------------------------------
const byte ROWS = 4, COLS = 3;
char keys[ROWS][COLS] = {
  {'1','2','3'},
  {'4','5','6'},
  {'7','8','9'},
  {'*','0','#'}
};
byte rowPins[ROWS] = {19, 18, 5, 17};
byte colPins[COLS] = {2, 16, 4};
Keypad keypad = Keypad(makeKeymap(keys), rowPins, colPins, ROWS, COLS);

// -----------------------------------------------------------------------------
// Motor / Encoder
// -----------------------------------------------------------------------------
#define PWM_PIN    23
#define DIR_PIN    25
#define ENCODER_A  32
#define ENCODER_B  33

volatile long pulseCount = 0;
float liveRPM = 0.0f;
const int PPR = 7;
const int CPR = PPR * 4;

int targetRPM = 0;
bool directionForward = true;
bool motorRunning = false;

const int pwmChannel    = 0;
const int pwmFreq       = 20000;
const int pwmResolution = 10;

int baseDuty = 0;
int currentDuty = 0;

// -----------------------------------------------------------------------------
// Potentiometer
// -----------------------------------------------------------------------------
#define POT_PIN 36
int lastPotRPM = -999;
const int potThreshold = 500; // reduced for faster update

// -----------------------------------------------------------------------------
// Feed-forward curve
// -----------------------------------------------------------------------------
struct DutyMap { int rpm, duty; };

const DutyMap motorCurve[] = {
  {0,0},
  {167,25},
  {505,50},
  {802,75},
  {1111,100},
  {1423,125},
  {1738,150},
  {2053,175},
  {2366,200},
  {2688,225},
  {3010,250},
  {3312,275},
  {3628,300},
  {3936,325},
  {4260,350},
  {4572,375},
  {4877,400},
  {5206,425},
  {5531,450},
  {5846,475},
  {6158,500},
  {6458,525},
  {6778,550},
  {7102,575},
  {7409,600},
  {7733,625},
  {8030,650},
  {8365,675},
  {8684,700},
  {9008,725},
  {9334,750},
  {9625,775},
  {9901,800},
  {10214,825},
  {10523,850},
  {10765,875},
  {11062,900},
  {11347,925},
  {11678,950},
  {11979,975},
  {12000,977},   // derived from your last two points
  {12284,1000}   // measured (above 12k)
};

const int curveSize = sizeof(motorCurve) / sizeof(motorCurve[0]);
float dutyBias[curveSize] = {0};

int predictDuty(int rpmTarget) {
  if (rpmTarget <= 0) return 0;

  if (rpmTarget >= motorCurve[curveSize - 1].rpm) {
    float d = motorCurve[curveSize - 1].duty + dutyBias[curveSize - 1];
    return constrain((int)d, 0, 1023);
  }

  for (int i = 0; i < curveSize - 1; i++) {
    if (rpmTarget >= motorCurve[i].rpm && rpmTarget <= motorCurve[i+1].rpm) {
      float ratio = (float)(rpmTarget - motorCurve[i].rpm) /
                    (motorCurve[i+1].rpm - motorCurve[i].rpm);

      float d0 = motorCurve[i].duty   + dutyBias[i];
      float d1 = motorCurve[i+1].duty + dutyBias[i+1];
      int duty = (int)(d0 + ratio * (d1 - d0));
      return constrain(duty, 0, 1023);
    }
  }
  return constrain(motorCurve[curveSize - 1].duty + dutyBias[curveSize - 1], 0, 1023);
}

void computeBaseDutyFromTargetRPM() {
  baseDuty = predictDuty(targetRPM);
  if (baseDuty > 0 && baseDuty < 60) baseDuty = 60;
}

// -----------------------------------------------------------------------------
// PID
// -----------------------------------------------------------------------------
float Kp = 0.02f;
float Ki = 0.00015f;
float Kd = 0.0006f;
float pidIntegral = 0;
float lastError = 0;
float lastDerivative = 0;
const float DERIVATIVE_FILTER = 0.62f;

float rpmBuf[12] = {0};     // â† CHANGED (20 â†’ 12)
int rpmPtr = 0;

float smoothRPM(float raw) {
  rpmBuf[rpmPtr] = raw;
  rpmPtr = (rpmPtr + 1) % 12;   // â† CHANGED
  float sum = 0;
  for (int i = 0; i < 12; i++) sum += rpmBuf[i];   // â† CHANGED
  liveRPM = sum / 12.0;        // â† CHANGED
  return liveRPM;
}

void IRAM_ATTR readEncoderA() {
  pulseCount += (digitalRead(ENCODER_A) == digitalRead(ENCODER_B) ? 1 : -1);
}
void IRAM_ATTR readEncoderB() {
  pulseCount += (digitalRead(ENCODER_A) != digitalRead(ENCODER_B) ? 1 : -1);
}

void applyDuty(int d) {
  currentDuty = constrain(d, 0, 1023);
  ledcWrite(pwmChannel, currentDuty);
}

void softStartToBaseDuty(int finalBaseDuty) {
  int start = currentDuty;
  int step = max(abs(finalBaseDuty - start) / 30, 2);
  for (int d = start; (finalBaseDuty > start ? d <= finalBaseDuty : d >= finalBaseDuty); d += (finalBaseDuty > start ? step : -step)) {
    applyDuty(d);
    delay(25);
    Blynk.run();
  }
  applyDuty(finalBaseDuty);
}

void softStopToZero() {
  int step = max(currentDuty / 30, 4);
  for (int d = currentDuty; d >= 0; d -= step) {
    applyDuty(d);
    delay(25);
    Blynk.run();
  }
  applyDuty(0);
}

void updateDisplay() {
  static unsigned long last = 0;
  if (millis() - last < 150) return;
  last = millis();

  if (!motorRunning) {
    showLCD(0, "Enter Speed:");
    showLCD(1, String(targetRPM));
  } else {
    showLCD(0, "Set Speed:" + String(targetRPM));
    showLCD(1, "Live Speed:" + String((int)liveRPM));
  }
}

void stopMotor();

void safeSetDirection(bool forward) {
  directionForward = forward;
  digitalWrite(DIR_PIN, forward ? HIGH : LOW);
}

void startMotor() {
  if (targetRPM <= 0) {
    showLCD(0, "Enter Speed:");
    showLCD(1, "");
    return;
  }

  safeSetDirection(directionForward);
  pidIntegral = lastError = lastDerivative = 0;

  for (int i = 0; i < 12; i++) rpmBuf[i] = 0;
  liveRPM = 0;

  computeBaseDutyFromTargetRPM();
  softStartToBaseDuty(baseDuty);
  motorRunning = true;
  updateDisplay();
  Blynk.virtualWrite(V3, 1);
}

void stopMotor() {
  motorRunning = false;
  softStopToZero();
  applyDuty(0);
  showLCD(0, "Motor STOPPED");
  showLCD(1, "");
  delay(500);
  showLCD(0, "Enter Speed:");
  showLCD(1, String(targetRPM));
  Blynk.virtualWrite(V3, 0);
}

// -----------------------------------------------------------------------------
// Potentiometer Fix
// -----------------------------------------------------------------------------
int readPotRPM() {
  long sum = 0;
  const int samples = 15;
  for (int i = 0; i < samples; i++) {
    sum += analogRead(POT_PIN);
    delayMicroseconds(150);
  }
  int avg = sum / samples;
  return map(avg, 0, 4095, 0, 12000);
}

void checkPotControl() {
  int potRPM = readPotRPM();
  if (abs(potRPM - lastPotRPM) > potThreshold) {
    targetRPM = potRPM;
    computeBaseDutyFromTargetRPM();
    Blynk.virtualWrite(V1, targetRPM);
    updateDisplay();      // <--- FIXED
    if (motorRunning) softStartToBaseDuty(baseDuty);
    lastPotRPM = potRPM;
  }
}

// -----------------------------------------------------------------------------
// Blynk handlers
// -----------------------------------------------------------------------------
BLYNK_WRITE(V1) {
  targetRPM = constrain(param.asInt(), 0, 12000);
  computeBaseDutyFromTargetRPM();
  updateDisplay();       // <--- LCD in sync
  if (motorRunning) softStartToBaseDuty(baseDuty);
}

BLYNK_WRITE(V2) {
  bool newForward = (param.asInt() == 0);
  if (newForward == directionForward) return;

  if (motorRunning) {
    stopMotor();
    delay(300);
  }

  safeSetDirection(newForward);
  if (targetRPM > 0 && !motorRunning) {
    computeBaseDutyFromTargetRPM();
    startMotor();
  }
}

BLYNK_WRITE(V3) {
  if (param.asInt() == 1) startMotor();
  else stopMotor();
}

// -----------------------------------------------------------------------------
// updateRPM loop (PID + safety)
// -----------------------------------------------------------------------------
void updateRPM() {
  if (!motorRunning) {
    noInterrupts(); pulseCount = 0; interrupts();
    return;
  }

static unsigned long lastTime = 0;
unsigned long now = millis();

if (lastTime == 0) {          // first call
  lastTime = now;
  return;
}

float dt = (now - lastTime) / 1000.0f;
lastTime = now;

if (dt <= 0.0f) return;       // safety


  noInterrupts();
  long count = pulseCount;
  pulseCount = 0;
  interrupts();

  float rawRPM = (abs(count) / (float)CPR) * (60.0f / dt);
  smoothRPM(rawRPM);

  float error = targetRPM - liveRPM;
  if (abs(error) < 5) error = 0;

  static unsigned long stallStart = 0;
  if (targetRPM > 200) {
    if (liveRPM < targetRPM * 0.2f) {
      if (stallStart == 0) stallStart = now;
      else if (now - stallStart > 2000) {
        stopMotor();
        showLCD(0, "STALL ERROR");
        showLCD(1, "Check load");
        stallStart = 0;
        return;
      }
    } else stallStart = 0;
  }

  static unsigned long overStart = 0;
  if (liveRPM > targetRPM * 1.3f) {
    if (overStart == 0) overStart = now;
    else if (now - overStart > 1000) {
      applyDuty(currentDuty / 2);
      overStart = 0;
    }
  } else overStart = 0;

  if (abs(error) > 10 && abs(error) < 500) {
    int seg = 0;
    for (int i = 0; i < curveSize - 1; i++)
      if (targetRPM >= motorCurve[i].rpm &&
          targetRPM <= motorCurve[i+1].rpm) seg = i;

    float dutyPerRPM = 1023.0f / 12000.0f;
    dutyBias[seg] = constrain(dutyBias[seg] + 0.02f * dutyPerRPM * error, -120.0f, 120.0f);
  }

  for (int i = 0; i < curveSize; i++)
    dutyBias[i] *= 0.9997f;

  baseDuty = predictDuty(targetRPM);

  float derivative = (error - lastError) / dt;
  derivative = DERIVATIVE_FILTER * lastDerivative + (1 - DERIVATIVE_FILTER) * derivative;
  lastDerivative = derivative;
  lastError = error;

  if (abs(error) > 10)
    pidIntegral = constrain(pidIntegral + error * dt, -12000, 12000);

  int duty = baseDuty +
             (int)(Kp * error) +
             (int)(Ki * pidIntegral) +
             (int)(Kd * derivative);

  applyDuty(duty);

  updateDisplay();
  Blynk.virtualWrite(V0, liveRPM);
  Blynk.virtualWrite(V1, targetRPM);
  Blynk.virtualWrite(V2, directionForward ? 0 : 1);
}

// -----------------------------------------------------------------------------
// Setup
// -----------------------------------------------------------------------------
void setup() {
  Serial.begin(115200);
  lcd.init();
  lcd.backlight();

  showLCD(0, "Connecting WiFi");
  showLCD(1, "");

  pinMode(DIR_PIN, OUTPUT);
  digitalWrite(DIR_PIN, HIGH);

  ledcSetup(pwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(PWM_PIN, pwmChannel);
  applyDuty(0);

  pinMode(ENCODER_A, INPUT_PULLUP);
  pinMode(ENCODER_B, INPUT_PULLUP);
  pinMode(POT_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(ENCODER_A), readEncoderA, CHANGE);
  attachInterrupt(digitalPinToInterrupt(ENCODER_B), readEncoderB, CHANGE);

  Blynk.begin(auth, ssid, pass);
  lcdApp.clear();

  timer.setInterval(150L, updateRPM);

  showLCD(0, "Enter Speed:");
  showLCD(1, "0");
}

// -----------------------------------------------------------------------------
// Main loop
// -----------------------------------------------------------------------------
void loop() {
  Blynk.run();
  timer.run();

  static bool newEntry = true;
  static bool typing = false;
  static unsigned long lastKeyTime = 0;


  if (!typing) checkPotControl();

  char key = keypad.getKey();
  if (!key) return;
  lastKeyTime = millis();
  typing = true;
  if (key == '#') {
    typing = false;
    if (motorRunning) {
      stopMotor();
      delay(300);
    }

    newEntry = true;
    showLCD(0, "1:FWD 2:REV");
    showLCD(1, "");

    char dkey = 0;
    while (dkey != '1' && dkey != '2') {
      dkey = keypad.getKey();
      Blynk.run();
      timer.run(); 
    }

    directionForward = (dkey == '1');
    safeSetDirection(directionForward);

    showLCD(0, directionForward ? "FORWARD" : "REVERSE");
    showLCD(1, "");
    delay(600);

    computeBaseDutyFromTargetRPM();
    startMotor();
  }

  else if (key >= '0' && key <= '9') {
    typing = true;
    if (newEntry) {
      targetRPM = 0;
      newEntry = false;
      showLCD(0, "Enter Speed:");
      showLCD(1, "");
    }

    targetRPM = constrain(targetRPM * 10 + (key - '0'), 0, 12000);
    computeBaseDutyFromTargetRPM();
    updateDisplay();
  }

  else if (key == '*') {
    typing = false;
    lastPotRPM = readPotRPM();
    targetRPM = lastPotRPM;
    newEntry = true;
    stopMotor();
  }
}
