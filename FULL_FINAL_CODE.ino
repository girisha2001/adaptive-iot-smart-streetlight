#define REMOTEXY_MODE__WIFI_POINT
#include <WiFi.h>
#include <RemoteXY.h>
#include <Wire.h>
#include <RTClib.h>
#include <Adafruit_INA219.h>

// --- RemoteXY connection settings ---
#define REMOTEXY_WIFI_SSID "RemoteXY"
#define REMOTEXY_WIFI_PASSWORD "12345678"
#define REMOTEXY_SERVER_PORT 6377

// --- BRIGHTNESS SETTINGS ---
#define N_FULL 100
#define N_MID  70
#define N_IDLE 30
#define L_FULL 100
#define L_MID  50
#define L_IDLE 20
#define DAY_DARK_IDLE 40

// --- SPEED CONSTANTS ---
#define SENSOR_DISTANCE_M 0.11   // 11 cm between sensor 1 and sensor 2

#define FAST_VEHICLE_THRESHOLD   4.0
#define SLOW_VEHICLE_THRESHOLD   1.5

#define STEP_TIME_FAST    1500
#define STEP_TIME_NORMAL  2500
#define STEP_TIME_SLOW    3500
int STEP_TIME = STEP_TIME_NORMAL;

float actualSpeedKmH = 0;
float sequenceDuration = 0;
String speedStatusExcel = "STDBY";

// --- TRAFFIC DENSITY SETTINGS ---
#define TRAFFIC_WINDOW_MS 15000UL
#define TRAFFIC_LOW_IDLE   30
#define TRAFFIC_MED_IDLE   50
#define TRAFFIC_HIGH_IDLE  70

// --- BATTERY / RUNTIME SETTINGS ---
#define BATTERY_CAPACITY_MAH 4000.0
float estimatedRuntime = 0;

// --- REAL BATTERY SETTINGS ---
#define BATTERY_FULL_V 4.20
#define BATTERY_EMPTY_V 3.30
#define LOW_ENERGY_ENTER_PCT 40.0
#define LOW_ENERGY_EXIT_PCT 45.0

// --- HYBRID BATTERY UPDATE RATES ---
#define CHARGE_RATE_DAY          3.0
#define DISCHARGE_RATE_NIGHT     2.5
#define DISCHARGE_RATE_ACTIVE    4.0

// Flip sign if INA219 current direction is reversed
#define INA219_CURRENT_SIGN 1

// RemoteXY GUI configuration
#pragma pack(push, 1)
uint8_t const PROGMEM RemoteXY_CONF_PROGMEM[] =   // 409 bytes V19
  { 255,110,0,39,0,146,1,19,0,0,0,73,111,84,95,83,116,114,101,101,
  116,76,105,103,104,116,105,110,103,95,83,121,115,116,101,109,0,31,1,106,
  200,1,1,26,0,12,1,40,42,7,193,30,1,65,85,84,79,32,77,79,
  68,69,32,0,70,79,82,67,69,32,68,65,89,32,0,70,79,82,67,69,
  32,78,73,71,72,84,0,129,10,50,88,8,64,6,69,78,69,82,71,89,
  32,77,65,78,65,71,69,77,69,78,84,0,71,21,60,52,52,56,0,2,
  24,78,0,0,0,0,0,0,200,66,0,0,160,65,0,0,32,65,0,0,
  0,64,24,0,7,45,40,58,7,102,64,2,26,2,26,7,3,104,100,7,
  102,64,2,26,2,26,67,3,124,35,8,78,2,26,2,70,40,124,8,8,
  16,135,134,0,70,5,3,7,7,17,95,37,0,70,5,30,7,7,17,95,
  37,0,70,5,21,7,7,17,95,37,0,70,5,12,7,7,17,95,37,0,
  67,16,3,38,7,78,2,26,2,67,16,12,38,7,78,2,26,2,67,16,
  21,38,7,78,2,26,2,67,16,30,38,7,78,2,26,2,4,58,3,41,
  6,128,2,26,4,58,12,41,6,128,2,26,4,58,21,41,6,128,2,26,
  4,59,30,40,6,128,2,26,7,3,114,100,7,102,64,2,26,2,26,67,
  50,124,53,8,94,2,26,1,89,183,15,15,0,37,31,82,69,83,69,84,
  0,7,3,95,100,7,102,64,2,26,2,26,74,50,135,54,6,44,2,30,
  37,64,76,73,71,72,84,32,84,82,65,70,70,73,67,0,37,64,77,79,
  68,69,82,65,84,69,32,84,82,65,70,70,73,67,0,37,64,72,69,65,
  86,89,32,84,82,65,70,70,73,67,0,67,3,135,45,6,78,2,26,1,
  67,3,144,101,8,78,2,26,3 };

struct {
  uint8_t system_status;
  char status_text[26];
  char energy_mode_text[26];
  int8_t slider_1;
  int8_t slider_2;
  int8_t slider_3;
  int8_t slider_4;
  char live_clock[26];
  uint8_t reset_btn;
  char charge_status[26];

  float battery_percent;
  float ldr_value;
  uint8_t led_cloudy;
  uint8_t led_p1;
  uint8_t led_p4;
  uint8_t led_p3;
  uint8_t led_p2;
  float perc_1;
  float perc_2;
  float perc_3;
  float perc_4;
  int8_t vehicle_count_val;
  uint8_t traffic_text;
  float value_01;
  float runtime_text;

  uint8_t connect_flag;
} RemoteXY;
#pragma pack(pop)

// --- PIN DEFINES ---
#define TRIG_PIN 5
#define ECHO_PIN 18
#define TRIG_PIN_2 19
#define ECHO_PIN_2 23
#define POLE1 25
#define POLE2 26
#define POLE3 27
#define POLE4 14
#define LDR1 32
#define LDR2 33
#define LDR3 34
#define LDR4 35

RTC_DS3231 rtc;
Adafruit_INA219 ina219;

unsigned long sequenceStartTime = 0;
unsigned long lastDebugPrint = 0;
unsigned long lastTextUpdate = 0;
bool vehicleActive = false;
int totalVehicles = 0;
float batteryPercentReal = 0.0;
float displayedBatteryPercent = 0.0;
float smoothedVoltage = 3.70;
static bool lowEnergyMode = false;

int curP1, curP2, curP3, curP4;

// --- Sensor timing variables ---
unsigned long sensor1TriggerTime = 0;
unsigned long lastVehicleDetectTime = 0;
bool waitingForSpeed = false;

unsigned long trafficWindowStart = 0;
int vehiclesInWindow = 0;
int trafficIdleTarget = TRAFFIC_LOW_IDLE;

// --- Battery update timing ---
unsigned long lastBatteryUpdate = 0;

// --- Track previous mode so RTC changes only once per mode switch ---
uint8_t prevSystemStatus = 255;

void sendPLXHeader() {
  Serial.println("CLEARDATA");
  Serial.println("LABEL,Time,Cycle(as per RTC timing),LDR,Mode,Voltage(V),Current(mA),P1(%),P2(%),P3(%),P4(%),Battery(%),EnergyMode,VehicleActive,Speed(km/h),SpeedStatus,StepTime(ms),SeqDuration(s),TotalVehicles,TrafficLevel,EstRuntime(hrs)");
}

void updateTrafficLevel() {
  if (vehiclesInWindow <= 2) {
    RemoteXY.traffic_text = 1;
    trafficIdleTarget = TRAFFIC_LOW_IDLE;
  }
  else if (vehiclesInWindow <= 5) {
    RemoteXY.traffic_text = 2;
    trafficIdleTarget = TRAFFIC_MED_IDLE;
  }
  else {
    RemoteXY.traffic_text = 3;
    trafficIdleTarget = TRAFFIC_HIGH_IDLE;
  }
}

int readUltrasonicCM(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 15000);
  if (duration == 0) return 999;

  return duration * 0.034 / 2;
}

void measureSpeed() {
  int dist1 = readUltrasonicCM(TRIG_PIN, ECHO_PIN);
  int dist2 = readUltrasonicCM(TRIG_PIN_2, ECHO_PIN_2);

  unsigned long now = millis();

  // SENSOR 1 = vehicle detect + count + start sequence
  if (dist1 > 2 && dist1 < 25) {
    if ((now - lastVehicleDetectTime > 1500) && !waitingForSpeed) {
      lastVehicleDetectTime = now;
      sensor1TriggerTime = now;
      waitingForSpeed = true;

      totalVehicles++;
      vehiclesInWindow++;
      updateTrafficLevel();

      RemoteXY.vehicle_count_val = totalVehicles;
      RemoteXY.value_01 = vehiclesInWindow;

      actualSpeedKmH = 0;
      speedStatusExcel = "MEASURING";
      STEP_TIME = STEP_TIME_NORMAL;

      vehicleActive = true;
      sequenceStartTime = now;
      sequenceDuration = 0;
    }
  }

  // SENSOR 2 = speed calculate only
  if (waitingForSpeed && dist2 > 2 && dist2 < 25) {
    unsigned long timeDiffMs = now - sensor1TriggerTime;

    if (timeDiffMs > 50 && timeDiffMs < 4000) {
      actualSpeedKmH = (SENSOR_DISTANCE_M / (timeDiffMs / 1000.0)) * 3.6;

      if (actualSpeedKmH >= FAST_VEHICLE_THRESHOLD) {
        STEP_TIME = STEP_TIME_FAST;
        speedStatusExcel = "FAST";
      }
      else if (actualSpeedKmH <= SLOW_VEHICLE_THRESHOLD) {
        STEP_TIME = STEP_TIME_SLOW;
        speedStatusExcel = "SLOW";
      }
      else {
        STEP_TIME = STEP_TIME_NORMAL;
        speedStatusExcel = "NORMAL";
      }

      waitingForSpeed = false;
    }
  }

  // Timeout if sensor 2 does not detect anything
  if (waitingForSpeed && (now - sensor1TriggerTime > 4000)) {
    actualSpeedKmH = 0;
    STEP_TIME = STEP_TIME_NORMAL;
    speedStatusExcel = "NORMAL";
    waitingForSpeed = false;
  }
}

void applyModeRTC(uint8_t modeValue) {
  DateTime current = rtc.now();

  if (modeValue == 1) {
    // FORCE DAY -> 07:00:00
    rtc.adjust(DateTime(current.year(), current.month(), current.day(), 7, 0, 0));
  }
  else if (modeValue == 2) {
    // FORCE NIGHT -> 19:00:00
    rtc.adjust(DateTime(current.year(), current.month(), current.day(), 19, 0, 0));
  }
  else if (modeValue == 0) {
    // AUTO MODE -> 12:00:00
    rtc.adjust(DateTime(current.year(), current.month(), current.day(), 12, 0, 0));
  }
}

float readBatteryVoltage() {
  return ina219.getBusVoltage_V();
}

float voltageToBatteryPercent(float voltage) {
  if (voltage >= 4.20) return 100;
  else if (voltage >= 4.15) return 95;
  else if (voltage >= 4.10) return 90;
  else if (voltage >= 4.05) return 85;
  else if (voltage >= 4.00) return 80;
  else if (voltage >= 3.95) return 70;
  else if (voltage >= 3.90) return 60;
  else if (voltage >= 3.85) return 50;
  else if (voltage >= 3.80) return 40;
  else if (voltage >= 3.75) return 30;
  else if (voltage >= 3.70) return 20;
  else if (voltage >= 3.60) return 10;
  else if (voltage >= 3.50) return 5;
  else if (voltage >= 3.30) return 0;
  else return 0;
}

float calculateRuntimeHours(float batteryPercent, float loadCurrent_mA) {
  if (loadCurrent_mA <= 1.0) return 0.0;

  float remaining_mAh = (batteryPercent / 100.0) * BATTERY_CAPACITY_MAH;
  return remaining_mAh / loadCurrent_mA;
}

void setup() {
  Serial.begin(38400);
  Wire.begin(21, 22);
  rtc.begin();
  ina219.begin();

  // ===== DEMO TIME SET (UNCOMMENT ONLY ONCE IF NEEDED) =====
  rtc.adjust(DateTime(2026, 1, 31, 19, 0, 0)); // NIGHT (7 PM) NIGHT MODE
  // rtc.adjust(DateTime(2026, 1, 31, 7, 0, 0));   // DAY (7 AM) DAY MODE
  // rtc.adjust(DateTime(2026, 1, 31, 12, 0, 0)); // AUTO MODE (12 PM)

  RemoteXY_Init();

  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
  pinMode(TRIG_PIN_2, OUTPUT);
  pinMode(ECHO_PIN_2, INPUT);

  pinMode(POLE1, OUTPUT);
  pinMode(POLE2, OUTPUT);
  pinMode(POLE3, OUTPUT);
  pinMode(POLE4, OUTPUT);

  pinMode(LDR1, INPUT);
  pinMode(LDR2, INPUT);
  pinMode(LDR3, INPUT);
  pinMode(LDR4, INPUT);

  // initialize battery display from real battery voltage
  smoothedVoltage = readBatteryVoltage();
  displayedBatteryPercent = voltageToBatteryPercent(smoothedVoltage);
  batteryPercentReal = displayedBatteryPercent;

  strcpy(RemoteXY.energy_mode_text, "NORMAL MODE");
  strcpy(RemoteXY.charge_status, "IDLE");

  delay(1500);
  sendPLXHeader();
  trafficWindowStart = millis();

  // Set initial tracker so RTC does not keep resetting on startup
  prevSystemStatus = RemoteXY.system_status;
}

void loop() {
  RemoteXY_Handler();

  // --- Change RTC only when mode is switched from phone ---
  if (RemoteXY.system_status != prevSystemStatus) {
    applyModeRTC(RemoteXY.system_status);
    prevSystemStatus = RemoteXY.system_status;
  }

  DateTime now = rtc.now();
  sprintf(RemoteXY.live_clock, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());

  measureSpeed();

  if (RemoteXY.reset_btn == 1) {
    totalVehicles = 0;
    lowEnergyMode = false;
    actualSpeedKmH = 0;
    speedStatusExcel = "STDBY";
    vehiclesInWindow = 0;
    trafficWindowStart = millis();
    waitingForSpeed = false;
    vehicleActive = false;
    STEP_TIME = STEP_TIME_NORMAL;
    estimatedRuntime = 0;

    smoothedVoltage = readBatteryVoltage();
    displayedBatteryPercent = voltageToBatteryPercent(smoothedVoltage);
    batteryPercentReal = displayedBatteryPercent;

    sendPLXHeader();
    RemoteXY.reset_btn = 0;
  }

  RemoteXY.vehicle_count_val = totalVehicles;
  RemoteXY.value_01 = vehiclesInWindow;

  float batteryVoltage = readBatteryVoltage();
  float rawCurrent_mA = ina219.getCurrent_mA();
  float batteryCurrent_mA = rawCurrent_mA * INA219_CURRENT_SIGN;

  bool isDark = (digitalRead(LDR1) == 1 || digitalRead(LDR2) == 1 || digitalRead(LDR3) == 1 || digitalRead(LDR4) == 1);
  bool isDaytime = (now.hour() >= 7 && now.hour() < 19);

  RemoteXY.led_cloudy = (isDaytime && isDark) ? 1 : 0;

  // --- MODE-BASED CHARGE / DISCHARGE STATUS ---
  bool shouldCharge = (RemoteXY.system_status == 1) || (RemoteXY.system_status == 0 && isDaytime);
  bool shouldDischarge = (RemoteXY.system_status == 2) || (RemoteXY.system_status == 0 && !isDaytime);

  // --- HYBRID BATTERY UPDATE (every 5 seconds) ---
  if (millis() - lastBatteryUpdate > 2000) {
    lastBatteryUpdate = millis();

    // keep small reference to real voltage
    smoothedVoltage = (smoothedVoltage * 0.97) + (batteryVoltage * 0.03);
    batteryPercentReal = voltageToBatteryPercent(batteryVoltage);

    // if battery display ever becomes invalid, recover from real reading
    if (displayedBatteryPercent < 0.0 || displayedBatteryPercent > 100.0) {
      displayedBatteryPercent = batteryPercentReal;
    }

    // hybrid behaviour
    if (shouldCharge) {
      displayedBatteryPercent += CHARGE_RATE_DAY;
    }

    if (shouldDischarge) {
      if (vehicleActive) displayedBatteryPercent -= DISCHARGE_RATE_ACTIVE;
      else displayedBatteryPercent -= DISCHARGE_RATE_NIGHT;
    }

    // soft anchor to real measured battery
    displayedBatteryPercent = (displayedBatteryPercent * 0.98) + (batteryPercentReal * 0.02);

    displayedBatteryPercent = constrain(displayedBatteryPercent, 0.0, 100.0);
  }

  // display battery %
  RemoteXY.battery_percent = displayedBatteryPercent;
  RemoteXY.ldr_value = (float)isDark;

  // --- LOW ENERGY MODE ---
  if (displayedBatteryPercent < LOW_ENERGY_ENTER_PCT) lowEnergyMode = true;
  if (displayedBatteryPercent >= LOW_ENERGY_EXIT_PCT) lowEnergyMode = false;

  if (millis() - lastTextUpdate >= 1000) {
    lastTextUpdate = millis();

    if (shouldCharge)
      strcpy(RemoteXY.charge_status, "SOLAR CHARGING");
    else if (shouldDischarge)
      strcpy(RemoteXY.charge_status, "SOLAR DISCHARGING");
    else
      strcpy(RemoteXY.charge_status, "IDLE");
  }

  // --- ENERGY MODE TEXT ---
  strcpy(RemoteXY.energy_mode_text,
         lowEnergyMode ? "LOW ENERGY MODE" : "NORMAL MODE");

  // --- RUNTIME CALCULATION ---
  float effectiveCurrent_mA;

  if (vehicleActive) {
    effectiveCurrent_mA = 180.0;   // active sequence
  } else if (lowEnergyMode) {
    effectiveCurrent_mA = 70.0;    // low energy idle
  } else {
    effectiveCurrent_mA = 100.0;   // normal idle
  }

  estimatedRuntime = calculateRuntimeHours(displayedBatteryPercent, effectiveCurrent_mA);
  RemoteXY.runtime_text = estimatedRuntime;

  if (millis() - trafficWindowStart >= TRAFFIC_WINDOW_MS) {
    vehiclesInWindow = 0;
    trafficWindowStart = millis();
    updateTrafficLevel();
  }

  bool lightsON = (RemoteXY.system_status == 2) ||
                  (RemoteXY.system_status == 0 && (!isDaytime || isDark)) ||
                  (RemoteXY.system_status == 1 && isDark);

  if (!lightsON) {
    strcpy(RemoteXY.status_text, (RemoteXY.system_status == 1) ? "FORCED DAY" : "AUTO: DAY");
    updateHardware(0, 0, 0, 0);

    if (vehicleActive && (millis() - sequenceStartTime > STEP_TIME * 4)) {
      sequenceDuration = (millis() - sequenceStartTime) / 1000.0;
      vehicleActive = false;
    }
  } else {
    if (RemoteXY.system_status == 2) strcpy(RemoteXY.status_text, "FORCED NIGHT");
    else if (!isDaytime) strcpy(RemoteXY.status_text, "AUTO: NIGHT");
    else strcpy(RemoteXY.status_text, "AUTO: DARK DAY");

    if (vehicleActive) handleSequence(lowEnergyMode, isDaytime, isDark);
    else {
      int idle = (isDaytime && isDark) ? DAY_DARK_IDLE : (lowEnergyMode ? L_IDLE : trafficIdleTarget);
      updateHardware(idle, idle, idle, idle);
    }
  }

  if (millis() - lastDebugPrint > 2000) {
    lastDebugPrint = millis();
    Serial.print("DATA,");
    Serial.print(RemoteXY.live_clock); Serial.print(",");
    Serial.print(isDaytime ? "DAY" : "NIGHT"); Serial.print(",");
    Serial.print(isDark ? "DARK" : "BRIGHT"); Serial.print(",");
    Serial.print(RemoteXY.status_text); Serial.print(",");
    Serial.print(batteryVoltage, 3); Serial.print(",");
    Serial.print(batteryCurrent_mA, 2); Serial.print(",");
    Serial.print(curP1); Serial.print(",");
    Serial.print(curP2); Serial.print(",");
    Serial.print(curP3); Serial.print(",");
    Serial.print(curP4); Serial.print(",");
    Serial.print(displayedBatteryPercent, 2); Serial.print(",");
    Serial.print(RemoteXY.energy_mode_text); Serial.print(",");
    Serial.print(vehicleActive ? 1 : 0); Serial.print(",");
    Serial.print(actualSpeedKmH); Serial.print(",");
    Serial.print(speedStatusExcel); Serial.print(",");
    Serial.print(STEP_TIME); Serial.print(",");
    Serial.print(sequenceDuration); Serial.print(",");
    Serial.print(totalVehicles); Serial.print(",");

    if (RemoteXY.traffic_text == 1) Serial.print("LIGHT");
    else if (RemoteXY.traffic_text == 2) Serial.print("MODERATE");
    else Serial.print("HEAVY");

    Serial.print(",");
    Serial.println(estimatedRuntime, 2);
    Serial.println(batteryVoltage);
  }
}

void handleSequence(bool lowMode, bool isDaytime, bool isDark) {
  unsigned long elapsed = millis() - sequenceStartTime;
  int f = lowMode ? L_FULL : N_FULL;
  int m = lowMode ? L_MID  : N_MID;

  int i;
  if (lowMode) {
    i = L_IDLE;
  } else if (isDaytime && isDark) {
    i = DAY_DARK_IDLE;
  } else {
    i = trafficIdleTarget;
  }

  if (elapsed < STEP_TIME)           updateHardware(f, m, i, i);
  else if (elapsed < STEP_TIME * 2)  updateHardware(m, f, m, i);
  else if (elapsed < STEP_TIME * 3)  updateHardware(i, m, f, m);
  else if (elapsed < STEP_TIME * 4)  updateHardware(i, i, m, f);
  else {
    sequenceDuration = (millis() - sequenceStartTime) / 1000.0;
    vehicleActive = false;
  }
}

void updateHardware(int p1, int p2, int p3, int p4) {
  curP1 = p1; curP2 = p2; curP3 = p3; curP4 = p4;

  RemoteXY.slider_1 = p1; RemoteXY.perc_1 = p1;
  RemoteXY.slider_2 = p2; RemoteXY.perc_2 = p2;
  RemoteXY.slider_3 = p3; RemoteXY.perc_3 = p3;
  RemoteXY.slider_4 = p4; RemoteXY.perc_4 = p4;

  RemoteXY.led_p1 = (p1 == 100);
  RemoteXY.led_p2 = (p2 == 100);
  RemoteXY.led_p3 = (p3 == 100);
  RemoteXY.led_p4 = (p4 == 100);

  analogWrite(POLE1, map(p1, 0, 100, 0, 255));
  analogWrite(POLE2, map(p2, 0, 100, 0, 255));
  analogWrite(POLE3, map(p3, 0, 100, 0, 255));
  analogWrite(POLE4, map(p4, 0, 100, 0, 255));
}
