/**************************************************************************
 * IoT-based Water Monitoring and Control System for Fish Farm
 * 
 * Authors: Khalid Hossain, Soumik Saha , Sayba Kamal Orni
 * Description: Real-time monitoring of temperature, pH, and turbidity
 *              with automated control of heaters, coolers, and pH correction
 * 
 * Hardware:
 *   - Arduino Uno/Mega
 *   - ESP8266 (ESP-01) WiFi Module
 *   - DS18B20 Temperature Sensor
 *   - Analog pH Sensor
 *   - Analog Turbidity Sensor
 *   - Relay modules for heater/cooler control
 *   - DC motors for acid/base dispensing
 * 
 * Cloud Platform: Blynk IoT
 **************************************************************************/

#define BLYNK_TEMPLATE_ID "TMPL6_ulKIBon"
#define BLYNK_TEMPLATE_NAME "fishfarm project"
#define BLYNK_AUTH_TOKEN "JxAKVvEapVuCEggjvH4zEZTVwIUn0K1t"

#include <OneWire.h>
#include <DallasTemperature.h>
#include <SoftwareSerial.h>
#include <ESP8266_Lib.h>
#include <BlynkSimpleShieldEsp8266.h>

// ==================== WiFi Configuration ====================
char ssid[] = "Khalid1";
char pass[] = "12345678";

// ESP8266 communication: RX on pin 2, TX on pin 3
SoftwareSerial EspSerial(2, 3);
ESP8266 wifi(&EspSerial);

// ==================== Sensor Configuration ====================
// DS18B20 Temperature Sensor
const byte ONE_WIRE_BUS = 5;
OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature tempSensor(&oneWire);

// Turbidity Sensor
const byte TURBIDITY_PIN = A0;
const byte NUM_SAMPLES = 100;
const int TURBIDITY_THRESHOLD = 30;  // Percentage threshold for alerts

// pH Sensor calibration constants
const int PH_PIN = A1;
const float V_THRESHOLD = 2.62;      // Voltage at pH 6.86
const float V_BUFFER = 0.02;         // ±0.02V deadband for smooth transition

// ==================== Actuator Pin Definitions ====================
// Temperature control
const byte HEATER_PIN = 8;
const byte COOLER_PIN = 9;
const byte HEATER_RELAY = 10;
const byte COOLER_RELAY = 11;

// pH correction motors
const byte ACID_MOTOR_PIN = 6;       // Lowers pH
const byte BASE_MOTOR_PIN = 7;       // Raises pH

// ==================== Control Parameters ====================
// Temperature setpoints (°C)
const float TEMP_HIGH = 26.5;
const float TEMP_LOW = 24.5;

// pH setpoints
const float PH_HIGH = 8.5;
const float PH_LOW = 6.5;
const float PH_CRITICAL_HIGH = 8.0;
const float PH_CRITICAL_LOW = 6.0;

// Motor actuation time (ms)
const int MOTOR_PULSE_DURATION = 500;

BlynkTimer timer;

// ==================== Helper Functions ====================

/**
 * Read analog pin with averaging to reduce noise
 * @param pin Analog pin number
 * @param samples Number of samples to average
 * @return Averaged ADC value
 */
int readAverageAnalog(uint8_t pin, uint8_t samples = NUM_SAMPLES) {
  uint32_t sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += analogRead(pin);
    delay(3);  // 3ms delay between samples
  }
  return sum / samples;
}

/**
 * Convert turbidity ADC value to percentage
 * 0 ADC = 100% turbidity (very dirty)
 * 850 ADC = 0% turbidity (clean water)
 * @param adcValue Raw ADC reading
 * @return Turbidity percentage (0-100%)
 */
int mapToPercent(int adcValue) {
  return map(adcValue, 0, 850, 100, 0);
}

/**
 * Calculate pH from sensor voltage using piecewise calibration
 * Calibration points: pH 4 @ 3.11V, pH 6.86 @ 2.62V, pH 9.18 @ 2.25V
 * @param voltage Sensor voltage (0-5V)
 * @return Calculated pH value
 */
float calculatePH(float voltage) {
  float pH;
  
  // High voltage region (pH < 6.86)
  if (voltage > V_THRESHOLD + V_BUFFER) {
    pH = -5.84 * voltage + 22.16;
  } 
  // Low voltage region (pH > 6.86)
  else if (voltage < V_THRESHOLD - V_BUFFER) {
    pH = -6.27 * voltage + 23.28;
  } 
  // Buffer zone: smooth interpolation between regions
  else {
    float pH1 = -5.84 * voltage + 22.16;
    float pH2 = -6.27 * voltage + 23.28;
    float weight = (voltage - (V_THRESHOLD - V_BUFFER)) / (2 * V_BUFFER);
    pH = pH1 * (1 - weight) + pH2 * weight;
  }
  
  return pH;
}

// ==================== Main Control Functions ====================

/**
 * Control temperature using heater and cooler
 * Maintains water temperature between 24.5°C and 26.5°C
 * @param tempC Current temperature in Celsius
 */
void controlTemperature(float tempC) {
  if (tempC == DEVICE_DISCONNECTED_C) return;
  
  if (tempC > TEMP_HIGH) {
    // Activate cooler
    digitalWrite(HEATER_PIN, HIGH);
    digitalWrite(COOLER_PIN, LOW);
    digitalWrite(HEATER_RELAY, LOW);
    digitalWrite(COOLER_RELAY, HIGH);
  } 
  else if (tempC < TEMP_LOW) {
    // Activate heater
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(COOLER_PIN, HIGH);
    digitalWrite(HEATER_RELAY, HIGH);
    digitalWrite(COOLER_RELAY, LOW);
  } 
  else {
    // Temperature in range - turn off both
    digitalWrite(HEATER_PIN, LOW);
    digitalWrite(COOLER_PIN, LOW);
    digitalWrite(HEATER_RELAY, HIGH);
    digitalWrite(COOLER_RELAY, HIGH);
  }
}

/**
 * Correct pH level using acid or base dispensing
 * Maintains pH between 6.5 and 8.5
 * @param pH Current pH value
 */
void correctPH(float pH) {
  if (pH > PH_HIGH) {
    // Dispense acid to lower pH
    digitalWrite(ACID_MOTOR_PIN, HIGH);
    delay(MOTOR_PULSE_DURATION);
    digitalWrite(ACID_MOTOR_PIN, LOW);
  } 
  else if (pH < PH_LOW) {
    // Dispense base to raise pH
    digitalWrite(BASE_MOTOR_PIN, HIGH);
    delay(MOTOR_PULSE_DURATION);
    digitalWrite(BASE_MOTOR_PIN, LOW);
  } 
  else {
    // pH in acceptable range
    digitalWrite(ACID_MOTOR_PIN, LOW);
    digitalWrite(BASE_MOTOR_PIN, LOW);
  }
}

/**
 * Main sensor reading and control function
 * Called every 5 seconds by Blynk timer
 */
void sendSensorData() {
  // ========== Temperature Reading ==========
  tempSensor.requestTemperatures();
  float tempC = tempSensor.getTempCByIndex(0);

  // ========== Turbidity Reading ==========
  int turbRaw = readAverageAnalog(TURBIDITY_PIN);
  int turbidityPercent = mapToPercent(turbRaw);

  // ========== pH Reading ==========
  int phRawAvg = readAverageAnalog(PH_PIN, 100);
  float phVoltage = phRawAvg * 5.0 / 1023.0;
  float pH = calculatePH(phVoltage);

  // ========== Send Data to Blynk Cloud ==========
  Blynk.virtualWrite(V0, tempC);                // Virtual Pin V0: Temperature
  Blynk.virtualWrite(V1, pH);                   // Virtual Pin V1: pH
  Blynk.virtualWrite(V2, turbidityPercent);     // Virtual Pin V2: Turbidity

  // ========== Automated Control ==========
  controlTemperature(tempC);
  correctPH(pH);

  // ========== Alert Notifications ==========
  if (tempC != DEVICE_DISCONNECTED_C && tempC > TEMP_HIGH) {
    Blynk.logEvent("temperature_alert", "⚠️ High water temperature!");
  }
  
  if (turbidityPercent > TURBIDITY_THRESHOLD) {
    Blynk.logEvent("ntu_alert", "⚠️ High turbidity level!");
  }
  
  if (pH < PH_CRITICAL_LOW || pH > PH_CRITICAL_HIGH) {
    Blynk.logEvent("ph_limit", "⚠️ Abnormal pH detected!");
  }

  // ========== Serial Monitor Output ==========
  Serial.print(F("Temp: "));
  Serial.print(tempC == DEVICE_DISCONNECTED_C ? NAN : tempC, 2);
  Serial.print(F(" °C | Turbidity: "));
  Serial.print(turbRaw);
  Serial.print(F(" ("));
  Serial.print(turbidityPercent);
  Serial.print(F("%) | pH: "));
  Serial.print(pH, 2);
  Serial.print(F(" ["));
  Serial.print(phVoltage, 3);
  Serial.println(F(" V]"));
}

// ==================== Arduino Setup ====================
void setup() {
  // Initialize serial communication
  Serial.begin(38400);
  EspSerial.begin(38400);
  
  // Connect to Blynk cloud
  Blynk.begin(BLYNK_AUTH_TOKEN, wifi, ssid, pass);
  Serial.println(F("System Initialized"));

  // Initialize temperature sensor
  tempSensor.begin();
  if (!tempSensor.getDeviceCount()) {
    Serial.println(F("⚠️ No DS18B20 detected"));
  }

  // Configure actuator pins
  pinMode(HEATER_PIN, OUTPUT);
  pinMode(COOLER_PIN, OUTPUT);
  pinMode(HEATER_RELAY, OUTPUT);
  pinMode(COOLER_RELAY, OUTPUT);
  pinMode(ACID_MOTOR_PIN, OUTPUT);
  pinMode(BASE_MOTOR_PIN, OUTPUT);

  // Set initial states (all off)
  digitalWrite(HEATER_PIN, LOW);
  digitalWrite(COOLER_PIN, LOW);
  digitalWrite(HEATER_RELAY, HIGH);      // Active LOW relay
  digitalWrite(COOLER_RELAY, HIGH);      // Active LOW relay
  digitalWrite(ACID_MOTOR_PIN, LOW);
  digitalWrite(BASE_MOTOR_PIN, LOW);

  // Schedule sensor reading every 5 seconds
  timer.setInterval(5000L, sendSensorData);
}

// ==================== Arduino Main Loop ====================
void loop() {
  Blynk.run();
  timer.run();
}
