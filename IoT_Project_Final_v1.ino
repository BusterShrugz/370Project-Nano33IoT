#include <SPI.h>
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <Arduino_LSM6DS3.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include "secretNets.h"
#include <MAX30105.h>
#include <heartRate.h>
#include "spo2_algorithm.h"

// WiFi fallback list
const char *wifiSSIDs[] = {ssid, ssidTwo, ssidHotSpot};
const char *wifiPASSes[] = {pass, passTwo, passHotSpot};
const int wifiCount = sizeof(wifiSSIDs) / sizeof(wifiSSIDs[0]);

// UDP setup
IPAddress piAddress(10, 0, 0, 120); // Raspberry Pi IP OLD?
unsigned int udpPort = 5005;        // Ethan's port

// IPAddress piAddress(192, 168, 0, 23); // Reese's house Raspberry Pi IP
// unsigned int udpPort = 3333;          // Reese's port
WiFiUDP Udp;

// Sensors
#define LED_PIN 2
#define ADC_TEMPERATURE 0x18 // ADC channel for SAMD21 internal temp
Adafruit_MLX90614 mlx;       // Standard MLX90614 I2C address
MAX30105 particleSensor;     // Standard MAX30105 I2C address

// Max Sensor data
#define MAX30105_ADDRESS 0x57
#define MAX30105_REG_MODE_CONFIG 0x09
#define MAX_BUFFER 200
uint32_t irBuffer[MAX_BUFFER];
uint32_t redBuffer[MAX_BUFFER];
int maxSensorBufferIndex = 0; // next slot
int maxSensorSampleCount = 0; // how many valid samples stored (MAX_BUFFER)

// variables for smoothing data
#define HR_SMOOTH_SIZE 5
#define SPO2_SMOOTH_SIZE 5
#define BPM_SMOOTHING_SIZE 5
                              // Long-window BPM smoothing (5–10s)
#define LONG_BPM_SECONDS 10 // average over last 10 seconds
#define MAX_BPM_SAMPLES 20  // enough for ~20 beats (2Hz worst case)

float longBpmBuffer[MAX_BPM_SAMPLES];
unsigned long longBpmTime[MAX_BPM_SAMPLES];
int longBpmIndex = 0;
long hrSmoothBuffer[HR_SMOOTH_SIZE] = {0};
int hrSmoothIndex = 0;
long lastStableHR = 0;
long spo2SmoothBuffer[SPO2_SMOOTH_SIZE] = {0};
int spo2SmoothIndex = 0;
long lastStableSpO2 = 0;
float bpmHistory[BPM_SMOOTHING_SIZE] = {0};
int bpmHistIndex = 0;

// MAX sensor output variables
int32_t spo2 = 0;
int8_t validSPO2 = 0;
int32_t heartRate = 0;
int8_t validHeartRate = 0;

// Last measured values (made global so printSerial() can use them)
float ambientC = 0;
float objectC = 0;
float cpuTemp = 0;
unsigned long lastSendTime = 0;

// ECG
const int ecgPin = A0;     // AD8232 analog output
const int LOplusPin = 10;  // Lead-Off + pin
const int LOminusPin = 11; // Lead-Off - pin
#define ECG_BATCH_SIZE 50
float ecgBatch[ECG_BATCH_SIZE];
int ecgIndex = 0;
bool ecgValid = true;
const float voltageRef = 3.3;

// Fast BPM (quick beat detect)
float bpm = 0.0;
static uint32_t lastBeat = 0;

// Timing
unsigned long lastBlink = 0;
bool ledState = false;
unsigned long lastSensorUpdate = 0;
const unsigned long sensorInterval = 1000;
unsigned long lastECGSample = 0; // for 500Hz sampling
unsigned long lastUDPSend = 0;   // for 100ms batching
const unsigned long ecgInterval = 2000;

// Helper Prototypes
void connectToBestWiFi();
long smoothHR(long newHR);
long smoothSpO2(long newSpO2);
float smoothBPM(float newBPM);
void updateStatusLED();

void setup()
{
  Serial.begin(115200);
  while (!Serial && millis() < 2000)
    ;

  pinMode(LED_PIN, OUTPUT);
  pinMode(ecgPin, INPUT);
  pinMode(LOplusPin, INPUT);
  pinMode(LOminusPin, INPUT);
  digitalWrite(LED_PIN, HIGH); // Solid ON during startup
  analogReadResolution(12);    // 12-bit resolution for AD8232

  // Connect to the best WiFi option
  connectToBestWiFi();

  // Start UDP
  Udp.begin(udpPort);
  Serial.print("UDP listening on port ");
  Serial.println(udpPort);

  // Initialize MLX90614
  Wire.begin();
  delay(500); // Give the sensor time to wake up
  if (!mlx.begin(0x5A))
  {
    Serial.println("ERROR: Could not find MLX90614 sensor at 0x5A!");
    Serial.println("Check wiring and voltage (use 3.3V on Nano 33 IoT).");
    while (1)
      ;
  }
  Serial.println("MLX90614 sensor initialized successfully.");

  // Initialize MAX30105
  Serial.println("Initializing MAX30105...");
  if (!particleSensor.begin(Wire))
  {
    Serial.println("ERROR: MAX30100 not detected!");
    Serial.println("Check SDA/SCL wiring and power (must be 3.3V).");
    while (1)
      ;
  }

  Serial.println("Configuring MAX30105...");

  particleSensor.setup();                    // Base config
  particleSensor.setLEDMode(2);              // 2 = Red + IR
  particleSensor.setPulseAmplitudeRed(0x1F); // moderate currents
  particleSensor.setPulseAmplitudeIR(0x1F);
  particleSensor.setPulseWidth(411);
  particleSensor.setSampleRate(50); // 50Hz
  particleSensor.setADCRange(16384);
  Serial.println("MAX30105 initialized");

  // Initialize buffers and ECG data
  for (int i = 0; i < ECG_BATCH_SIZE; i++)
    ecgBatch[i] = 0.0;
  maxSensorBufferIndex = 0;
  maxSensorSampleCount = 0;
}

void loop()
{
  unsigned long now = millis();
  unsigned long nowMicros = micros();
  static unsigned long lastMaxRead = 0;
  updateStatusLED();

  // MAX31005 sensor read data continuous
  if (millis() - lastMaxRead >= 20)
  { // 50hz
    lastMaxRead = now;
    particleSensor.check();
    uint32_t ir = particleSensor.getIR();
    uint32_t red = particleSensor.getRed();

    // Store samples for circle buffer
    irBuffer[maxSensorBufferIndex] = ir;
    redBuffer[maxSensorBufferIndex] = red;
    maxSensorBufferIndex = (maxSensorBufferIndex + 1) % MAX_BUFFER;
    if (maxSensorSampleCount < MAX_BUFFER)
      maxSensorSampleCount++;

    // smoothing the ir for BPM
    int idx1 = (maxSensorBufferIndex - 1 + MAX_BUFFER) % MAX_BUFFER;
    int idx2 = (maxSensorBufferIndex - 2 + MAX_BUFFER) % MAX_BUFFER;
    int idx3 = (maxSensorBufferIndex - 3 + MAX_BUFFER) % MAX_BUFFER;
    long smoothIR = (irBuffer[idx1] + irBuffer[idx2] + irBuffer[idx3]) / 3;

    // bpm using checkForBeat()
    long scaledIR = (smoothIR >> 2); // scale down
    if (scaledIR > 50000)
      scaledIR = 50000;
    if (checkForBeat(scaledIR))
    {
      uint32_t nowBeat = now;
      uint32_t diff = nowBeat - lastBeat;
      lastBeat = nowBeat;

      float rawBPM = 60.0 / (diff / 1000.0);
      if (rawBPM < 20 || rawBPM > 220)
        rawBPM = 0.0;

      float shortSmooth = smoothBPM(rawBPM);            // your 5-sample smoother
      float longSmooth = getLongWindowBPM(shortSmooth); // 5–10 sec avg

      bpm = longSmooth;
    }

    particleSensor.nextSample();

    // MAX31000 sensor read data 100 samples ~ 1 seccond
    if (maxSensorSampleCount >= MAX_BUFFER)
    {
      maxim_heart_rate_and_oxygen_saturation(
          irBuffer,
          MAX_BUFFER,
          redBuffer,
          &spo2,
          &validSPO2,
          &heartRate,
          &validHeartRate);
      maxSensorSampleCount = 0;
    }
  }

  // ECG sampling 500hz
  if (nowMicros - lastECGSample >= 2000) // 2ms interval
  {
    lastECGSample = nowMicros;
    float ecgVoltage = analogRead(ecgPin) * (voltageRef / 4095.0);
    float baseline = voltageRef / 2.0;
    float ecg_mV = (ecgVoltage - baseline) * 1000; // Converts AD8232 Voltage to milliVo

    // Clamp for spikes
    if (ecg_mV > 2000)
      ecg_mV = 2000;
    if (ecg_mV < -2000)
      ecg_mV = -2000;
    ecgBatch[ecgIndex++] = ecg_mV;

    if (ecgIndex >= ECG_BATCH_SIZE)
      ecgIndex = 0; // Circular buffer

    // Lead-off detection
    if (digitalRead(LOplusPin) || digitalRead(LOminusPin))
    {
      ecgValid = false;
    }
    else
    {
      ecgValid = true;
    }
  }

  // update MLX and CPU temps
  ambientC = mlx.readAmbientTempC();
  objectC = mlx.readObjectTempC();

  // Send UDP batch every 100ms
  if (now - lastUDPSend >= 100)
  {
    lastUDPSend = now;

    // ECG average
    float avgECGVoltage = NAN;
    if (ecgValid)
    {
      float sumECG = 0.0;
      for (int i = 0; i < ECG_BATCH_SIZE; i++)
        sumECG += ecgBatch[i];
      avgECGVoltage = sumECG / ECG_BATCH_SIZE;
    }

    // Average for IR and REd UDP packet sending
    uint64_t sumIR = 0;
    uint64_t sumRed = 0;
    for (int i = 0; i < MAX_BUFFER; i++)
    {
      sumIR += irBuffer[i];
      sumRed += redBuffer[i];
    }

    uint32_t avgIR = sumIR / MAX_BUFFER;
    uint32_t avgRed = sumRed / MAX_BUFFER;

    // Only report stable HR if valid
    long stableSpO2 = validSPO2 ? smoothSpO2(spo2) : lastStableSpO2;

    // Only report Max30105 data if finger is detected
    if (!fingerPresent())
    {
      validHeartRate = 0;
      validSPO2 = 0;
      stableSpO2 = 0;
      bpm = 0;
      spo2 = 0;
    }

    // Create UDP message
    char packet[700];
    int offset = 0;

    // ECG batch data
    for (int i = 0; i < ECG_BATCH_SIZE; i++)
    {
      offset += snprintf(packet + offset, sizeof(packet) - offset, "%.3f,", ecgBatch[i]);
    }

    char avgECGBuffer[16];
    const char *avgECGString;

    if (isnan(avgECGVoltage))
    {
      avgECGString = "nan";
    }
    else
    {
      snprintf(avgECGBuffer, sizeof(avgECGBuffer), "%.2f", avgECGVoltage);
      avgECGString = avgECGBuffer;
    }

    // Temps, avgECG, BPM avgIR/Red and Sp02
    offset += snprintf(packet + offset, sizeof(packet) - offset,
                       "amb:%.2f,obj:%.2f,avgECG:%s,BPM:%.2f,IR:%lu,RED:%lu,validHR:%d,SpO2:%ld,validSPO2:%d",
                       ambientC, objectC, avgECGString,
                       bpm, (unsigned long)avgIR, (unsigned long)avgRed,
                       validHeartRate, stableSpO2, validSPO2);

    // Sending packet
    Udp.beginPacket(piAddress, udpPort);
    Udp.write(packet);
    Udp.endPacket();

    // Check for bad readings (NaN)
    if (isnan(ambientC) || isnan(objectC))
    {
      Serial.println("WARNING: MLX90614 returned NaN values — check wiring or address!");
    }

    // Serial output
    Serial.println("\n===== Sending UDP packet =====");
    Serial.print("MLX Ambient: ");
    Serial.println(ambientC, 2);
    Serial.print("Body Temp  : ");
    Serial.println(objectC, 2);

    Serial.print("Avg. ECG   : ");
    Serial.println(avgECGVoltage);

    Serial.print("BPM        : ");
    Serial.println(bpm);
    Serial.print("BPM Valid  : ");
    Serial.println(validHeartRate);

    Serial.print("Stable SpO2: ");
    Serial.println(stableSpO2);
    Serial.print("SpO2 Valid : ");
    Serial.println(validSPO2);

    // Print last raw values (slot 99)
    Serial.print("IR Avg.    : ");
    Serial.println(avgIR);
    Serial.print("RED Avg.   : ");
    Serial.println(avgRed);
    Serial.println("===========================\n");
  }
}

// Helper functions for smoothing data
long smoothHR(long newHR)
{
  if (newHR <= 0)
    newHR = lastStableHR; // Ignore invalid/zero
  hrSmoothBuffer[hrSmoothIndex] = newHR;
  hrSmoothIndex = (hrSmoothIndex + 1) % HR_SMOOTH_SIZE;

  long sum = 0;
  int count = 0;
  for (int i = 0; i < HR_SMOOTH_SIZE; i++)
  {
    if (hrSmoothBuffer[i] > 0)
    {
      sum += hrSmoothBuffer[i];
      count++;
    }
  }
  long smoothed = count ? sum / count : lastStableHR;
  lastStableHR = smoothed;
  return smoothed;
}

float getLongWindowBPM(float newBPM)
{
  unsigned long now = millis();

  // Ignore zeros (invalid beats)
  if (newBPM > 0)
  {
    longBpmBuffer[longBpmIndex] = newBPM;
    longBpmTime[longBpmIndex] = now;
    longBpmIndex = (longBpmIndex + 1) % MAX_BPM_SAMPLES;
  }

  // Compute 5-10 second rolling average
  float sum = 0;
  int count = 0;
  for (int i = 0; i < MAX_BPM_SAMPLES; i++)
  {
    if (now - longBpmTime[i] <= (LONG_BPM_SECONDS * 1000UL))
    {
      if (longBpmBuffer[i] > 0)
      {
        sum += longBpmBuffer[i];
        count += 1;
      }
    }
  }

  return (count > 0) ? (sum / count) : 0;
}

long smoothSpO2(long newSpO2)
{
  if (newSpO2 <= 0)
    newSpO2 = lastStableSpO2; // Ignore invalid/zero
  spo2SmoothBuffer[spo2SmoothIndex] = newSpO2;
  spo2SmoothIndex = (spo2SmoothIndex + 1) % SPO2_SMOOTH_SIZE;

  long sum = 0;
  int count = 0;
  for (int i = 0; i < SPO2_SMOOTH_SIZE; i++)
  {
    if (spo2SmoothBuffer[i] > 0)
    {
      sum += spo2SmoothBuffer[i];
      count++;
    }
  }
  long smoothed = count ? sum / count : lastStableSpO2;
  lastStableSpO2 = smoothed;
  return smoothed;
}

float smoothBPM(float newBPM)
{
  bpmHistory[bpmHistIndex] = newBPM;
  bpmHistIndex = (bpmHistIndex + 1) % BPM_SMOOTHING_SIZE;

  float sum = 0;
  int count = 0;
  for (int i = 0; i < BPM_SMOOTHING_SIZE; i++)
  {
    if (bpmHistory[i] > 0)
    { // Ignore zero/no-beat values
      sum += bpmHistory[i];
      count++;
    }
  }
  return (count > 0) ? (sum / count) : 0;
}
// Helper functions for WiFi and LED blink
void updateStatusLED()
{
  // Wifi connected slow blink
  if (WiFi.status() == WL_CONNECTED)
  {
    if (millis() - lastBlink >= 1000)
    {
      lastBlink = millis();
      ledState = !ledState;
      digitalWrite(LED_PIN, ledState);
    }
  }
  // Disconnected from wifi error flashing
  else
  {
    unsigned long t = millis() % 2000; // 2 seconds cycle
    if ((t < 100) || (t > 300 && t < 400))
    {
      digitalWrite(LED_PIN, HIGH);
    }
    else
    {
      digitalWrite(LED_PIN, LOW);
    }
  }
}

void connectToBestWiFi()
{
  Serial.println("\nIoT Vital Monitor Starting... \nPlease wait...");

  for (int i = 0; i < wifiCount; i++)
  {
    // Skip the hotspot until we set it up
    if (strlen(wifiSSIDs[i]) == 0)
      continue;

    Serial.print("Testing available WiFi: ");
    Serial.println(wifiSSIDs[i]);

    WiFi.begin(wifiSSIDs[i], wifiPASSes[i]);

    unsigned long startAttempt = millis();
    const unsigned long timeout = 8000; // Waiting 8 seconds and if none available switching to next wifi

    while (WiFi.status() != WL_CONNECTED &&
           millis() - startAttempt < timeout)
    {
      Serial.print(".");
      delay(500);
    }
    if (WiFi.status() == WL_CONNECTED)
    {
      Serial.println("\n ^(0.0)> Connected Successfully! <(0_0)^");
      Serial.print("Connected to SSID: ");
      Serial.println(wifiSSIDs[i]);
      Serial.print("IP Address: ");
      Serial.println(WiFi.localIP());
      return;
    }
    Serial.println("\nFailed - Attempting to connect to next WiFi option. ");
  }
  Serial.println("ERROR!! Could not connect to any network via WiFi.");
}

bool fingerPresent()
{
  uint32_t irValue = particleSensor.getIR();
  return (irValue > 20000 && irValue < 100000); // typical finger range
}