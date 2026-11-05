#include <Wire.h>
#include "MAX30105.h"
#include "heartRate.h"

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// PINES (ESP32-C6-MINI-1)

#define SDA_PIN          5
#define SCL_PIN          4
#define LOAD_SWITCH_PIN  20
#define POWER_BUTTON_PIN 2
#define LED_PIN          22

// DIRECCIONES Y REGISTROS I2C

#define RV3028_ADDR     0x52
#define MAX17048_ADDR   0x36
#define REG_VCELL       0x02
#define REG_SOC         0x04
#define LSM6DSO32_ADDR  0x6A
#define CTRL1_XL        0x10
#define OUTX_L_A        0x28

// UUIDs BLE

#define SERVICE_UUID  "12345678-1234-1234-1234-123456789000"
#define BPM_UUID      "12345678-1234-1234-1234-123456789001"
#define EVENT_UUID    "12345678-1234-1234-1234-123456789002"
#define RTC_UUID      "12345678-1234-1234-1234-123456789003"
#define BATTERY_UUID  "12345678-1234-1234-1234-123456789004"

bool bleConnected = false;

// LED INDICADOR

enum LedMode {
  LED_SEARCHING,
  LED_CONNECTED,
  LED_EVENT
};

LedMode  currentLedMode  = LED_SEARCHING;
uint32_t ledLastToggle   = 0;
bool     ledState        = false;
uint32_t eventLedEndTime = 0;

void updateLED()
{
  uint32_t now = millis();

  LedMode mode;
  if (eventLedEndTime != 0 && (int32_t)(eventLedEndTime - now) > 0)
    mode = LED_EVENT;
  else
  {
    eventLedEndTime = 0;
    mode = bleConnected ? LED_CONNECTED : LED_SEARCHING;
  }

  bool target = false;
  switch (mode)
  {
    case LED_SEARCHING: target = (now % 1000) < 100; break;
    case LED_CONNECTED: target = (now % 3000) < 30;  break;
    case LED_EVENT:     target = (now % 300)  < 150; break;
  }

  if (target != ledState)
  {
    ledState = target;
    digitalWrite(LED_PIN, target ? HIGH : LOW);
  }
}

// BOTÓN DE ENCENDIDO/APAGADO

void checkPowerButton()
{
  static uint32_t pressedTime = 0;

  if (digitalRead(POWER_BUTTON_PIN) == LOW)
  {
    if (pressedTime == 0) pressedTime = millis();

    if (millis() - pressedTime > 3000)
    {
      Serial.println("[SYS] Apagado por botón");
      digitalWrite(LED_PIN, LOW);
      digitalWrite(LOAD_SWITCH_PIN, LOW);
      while (1);
    }
  }
  else
  {
    pressedTime = 0;
  }
}

// RTC (RV3028)

BLECharacteristic *rtcCharacteristic;

uint8_t bcdToDec(uint8_t val)
{
  return ((val >> 4) * 10) + (val & 0x0F);
}

uint8_t decToBcd(uint8_t val)
{
  return ((val / 10) << 4) | (val % 10);
}

void writeRegister(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(RV3028_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

uint8_t readRegister(uint8_t reg)
{
  Wire.beginTransmission(RV3028_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(RV3028_ADDR, 1);
  if (Wire.available()) return Wire.read();
  return 0;
}

String getRTCString()
{
  uint8_t sec  = bcdToDec(readRegister(0x00));
  uint8_t min  = bcdToDec(readRegister(0x01));
  uint8_t hour = bcdToDec(readRegister(0x02));
  char buffer[16];
  sprintf(buffer, "%02d:%02d:%02d", hour, min, sec);
  return String(buffer);
}

class RTCCallback : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    String value = pCharacteristic->getValue().c_str();
    Serial.print("[RTC] Sincronización recibida: ");
    Serial.println(value);

    int hh, mm, ss;
    if (sscanf(value.c_str(), "%d:%d:%d", &hh, &mm, &ss) == 3)
    {
      writeRegister(0x00, decToBcd(ss));
      writeRegister(0x01, decToBcd(mm));
      writeRegister(0x02, decToBcd(hh));
      Serial.println("[RTC] Hora actualizada correctamente");
    }
    else
    {
      Serial.println("[RTC] Formato inválido — se esperaba HH:MM:SS");
    }
  }
};

void updateRTCBLE()
{
  static uint32_t lastRTC = 0;

  if (millis() - lastRTC > 1000)
  {
    lastRTC = millis();

    String rtcString = getRTCString();
    rtcCharacteristic->setValue(rtcString.c_str());
    rtcCharacteristic->notify();
  }
}

// BATERÍA (MAX17048)

BLECharacteristic *batteryCharacteristic;

uint16_t readRegister16(uint8_t reg)
{
  Wire.beginTransmission(MAX17048_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return 0xFFFF;
  Wire.requestFrom(MAX17048_ADDR, 2);
  if (Wire.available() < 2) return 0xFFFF;
  uint8_t msb = Wire.read();
  uint8_t lsb = Wire.read();
  return ((uint16_t)msb << 8) | lsb;
}

float getBatterySOC()
{
  uint16_t rawSoc = readRegister16(REG_SOC);
  if (rawSoc == 0xFFFF) return -1;
  return (rawSoc >> 8) + ((rawSoc & 0xFF) / 256.0);
}

float getBatteryVoltage()
{
  uint16_t rawVcell = readRegister16(REG_VCELL);
  if (rawVcell == 0xFFFF) return -1;
  return (rawVcell >> 4) * 0.00125;
}

void updateBatteryBLE()
{
  static uint32_t lastBattery = 0;

  if (millis() - lastBattery > 10000)
  {
    lastBattery = millis();

    float voltage = getBatteryVoltage();
    float soc     = getBatterySOC();

    char buffer[32];
    sprintf(buffer, "%.2fV %.1f%%", voltage, soc);

    batteryCharacteristic->setValue(buffer);
    batteryCharacteristic->notify();

    Serial.print("[BAT] ");
    Serial.print(buffer);

    if (soc < 10.0)
      Serial.println("  (batería extremadamente baja)");
    else if (soc < 20.0)
      Serial.println("  (batería baja)");
    else
      Serial.println();
  }
}

// SENSOR PPG (MAX30101) Y CÁLCULO DE BPM

MAX30105 particleSensor;

const byte RATE_SIZE = 4;
byte  rates[RATE_SIZE];
byte  rateSpot       = 0;
long  lastBeat       = 0;
float beatsPerMinute = 0;
int   beatAvg        = 0;

void setupSensor()
{
  Serial.println("[PPG] Inicializando MAX30101...");

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {
    Serial.println("[PPG] ERROR — MAX30101 no encontrado. Verifica I2C y alimentación.");
    while (1)
    {
      digitalWrite(LED_PIN, HIGH); delay(100);
      digitalWrite(LED_PIN, LOW);  delay(100);
    }
  }

  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0);
  particleSensor.setPulseAmplitudeGreen(255);
  particleSensor.setPulseAmplitudeIR(0);

  Serial.println("[PPG] MAX30101 OK");
  Serial.println("[PPG] Canal activo: Verde (537 nm)");
}

void updateBPM()
{
  long greenValue = particleSensor.getGreen();

  if (checkForBeat(greenValue))
  {
    long delta = millis() - lastBeat;
    lastBeat = millis();

    beatsPerMinute = 60.0 / (delta / 1000.0);

    if (beatsPerMinute > 20 && beatsPerMinute < 255)
    {
      rates[rateSpot++] = (byte)beatsPerMinute;
      rateSpot %= RATE_SIZE;

      beatAvg = 0;
      for (byte i = 0; i < RATE_SIZE; i++)
        beatAvg += rates[i];
      beatAvg /= RATE_SIZE;
    }
  }
}

// BUFFER CIRCULAR

#define CIRCULAR_BUFFER_SIZE 20

int     circBuffer[CIRCULAR_BUFFER_SIZE];
uint8_t circHead  = 0;
uint8_t circCount = 0;

void circBufferPush(int bpm)
{
  circBuffer[circHead] = bpm;
  circHead = (circHead + 1) % CIRCULAR_BUFFER_SIZE;
  if (circCount < CIRCULAR_BUFFER_SIZE)
    circCount++;
}

bool circBufferFull()
{
  return circCount == CIRCULAR_BUFFER_SIZE;
}

// TRANSMISIÓN BLE DE BPM Y EVENTOS

BLECharacteristic *bpmCharacteristic;
BLECharacteristic *eventCharacteristic;

void sendBLEData(int bpm, String eventType)
{
  if (!bleConnected) return;

  bpmCharacteristic->setValue(String(bpm).c_str());
  bpmCharacteristic->notify();

  eventCharacteristic->setValue(eventType.c_str());
  eventCharacteristic->notify();

  Serial.print("[BLE TX] BPM=");
  Serial.print(bpm);
  Serial.print("  EVENT=");
  Serial.println(eventType);
}

// Transmisión a intervalo fijo de 1 segundo.
void sendPeriodicBLE()
{
  static uint32_t lastBLESend = 0;

  if (millis() - lastBLESend >= 1000)
  {
    lastBLESend = millis();

    if (bleConnected && beatAvg > 0)
    {
      circBufferPush(beatAvg);

      String eventType = "NORMAL";

      if (beatAvg > 100)
      {
        eventType = "HIGH";
      }
      else if (beatAvg < 50)
      {
        eventType = "LOW";
      }

      if (eventType != "NORMAL")
      {
        eventLedEndTime = millis() + 3000;

        Serial.print("[PPG] Evento: ");
        Serial.print(eventType);
        Serial.print(" — BPM: ");
        Serial.print(beatAvg);

        if (!circBufferFull())
        {
          Serial.print(" (llenando buffer: ");
          Serial.print(circCount);
          Serial.print("/20)");
        }
        Serial.println();
      }

      sendBLEData(beatAvg, eventType);
    }
  }
}

// IMU (LSM6DSO32): IMPACTO / CAÍDA

bool     impactDetected = false;
uint32_t impactTime     = 0;

void imuWrite(uint8_t reg, uint8_t value)
{
  Wire.beginTransmission(LSM6DSO32_ADDR);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

int16_t imuRead16(uint8_t reg)
{
  Wire.beginTransmission(LSM6DSO32_ADDR);
  Wire.write(reg);
  Wire.endTransmission(false);
  Wire.requestFrom(LSM6DSO32_ADDR, 2);
  if (Wire.available() < 2) return 0;
  uint8_t low  = Wire.read();
  uint8_t high = Wire.read();
  return (int16_t)(high << 8 | low);
}

void setupIMU()
{
  Serial.println("[IMU] Inicializando LSM6DSO...");
  imuWrite(CTRL1_XL, 0x42);
  Serial.println("[IMU] LSM6DSO OK — Acelerómetro habilitado (104 Hz, ±4g)");
}

void checkIMU()
{
  static uint32_t lastIMU = 0;
  if (millis() - lastIMU < 50) return;
  lastIMU = millis();

  int16_t axRaw = imuRead16(OUTX_L_A);
  int16_t ayRaw = imuRead16(OUTX_L_A + 2);
  int16_t azRaw = imuRead16(OUTX_L_A + 4);

  float ax = axRaw * 0.000122f;
  float ay = ayRaw * 0.000122f;
  float az = azRaw * 0.000122f;

  float accMagnitude = sqrt(ax*ax + ay*ay + az*az);

  if (accMagnitude > 2.5f)
  {
    impactDetected = true;
    impactTime = millis();

    Serial.print("[IMU] Impacto detectado — magnitud: ");
    Serial.print(accMagnitude, 2);
    Serial.println(" g");

    eventLedEndTime = millis() + 3000;

    sendBLEData(beatAvg, "MOVEMENT");
  }

  if (impactDetected)
  {
    if (millis() - impactTime > 3000)
    {
      if (accMagnitude < 1.1f)
      {
        Serial.println("[IMU] Posible caída detectada (impacto + inmovilidad)");

        eventLedEndTime = millis() + 3000;

        sendBLEData(beatAvg, "FALL");
      }
      impactDetected = false;
    }
  }
}

// SERVIDOR BLE

class ServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer* pServer)
  {
    bleConnected = true;
    currentLedMode = LED_CONNECTED;
    Serial.println("[BLE] Cliente conectado");
  }

  void onDisconnect(BLEServer* pServer)
  {
    bleConnected = false;
    currentLedMode = LED_SEARCHING;
    Serial.println("[BLE] Cliente desconectado — reanunciando...");
    BLEDevice::startAdvertising();
  }
};

void setupBLE()
{
  Serial.println("[BLE] Inicializando...");

  BLEDevice::init("CardioWatch-WEARABLE");

  BLEServer *server = BLEDevice::createServer();
  server->setCallbacks(new ServerCallbacks());

  BLEService *service = server->createService(SERVICE_UUID);

  bpmCharacteristic = service->createCharacteristic(
    BPM_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY);

  eventCharacteristic = service->createCharacteristic(
    EVENT_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY);

  rtcCharacteristic = service->createCharacteristic(
    RTC_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_WRITE |
    BLECharacteristic::PROPERTY_NOTIFY);

  batteryCharacteristic = service->createCharacteristic(
    BATTERY_UUID,
    BLECharacteristic::PROPERTY_READ |
    BLECharacteristic::PROPERTY_NOTIFY);

  bpmCharacteristic->addDescriptor(new BLE2902());
  eventCharacteristic->addDescriptor(new BLE2902());
  rtcCharacteristic->addDescriptor(new BLE2902());
  rtcCharacteristic->setCallbacks(new RTCCallback());
  batteryCharacteristic->addDescriptor(new BLE2902());

  service->start();

  BLEAdvertising *advertising = BLEDevice::getAdvertising();
  advertising->addServiceUUID(SERVICE_UUID);
  advertising->start();

  Serial.println("[BLE] Listo — anunciando como CardioWatch-WEARABLE");
  Serial.print("[BLE] Service UUID: ");
  Serial.println(SERVICE_UUID);
}

void setup()
{
  Serial.begin(115200);
  delay(1000);

  Serial.println("========================================");
  Serial.println("MAX30101_Wristband");
  Serial.println("Firmware v1.0");
  Serial.println("========================================");
  Serial.println();

  // LED indicador.
  Serial.println("[SYS] Configurando LED indicador (GPIO22)...");
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  // Parpadeo de prueba al arrancar.
  for (int i = 0; i < 3; i++) {
    digitalWrite(LED_PIN, HIGH); delay(80);
    digitalWrite(LED_PIN, LOW);  delay(80);
  }
  Serial.println("[SYS] LED OK");

  // Power switch.
  Serial.println("[SYS] Habilitando power switch (GPIO20)...");
  pinMode(LOAD_SWITCH_PIN, OUTPUT);
  digitalWrite(LOAD_SWITCH_PIN, HIGH);
  Serial.println("[SYS] Power switch ON");

  // Botón de apagado.
  pinMode(POWER_BUTTON_PIN, INPUT_PULLUP);
  Serial.println("[SYS] Botón de apagado configurado (GPIO2, mantener 3s)");

  // I2C.
  Serial.println("[I2C] Iniciando bus (SDA=GPIO5, SCL=GPIO4, 100 kHz)...");
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Serial.println("[I2C] Bus OK");

  // Subsistemas.
  setupSensor();
  setupIMU();
  setupBLE();

  // Estado inicial del buffer.
  memset(circBuffer, 0, sizeof(circBuffer));
  Serial.println("[BUF] Buffer circular inicializado (20 muestras)");

  Serial.println();
  Serial.println("[SYS] Inicialización completada");
  Serial.println("[SYS] LED parpadeando lento = buscando cliente BLE...");
  Serial.println("========================================");
  Serial.println();
}

void loop()
{
  checkPowerButton();
  checkIMU();
  updateRTCBLE();
  updateBatteryBLE();
  updateLED();

  updateBPM();
  sendPeriodicBLE();
}
