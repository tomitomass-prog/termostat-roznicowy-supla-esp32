/*
 * Termostat roznicowy ON/OFF - ESP32 + SUPLA
 *
 * Zastosowanie: ochrona bojlera przed wychladzaniem przez chlodniejszy czynnik.
 *
 * AUTO:
 *   - brak/blad konfiguracji adresow -> OFF (nie uruchamiaj niezaprogramowanego urzadzenia)
 *   - uszkodzenie / brak odczytu jednego z czujnikow -> FAILSAFE (domyslnie ON)
 *   - T2 >= Tmax -> OFF
 *   - T1 < Tmin -> OFF
 *   - T1 >= T2 + DeltaON -> ON
 *   - T1 <= T2 + DeltaOFF -> OFF
 *   - pomiedzy progami stan jest podtrzymywany
 *
 * RECZNY:
 *   - AUTO = OFF, kanal RECZNY bezposrednio steruje wyjsciem
 *
 * Sprzet referencyjny:
 *   GPIO4  - 1-Wire, 2 x DS18B20, pull-up ok. 4.7 kOhm do 3.3 V
 *   GPIO21 - OLED SDA
 *   GPIO22 - OLED SCL
 *   GPIO26 - wyjscie przekaznika
 *   GPIO27 - przycisk OLED / lokalna zmiana trybu -> GND
 *   GPIO0  - przycisk konfiguracji SUPLA (BOOT)
 *
 * UWAGA: GPIO ESP32 nie moze bezposrednio zasilac cewki przekaznika.
 * Uzyj modulu przekaznikowego lub tranzystora z dioda zabezpieczajaca.
 */

#include <Arduino.h>
#include <cmath>
#include <cstring>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <WiFi.h>
#include <Wire.h>

#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#include <SuplaDevice.h>
#include <supla/control/button.h>
#include <supla/control/hvac_base.h>
#include <supla/control/virtual_relay.h>
#include <supla/network/esp_web_server.h>
#include <supla/network/esp_wifi.h>
#include <supla/network/html/custom_parameter.h>
#include <supla/network/html/custom_text_parameter.h>
#include <supla/network/html/device_info.h>
#include <supla/network/html/hvac_parameters.h>
#include <supla/network/html/protocol_parameters.h>
#include <supla/network/html/wifi_parameters.h>
#include <supla/sensor/general_purpose_measurement.h>
#include <supla/sensor/virtual_thermometer.h>
#include <supla/storage/eeprom.h>
#include <supla/storage/littlefs_config.h>

namespace {

constexpr uint8_t PIN_ONEWIRE = 4;
constexpr uint8_t PIN_I2C_SDA = 21;
constexpr uint8_t PIN_I2C_SCL = 22;
constexpr uint8_t PIN_RELAY = 26;
constexpr uint8_t PIN_DISPLAY_BUTTON = 27;
constexpr uint8_t PIN_CONFIG_BUTTON = 0;
constexpr uint8_t OLED_ADDRESS = 0x3C;

constexpr uint32_t SENSOR_REQUEST_PERIOD_MS = 2000;
constexpr uint32_t SENSOR_CONVERSION_MS = 800;
constexpr uint32_t SENSOR_STALE_MS = 12000;
constexpr uint32_t CONTROL_PERIOD_MS = 500;
constexpr uint32_t DISPLAY_PERIOD_MS = 350;
constexpr uint32_t LONG_PRESS_MS = 1800;
constexpr uint8_t DISPLAY_PAGE_COUNT = 4;

constexpr int16_t DEFAULT_TMIN_X100 = 3000;  // 30.0 C
constexpr int16_t DEFAULT_TMAX_X100 = 7000;  // 70.0 C

Supla::Eeprom eeprom;
Supla::ESPWifi wifi;
Supla::LittleFsConfig configSupla;
Supla::EspWebServer suplaWebServer;

Supla::Control::HvacBase *minThermostat = nullptr;
Supla::Control::HvacBase *maxThermostat = nullptr;
Supla::Sensor::VirtualThermometer *sensor1Channel = nullptr;
Supla::Sensor::VirtualThermometer *sensor2Channel = nullptr;
Supla::Control::VirtualRelay *autoSwitch = nullptr;
Supla::Control::VirtualRelay *manualSwitch = nullptr;
Supla::Sensor::GeneralPurposeMeasurement *deltaChannel = nullptr;
Supla::Sensor::GeneralPurposeMeasurement *outputChannel = nullptr;
Supla::Sensor::GeneralPurposeMeasurement *alarmChannel = nullptr;
Supla::Sensor::GeneralPurposeMeasurement *modeChannel = nullptr;

using FloatParameter = Supla::Html::CustomParameterTemplate<float>;
using IntParameter = Supla::Html::CustomParameter;

FloatParameter *paramDeltaOn = nullptr;
FloatParameter *paramDeltaOff = nullptr;
FloatParameter *paramLimitHys = nullptr;
IntParameter *paramFailsafeOn = nullptr;
IntParameter *paramRelayActiveHigh = nullptr;
Supla::Html::CustomTextParameter *paramAddress1 = nullptr;
Supla::Html::CustomTextParameter *paramAddress2 = nullptr;

struct Settings {
  float deltaOnC = 2.0f;
  float deltaOffC = 0.5f;
  float limitHysC = 0.5f;
  bool failsafeOn = true;
  bool relayActiveHigh = true;
};
Settings settings;

struct TempState {
  DeviceAddress address = {};
  float value = NAN;
  bool hasValue = false;
  uint32_t lastValidMs = 0;
};

OneWire oneWire(PIN_ONEWIRE);
DallasTemperature dallas(&oneWire);
TempState temp[2];
bool address1Valid = false;
bool address2Valid = false;
bool addressConfigValid = false;
bool temperatureRequestPending = false;
uint32_t lastTemperatureRequestMs = 0;
uint32_t temperatureRequestStartedMs = 0;

Adafruit_SSD1306 display(128, 64, &Wire, -1);
bool displayReady = false;
uint8_t displayPage = 0;
bool displayButtonStable = HIGH;
bool displayButtonLastRaw = HIGH;
uint32_t displayButtonChangedMs = 0;
uint32_t displayButtonPressedMs = 0;
bool displayLongPressHandled = false;
uint32_t lastDisplayMs = 0;
uint32_t lastControlMs = 0;

bool outputState = false;
bool limitMinBlocked = false;
bool limitMaxBlocked = false;
uint32_t alarmMask = 0;

constexpr uint32_t ALARM_CONFIG = 1u << 0;
constexpr uint32_t ALARM_SENSOR1 = 1u << 1;
constexpr uint32_t ALARM_SENSOR2 = 1u << 2;
constexpr uint32_t ALARM_FAILSAFE = 1u << 3;
constexpr uint32_t ALARM_LIMIT_MIN = 1u << 4;
constexpr uint32_t ALARM_LIMIT_MAX = 1u << 5;

bool elapsed(uint32_t now, uint32_t since, uint32_t period) {
  return static_cast<uint32_t>(now - since) >= period;
}

bool validTemperature(float value) {
  return std::isfinite(value) && value >= -55.0f && value <= 125.0f &&
         fabsf(value - 85.0f) > 0.01f;
}

int hexDigit(char c) {
  if (c >= '0' && c <= '9') return c - '0';
  if (c >= 'a' && c <= 'f') return c - 'a' + 10;
  if (c >= 'A' && c <= 'F') return c - 'A' + 10;
  return -1;
}

bool parseDallasAddress(const char *text, DeviceAddress result) {
  if (!text) return false;
  while (*text == ' ' || *text == '\t') ++text;
  if (text[0] == '0' && (text[1] == 'x' || text[1] == 'X')) text += 2;

  char compact[17] = {};
  size_t length = 0;
  for (const char *p = text; *p != '\0'; ++p) {
    if (hexDigit(*p) >= 0) {
      if (length >= 16) return false;
      compact[length++] = *p;
    } else if (*p == ':' || *p == '-' || *p == ' ' || *p == '\t') {
      continue;
    } else {
      return false;
    }
  }
  if (length != 16) return false;

  for (size_t i = 0; i < 8; ++i) {
    int hi = hexDigit(compact[i * 2]);
    int lo = hexDigit(compact[i * 2 + 1]);
    if (hi < 0 || lo < 0) return false;
    result[i] = static_cast<uint8_t>((hi << 4) | lo);
  }
  return result[0] == 0x28 && OneWire::crc8(result, 7) == result[7];
}

void printAddress(const DeviceAddress address) {
  for (uint8_t i = 0; i < 8; ++i) {
    if (address[i] < 0x10) Serial.print('0');
    Serial.print(address[i], HEX);
  }
}

void ensureTextParameter(Supla::Html::CustomTextParameter *parameter,
                         const char *defaultValue,
                         char *destination,
                         size_t destinationSize) {
  if (!parameter || !destination || destinationSize == 0) return;
  if (!parameter->getParameterValue(destination,
                                    static_cast<int>(destinationSize))) {
    parameter->setParameterValue(defaultValue);
    strncpy(destination, defaultValue, destinationSize - 1);
    destination[destinationSize - 1] = '\0';
  }
}

void configureGpm(Supla::Sensor::GeneralPurposeMeasurement *channel,
                  const char *unit,
                  uint8_t precision) {
  channel->setDefaultUnitAfterValue(unit);
  channel->setDefaultValuePrecision(precision);
  channel->setDefaultRefreshIntervalMs(1000);
}

void configureSetpointThermostat(Supla::Control::HvacBase *hvac,
                                 int16_t initialSetpoint) {
  hvac->setHeatingAndCoolingSupported(true);
  hvac->setDefaultSubfunction(SUPLA_HVAC_SUBFUNCTION_HEAT);
  hvac->setTemperatureRoomMin(0);
  hvac->setTemperatureRoomMax(9500);
  hvac->setDefaultTemperatureRoomMin(SUPLA_CHANNELFNC_HVAC_THERMOSTAT, 0);
  hvac->setDefaultTemperatureRoomMax(SUPLA_CHANNELFNC_HVAC_THERMOSTAT, 9500);
  hvac->setTemperatureSetpointChangeSwitchesToManualMode(true);
  hvac->setTemperatureSetpointHeat(initialSetpoint);
  hvac->setTargetMode(SUPLA_HVAC_MODE_HEAT);
  new Supla::Html::HvacParameters(hvac);
}

void createConfigurationPage() {
  new Supla::Html::DeviceInfo(&SuplaDevice);
  new Supla::Html::WifiParameters;
  new Supla::Html::ProtocolParameters;

  paramDeltaOn = new FloatParameter(
      "don", "Roznica zalaczenia DeltaON [C]", 2.0f, 0.1f, 30.0f, 1);
  paramDeltaOff = new FloatParameter(
      "doff", "Roznica wylaczenia DeltaOFF [C]", 0.5f, -5.0f, 29.0f, 1);
  paramLimitHys = new FloatParameter(
      "lhys", "Histereza limitow Tmin/Tmax [C]", 0.5f, 0.0f, 10.0f, 1);
  paramFailsafeOn = new IntParameter(
      "fs", "Blad czujnika: 1=ON 0=OFF", 1, 0, 1);
  paramRelayActiveHigh = new IntParameter(
      "rpol", "Przekaznik: 1=aktywny HIGH 0=aktywny LOW", 1, 0, 1);
  paramAddress1 = new Supla::Html::CustomTextParameter(
      "addr1", "DS18B20 czujnik 1 / zrodlo (16 HEX)", 24);
  paramAddress2 = new Supla::Html::CustomTextParameter(
      "addr2", "DS18B20 czujnik 2 / bojler (16 HEX)", 24);
}

void createSuplaChannels() {
  // Kanaly 0-1: dwa nastawniki temperatur w aplikacji SUPLA.
  minThermostat = new Supla::Control::HvacBase();
  configureSetpointThermostat(minThermostat, DEFAULT_TMIN_X100);

  maxThermostat = new Supla::Control::HvacBase();
  configureSetpointThermostat(maxThermostat, DEFAULT_TMAX_X100);

  // Kanaly 2-3: temperatury fizyczne.
  sensor1Channel = new Supla::Sensor::VirtualThermometer;
  sensor2Channel = new Supla::Sensor::VirtualThermometer;

  // Kanaly 4-5: sterowanie trybem.
  autoSwitch = new Supla::Control::VirtualRelay;
  autoSwitch->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
  autoSwitch->setDefaultStateOn();

  manualSwitch = new Supla::Control::VirtualRelay;
  manualSwitch->setDefaultFunction(SUPLA_CHANNELFNC_POWERSWITCH);
  manualSwitch->setDefaultStateOff();

  // Kanaly diagnostyczne / podglad.
  deltaChannel = new Supla::Sensor::GeneralPurposeMeasurement;
  configureGpm(deltaChannel, " C", 1);
  outputChannel = new Supla::Sensor::GeneralPurposeMeasurement;
  configureGpm(outputChannel, "", 0);
  alarmChannel = new Supla::Sensor::GeneralPurposeMeasurement;
  configureGpm(alarmChannel, "", 0);
  modeChannel = new Supla::Sensor::GeneralPurposeMeasurement;
  configureGpm(modeChannel, "", 0);

  // W aplikacji przy obu nastawnikach widac odpowiedni pomiar temperatury.
  minThermostat->setMainThermometerChannelNo(2);
  maxThermostat->setMainThermometerChannelNo(3);
}

void loadRuntimeSettings() {
  settings.deltaOnC = paramDeltaOn->getParameterValue();
  settings.deltaOffC = paramDeltaOff->getParameterValue();
  settings.limitHysC = paramLimitHys->getParameterValue();
  settings.failsafeOn = paramFailsafeOn->getParameterValue() != 0;
  settings.relayActiveHigh = paramRelayActiveHigh->getParameterValue() != 0;

  if (settings.deltaOffC >= settings.deltaOnC) {
    settings.deltaOffC = settings.deltaOnC - 0.1f;
  }

  char a1[32] = {};
  char a2[32] = {};
  ensureTextParameter(paramAddress1, "", a1, sizeof(a1));
  ensureTextParameter(paramAddress2, "", a2, sizeof(a2));

  address1Valid = parseDallasAddress(a1, temp[0].address);
  address2Valid = parseDallasAddress(a2, temp[1].address);
  addressConfigValid = address1Valid && address2Valid;

  Serial.println("Konfiguracja termostatu roznicowego:");
  Serial.printf("  DeltaON %.1f C, DeltaOFF %.1f C, HysLimit %.1f C\n",
                settings.deltaOnC, settings.deltaOffC, settings.limitHysC);
  Serial.printf("  FAILSAFE przy bledzie czujnika: %s\n",
                settings.failsafeOn ? "ON" : "OFF");
  Serial.printf("  przekaznik aktywny stanem: %s\n",
                settings.relayActiveHigh ? "HIGH" : "LOW");
  Serial.printf("  DS1 '%s' -> %s\n", a1, address1Valid ? "OK" : "BLAD");
  Serial.printf("  DS2 '%s' -> %s\n", a2, address2Valid ? "OK" : "BLAD");
  Serial.printf("  adresy czujnikow: %s\n",
                addressConfigValid ? "poprawne" : "BLAD");
}

float minTemperatureC() {
  if (!minThermostat) return DEFAULT_TMIN_X100 / 100.0f;
  int value = minThermostat->getTemperatureSetpointHeat();
  if (value < 0 || value > 9500) value = DEFAULT_TMIN_X100;
  return value / 100.0f;
}

float maxTemperatureC() {
  if (!maxThermostat) return DEFAULT_TMAX_X100 / 100.0f;
  int value = maxThermostat->getTemperatureSetpointHeat();
  if (value < 0 || value > 9500) value = DEFAULT_TMAX_X100;
  return value / 100.0f;
}

bool sensorValid(uint8_t index, uint32_t now) {
  return index < 2 && temp[index].hasValue && validTemperature(temp[index].value) &&
         !elapsed(now, temp[index].lastValidMs, SENSOR_STALE_MS);
}

void publishTemperature(uint8_t index, uint32_t now) {
  Supla::Sensor::VirtualThermometer *channel =
      index == 0 ? sensor1Channel : sensor2Channel;
  if (!channel) return;
  channel->setValue(sensorValid(index, now)
                        ? temp[index].value
                        : TEMPERATURE_NOT_AVAILABLE);
}

void requestTemperatures(uint32_t now) {
  dallas.requestTemperatures();
  temperatureRequestStartedMs = now;
  lastTemperatureRequestMs = now;
  temperatureRequestPending = true;
}

void completeTemperatureRead(uint32_t now) {
  for (uint8_t i = 0; i < 2; ++i) {
    float raw = dallas.getTempC(temp[i].address);
    if (validTemperature(raw)) {
      temp[i].value = raw;
      temp[i].hasValue = true;
      temp[i].lastValidMs = now;
    }
    publishTemperature(i, now);
  }
  temperatureRequestPending = false;
}

void serviceTemperatureBus(uint32_t now) {
  if (temperatureRequestPending &&
      elapsed(now, temperatureRequestStartedMs, SENSOR_CONVERSION_MS)) {
    completeTemperatureRead(now);
  }
  if (!temperatureRequestPending &&
      elapsed(now, lastTemperatureRequestMs, SENSOR_REQUEST_PERIOD_MS)) {
    requestTemperatures(now);
  }
}

void scanDallasBus() {
  Serial.printf("DS18B20 znalezione: %u\n", dallas.getDeviceCount());
  DeviceAddress address;
  for (uint8_t i = 0; i < dallas.getDeviceCount(); ++i) {
    if (dallas.getAddress(address, i)) {
      Serial.print("  ");
      printAddress(address);
      Serial.println();
    }
  }
}

void writeRelay(bool on) {
  outputState = on;
  const bool physicalHigh = settings.relayActiveHigh ? on : !on;
  digitalWrite(PIN_RELAY, physicalHigh ? HIGH : LOW);
}

uint8_t currentModeCode(bool sensorFault) {
  if (autoSwitch && autoSwitch->isOn()) {
    return sensorFault ? 3 : 0;  // 0=AUTO, 3=FAILSAFE
  }
  return manualSwitch && manualSwitch->isOn() ? 2 : 1;  // 2=MAN ON, 1=MAN OFF
}

void runControl(uint32_t now) {
  bool desired = outputState;
  alarmMask = 0;

  const bool autoMode = autoSwitch && autoSwitch->isOn();
  const bool manualOn = manualSwitch && manualSwitch->isOn();

  const bool s1ok = sensorValid(0, now);
  const bool s2ok = sensorValid(1, now);
  const bool sensorFault = !s1ok || !s2ok;

  if (!addressConfigValid) {
    alarmMask |= ALARM_CONFIG;
    desired = false;
  } else if (!autoMode) {
    desired = manualOn;
  } else if (sensorFault) {
    if (!s1ok) alarmMask |= ALARM_SENSOR1;
    if (!s2ok) alarmMask |= ALARM_SENSOR2;
    alarmMask |= ALARM_FAILSAFE;
    desired = settings.failsafeOn;
  } else {
    const float t1 = temp[0].value;
    const float t2 = temp[1].value;
    const float tmin = minTemperatureC();
    const float tmax = maxTemperatureC();

    // Limity maja osobna histereze, zeby wyjscie nie klapalo na granicy.
    if (limitMinBlocked) {
      if (t1 >= tmin + settings.limitHysC) limitMinBlocked = false;
    } else if (t1 < tmin) {
      limitMinBlocked = true;
    }

    if (limitMaxBlocked) {
      if (t2 <= tmax - settings.limitHysC) limitMaxBlocked = false;
    } else if (t2 >= tmax) {
      limitMaxBlocked = true;
    }

    if (limitMinBlocked) {
      alarmMask |= ALARM_LIMIT_MIN;
      desired = false;
    } else if (limitMaxBlocked) {
      alarmMask |= ALARM_LIMIT_MAX;
      desired = false;
    } else {
      const float delta = t1 - t2;
      if (delta >= settings.deltaOnC) {
        desired = true;
      } else if (delta <= settings.deltaOffC) {
        desired = false;
      }
      // pomiedzy DeltaOFF a DeltaON zachowujemy poprzedni stan
    }
  }

  if (desired != outputState) {
    writeRelay(desired);
    Serial.printf("WYJSCIE -> %s\n", outputState ? "ON" : "OFF");
  }

  if (deltaChannel) {
    if (s1ok && s2ok) deltaChannel->setValue(temp[0].value - temp[1].value);
    else deltaChannel->setValue(NAN);
  }
  if (outputChannel) outputChannel->setValue(outputState ? 1.0 : 0.0);
  if (alarmChannel) alarmChannel->setValue(static_cast<double>(alarmMask));
  if (modeChannel) modeChannel->setValue(currentModeCode(sensorFault));

  publishTemperature(0, now);
  publishTemperature(1, now);
}

const char *modeText(uint32_t now) {
  if (autoSwitch && autoSwitch->isOn()) {
    if (!sensorValid(0, now) || !sensorValid(1, now)) return "FAILSAFE";
    return "AUTO";
  }
  return manualSwitch && manualSwitch->isOn() ? "MAN ON" : "MAN OFF";
}

void drawHeader(const char *title) {
  display.setTextSize(1);
  display.setCursor(0, 0);
  display.print(title);
  display.setCursor(96, 0);
  display.print(WiFi.status() == WL_CONNECTED ? "WIFI" : "OFF");
  display.drawLine(0, 10, 127, 10, SSD1306_WHITE);
}

void printTempOrDash(float value, bool valid) {
  if (valid) display.print(value, 1);
  else display.print("---");
}

void drawDisplay() {
  if (!displayReady) return;
  const uint32_t now = millis();
  const bool s1ok = sensorValid(0, now);
  const bool s2ok = sensorValid(1, now);

  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  switch (displayPage) {
    case 0: {
      drawHeader("TERMOSTAT ROZN.");
      display.setCursor(0, 16);
      display.print("T1: "); printTempOrDash(temp[0].value, s1ok); display.print(" C");
      display.setCursor(0, 29);
      display.print("T2: "); printTempOrDash(temp[1].value, s2ok); display.print(" C");
      display.setCursor(0, 42);
      display.print("dT: ");
      if (s1ok && s2ok) display.print(temp[0].value - temp[1].value, 1);
      else display.print("---");
      display.print(" C");
      display.setCursor(0, 55);
      display.print(modeText(now));
      display.setCursor(94, 55);
      display.print(outputState ? "ON" : "OFF");
      break;
    }
    case 1:
      drawHeader("PROGI SUPLA");
      display.setCursor(0, 18);
      display.printf("Tmin: %.1f C", minTemperatureC());
      display.setCursor(0, 34);
      display.printf("Tmax: %.1f C", maxTemperatureC());
      display.setCursor(0, 50);
      display.printf("dON %.1f dOFF %.1f", settings.deltaOnC, settings.deltaOffC);
      break;

    case 2:
      drawHeader("TRYB / ALARM");
      display.setCursor(0, 17);
      display.print("Tryb: "); display.print(modeText(now));
      display.setCursor(0, 31);
      display.print("Wyjscie: "); display.print(outputState ? "ON" : "OFF");
      display.setCursor(0, 45);
      display.printf("Alarm: 0x%02lX", static_cast<unsigned long>(alarmMask));
      display.setCursor(0, 57);
      display.print(settings.failsafeOn ? "Sensor ERR => ON" : "Sensor ERR => OFF");
      break;

    default:
      drawHeader("CZUJNIKI");
      display.setCursor(0, 17);
      display.print("DS1: "); display.print(s1ok ? "OK" : "BLAD");
      display.setCursor(0, 31);
      display.print("DS2: "); display.print(s2ok ? "OK" : "BLAD");
      display.setCursor(0, 45);
      display.print("Cfg: "); display.print(addressConfigValid ? "OK" : "BLAD");
      display.setCursor(0, 57);
      display.print("GPIO26 relay");
      break;
  }
  display.display();
}

void cycleLocalMode() {
  if (!autoSwitch || !manualSwitch) return;
  if (autoSwitch->isOn()) {
    autoSwitch->turnOff();
    manualSwitch->turnOn();
  } else if (manualSwitch->isOn()) {
    manualSwitch->turnOff();
  } else {
    autoSwitch->turnOn();
  }
}

void serviceDisplayButton(uint32_t now) {
  const bool raw = digitalRead(PIN_DISPLAY_BUTTON);
  if (raw != displayButtonLastRaw) {
    displayButtonLastRaw = raw;
    displayButtonChangedMs = now;
  }
  if (!elapsed(now, displayButtonChangedMs, 30)) return;

  if (raw != displayButtonStable) {
    displayButtonStable = raw;
    if (displayButtonStable == LOW) {
      displayButtonPressedMs = now;
      displayLongPressHandled = false;
    } else if (!displayLongPressHandled) {
      displayPage = (displayPage + 1) % DISPLAY_PAGE_COUNT;
      drawDisplay();
    }
  }

  if (displayButtonStable == LOW && !displayLongPressHandled &&
      elapsed(now, displayButtonPressedMs, LONG_PRESS_MS)) {
    displayLongPressHandled = true;
    cycleLocalMode();
    drawDisplay();
  }
}

void appSetup() {
  Serial.begin(115200);
  delay(100);
  Serial.println();
  Serial.println("Termostat roznicowy SUPLA ON/OFF - start");

  // Na starcie przyjmujemy standardowy modul aktywny HIGH. Po zaladowaniu
  // konfiguracji stan zostanie ponownie ustawiony zgodnie z parametrem rpol.
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);
  pinMode(PIN_DISPLAY_BUTTON, INPUT_PULLUP);

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  displayReady = display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDRESS, true, false);
  if (displayReady) {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.setTextSize(1);
    display.setCursor(0, 0);
    display.println("TERMOSTAT ROZNICOWY");
    display.println("SUPLA ESP32");
    display.println("Start...");
    display.display();
  } else {
    Serial.println("OLED 0x3C nie odpowiada - praca bez ekranu");
  }

  createConfigurationPage();
  createSuplaChannels();

  auto configButton = new Supla::Control::Button(PIN_CONFIG_BUTTON, true, true);
  configButton->configureAsConfigButton(&SuplaDevice);

  eeprom.setStateSavePeriod(5000);
  SuplaDevice.setName("Termostat roznicowy ON-OFF");
  SuplaDevice.setSwVersion("1.0.0");
  SuplaDevice.setCustomHostnamePrefix("SUPLA-DIFF");
  SuplaDevice.setInitialMode(Supla::InitialMode::StartInCfgMode);
  SuplaDevice.begin(23);

  loadRuntimeSettings();
  writeRelay(false);

  dallas.begin();
  dallas.setWaitForConversion(false);
  if (address1Valid) dallas.setResolution(temp[0].address, 12);
  if (address2Valid) dallas.setResolution(temp[1].address, 12);
  scanDallasBus();

  const uint32_t now = millis();
  lastTemperatureRequestMs = now - SENSOR_REQUEST_PERIOD_MS;
  lastControlMs = now - CONTROL_PERIOD_MS;
  lastDisplayMs = now - DISPLAY_PERIOD_MS;
  requestTemperatures(now);
  drawDisplay();
}

void appLoop() {
  SuplaDevice.iterate();
  const uint32_t now = millis();
  serviceTemperatureBus(now);
  serviceDisplayButton(now);

  if (elapsed(now, lastControlMs, CONTROL_PERIOD_MS)) {
    lastControlMs = now;
    runControl(now);
  }
  if (elapsed(now, lastDisplayMs, DISPLAY_PERIOD_MS)) {
    lastDisplayMs = now;
    drawDisplay();
  }
}

}  // namespace

void setup() { appSetup(); }
void loop() { appLoop(); }
