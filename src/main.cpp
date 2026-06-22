#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <Wire.h>
#include <SPI.h>
#include <RTClib.h>
#include <OneWire.h>
#include <DallasTemperature.h>

//? ADJUST_RTC_PIN - if LOW on startup, RTC is set to compile time

#define TEMPERATURE_PIN_VCC 2
#define TEMPERATURE_PIN_DATA 3
#define MOISTURE_PIN_VCC A2
#define MOISTURE_PIN_DATA A3
#define ADJUST_RTC_PIN 8
#define TEST_PIN A0
#define TEST_BUTTON 11
#define PUMP_PIN 10

const uint32_t wateringDuration = 120000; // 2 minutes 30 secondsin milliseconds
const uint8_t wateringHour = 7;           // 7 AM
const float temperatureThreshold = 30.0;  // 30 degrees Celsius
const uint16_t moistureThreshold = 1800;  // if more than 1800mV watering is needed

/* Calibration notes:
Soil Moisture Sensor Calibration:
Dry (secco)
Range 4.5V: 2690 → 2796 mV
Range 3.8V: 2710 → 2773 mV

Wet (bagnato)
Range 4.5V: 1007 → 1069 mV
Range 3.8V: 1014 → 1051 mV
*/

OneWire oneWire(TEMPERATURE_PIN_DATA);
DallasTemperature tempSensor(&oneWire);
RTC_DS3231 rtc;
bool isTestMode = false;

enum SeasonMode
{
  SPRING_EARLY, // irrigazione ogni 3 giorni
  SUMMER_EARLY, // irrigazione ogni 2 giorni + logica "irrigare domani"
  SUMMER_PEAK   // ogni giorno + controllo ore 13 per irrigazione alle 21
};

enum ButtonEvent
{
  NONE,
  SHORT_PRESS,
  LONG_PRESS
};

DateTime compileTime();
float readTemperatureC();
void computeNextWakeUpAndGoToSleep();
void goToSleepUntil(DateTime target);
bool canWatering();
ButtonEvent readTestButton();
void printDiagnostics();
uint16_t readSoilRaw();
long readVcc_mV();
long readSoil_mV();

// Watchdog interrupt handler
ISR(WDT_vect)
{
  // DO NOT put code here
  // WDT interrupt just wakes MCU from sleep
}

void sleepFor8s()
{
  // Imposta WDT per interrupt every 8s
  MCUSR &= ~(1 << WDRF); // pulisce flag reset WDT
  cli();
  wdt_reset();
  // abilitazione cambio WDT (WDCE)
  WDTCSR |= (1 << WDCE) | (1 << WDE);
  // setta WDT interrupt mode 8s
  WDTCSR = (1 << WDIE) | (1 << WDP3) | (1 << WDP0); // WDTO_8S
  sei();

  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
  sleep_cpu();
  sleep_disable();

  // disabilita WDT
  cli();
  wdt_disable();
  sei();
}

void setup()
{
  Serial.begin(9600);
  // small power saving: disable analog comparator
  ACSR |= (1 << ACD);

  pinMode(ADJUST_RTC_PIN, INPUT_PULLUP);

  pinMode(TEST_PIN, INPUT_PULLUP);
  isTestMode = (digitalRead(TEST_PIN) == LOW);

  pinMode(TEST_BUTTON, INPUT_PULLUP);

  pinMode(PUMP_PIN, OUTPUT);
  digitalWrite(PUMP_PIN, LOW);

  pinMode(MOISTURE_PIN_VCC, OUTPUT);
  digitalWrite(MOISTURE_PIN_VCC, LOW);

  pinMode(MOISTURE_PIN_DATA, INPUT);

  pinMode(TEMPERATURE_PIN_VCC, OUTPUT);
  digitalWrite(TEMPERATURE_PIN_VCC, HIGH);
  delay(20);

  tempSensor.begin();
  rtc.begin();

  if (digitalRead(ADJUST_RTC_PIN) == LOW)
  {
    // rtc.adjust(compileTime());
    rtc.adjust(DateTime(2026, 4, 23, 11, 16, 0));
  }
}

void loop()
{
  if (isTestMode)
  {
    static unsigned long pumpStartTime = 0;
    static bool pumpRunning = false;

    ButtonEvent ev = readTestButton();

    if (ev == SHORT_PRESS)
    {
      printDiagnostics();
    }

    if (ev == LONG_PRESS)
    {
      if (digitalRead(PUMP_PIN) == LOW)
      {
        Serial.println("Starting pump (manual test mode)...");
        digitalWrite(PUMP_PIN, HIGH);
        pumpRunning = true;
        pumpStartTime = millis(); // track start
      }
      else
      {
        Serial.println("Stopping pump (manual override)...");
        digitalWrite(PUMP_PIN, LOW);
        pumpRunning = false;
      }
    }

    // Auto-stop pump after wateringDuration
    if (pumpRunning && millis() - pumpStartTime >= wateringDuration)
    {
      Serial.println("Pump auto-stop (duration reached)");
      digitalWrite(PUMP_PIN, LOW);
      pumpRunning = false;
    }
  }
  else
  {
    computeNextWakeUpAndGoToSleep();
    if (canWatering())
    {
      digitalWrite(PUMP_PIN, HIGH);
      delay(wateringDuration);
      digitalWrite(PUMP_PIN, LOW);
    }
  }
}

float readTemperatureC()
{
  digitalWrite(TEMPERATURE_PIN_VCC, HIGH);
  delay(20);

  tempSensor.requestTemperatures(); // start conversion
  float t = tempSensor.getTempCByIndex(0);

  digitalWrite(TEMPERATURE_PIN_VCC, LOW);
  return t;
}

// misura Vcc (mV) usando la bandgap interna da 1.1V
long readVcc_mV()
{
  // seleziona il canale interno 1.1V come ingresso e usa Vcc come riferimento
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  delay(2);            // stabilizza
  ADCSRA |= _BV(ADSC); // avvia conversione
  while (bit_is_set(ADCSRA, ADSC))
    ;
  uint16_t result = ADC;
  // Vcc (mV) = 1.1V * 1023 / result
  long vcc = (1100L * 1023L) / result;
  return vcc;
}

uint16_t readSoilRaw()
{
  const uint8_t samples = 5;
  uint16_t sum = 0;
  digitalWrite(MOISTURE_PIN_VCC, HIGH);
  delay(20);
  for (uint8_t i = 0; i < samples; i++)
  {
    sum += analogRead(MOISTURE_PIN_DATA);
    delay(20);
  }
  digitalWrite(MOISTURE_PIN_VCC, LOW);
  return sum / samples;
}

long readSoil_mV()
{
  readSoilRaw(); // discard first reading
  uint16_t raw = readSoilRaw();
  long vcc_mV = readVcc_mV();
  long vout_mV = (raw * vcc_mV) / 1023L;
  return vout_mV;
}

DateTime compileTime()
{
  return DateTime(F(__DATE__), F(__TIME__));
}

void computeNextWakeUpAndGoToSleep()
{
  DateTime now = rtc.now();

  // -----------------------------
  // 1) DETERMINA STAGIONE
  // -----------------------------
  SeasonMode season;

  if (now.month() == 3 || now.month() == 4)
  {
    season = SPRING_EARLY; // primavera
  }
  else if (now.month() == 5 || now.month() == 6)
  {
    season = SUMMER_EARLY; // inizio estate
  }
  else if (now.month() == 7 || now.month() == 8)
  {
    season = SUMMER_PEAK; // piena estate
  }
  else
  {
    season = SPRING_EARLY; // default
  }

  //
  // *********************
  //      SPRING_EARLY
  // *********************
  //
  if (season == SPRING_EARLY)
  {
    // Wake up directly at next 7AM in 3 days
    DateTime target = DateTime(
        now.year(),
        now.month(),
        now.day(),
        7, 0, 0);

    // Add 3 days
    target = target + TimeSpan(3, 0, 0, 0);

    goToSleepUntil(target);
    return;
  }

  // *********************
  //      SUMMER_EARLY
  // *********************
  //
  if (season == SUMMER_EARLY)
  {
    // If just woke at 13:00 → read temperature then sleep until 7AM next day
    if (now.hour() == 13)
    {
      float temp = readTemperatureC();

      // if temp > threshold → water tomorrow at 7AM even if not 2 days
      if (temp > temperatureThreshold)
      {
        // Irrigation flag mapped via alarm logic:
        // → We simply schedule wake-up at next day 7AM
        DateTime target = DateTime(now.year(), now.month(), now.day(), 7, 0, 0) + TimeSpan(1, 0, 0, 0);
        goToSleepUntil(target);
        return;
      }

      // else: normal mode → sleep until 7AM after 2 days
      DateTime target = DateTime(now.year(), now.month(), now.day(), 7, 0, 0) + TimeSpan(2, 0, 0, 0);
      goToSleepUntil(target);
      return;
    }

    // If NOT 13:00 yet → sleep until 13:00 today
    if (now.hour() < 13)
    {
      DateTime target(now.year(), now.month(), now.day(), 13, 0, 0);
      goToSleepUntil(target);
      return;
    }

    // If past 13:00 → sleep until 7AM after 2 days
    DateTime target = DateTime(now.year(), now.month(), now.day(), 7, 0, 0) + TimeSpan(2, 0, 0, 0);
    goToSleepUntil(target);
    return;
  }

  //
  // *********************
  //      SUMMER_PEAK
  // *********************
  //
  if (season == SUMMER_PEAK)
  {
    // If at 13:00 → sample temperature then sleep for irrigation or next 13
    if (now.hour() == 13)
    {
      float temp = readTemperatureC();

      if (temp > temperatureThreshold)
      {
        // High temperature → extra watering at 21:00
        DateTime target(now.year(), now.month(), now.day(), 21, 0, 0);
        goToSleepUntil(target);
        return;
      }

      // Else: wake tomorrow at 7:00
      DateTime target = DateTime(now.year(), now.month(), now.day(), 7, 0, 0) + TimeSpan(1, 0, 0, 0);
      goToSleepUntil(target);
      return;
    }

    // If before 13 → sleep until 13
    if (now.hour() < 13)
    {
      DateTime target(now.year(), now.month(), now.day(), 13, 0, 0);
      goToSleepUntil(target);
      return;
    }

    // If past 13 → next wake is 7AM tomorrow
    DateTime target = DateTime(now.year(), now.month(), now.day(), 7, 0, 0) + TimeSpan(1, 0, 0, 0);
    goToSleepUntil(target);
    return;
  }
}

void goToSleepUntil(DateTime target)
{
  while (target.unixtime() > rtc.now().unixtime())
  {
    // Clear watchdog reset flag properly
    MCUSR &= ~(1 << WDRF);

    // Disable interrupts while reconfiguring WDT
    cli();
    wdt_reset();
    // Enable watchdog change
    WDTCSR |= (1 << WDCE) | (1 << WDE);
    // Set WDT interrupt every 8 seconds (no reset)
    WDTCSR = (1 << WDIE) | WDTO_8S;
    sei();

    // Sleep
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
    sleep_cpu();
    sleep_disable();

    // Disable WDT to avoid unexpected reset while running
    cli();
    wdt_disable();
    sei();
  }
}

bool canWatering()
{
  uint8_t currentHour = rtc.now().hour();
  if (currentHour == 7 || currentHour == 21)
  {
    uint16_t moisture = readSoil_mV();
    return (moisture >= moistureThreshold);
  }
  return false;
}

ButtonEvent readTestButton()
{
  static unsigned long pressStartTime = 0;
  static bool wasPressed = false;

  bool isPressed = (digitalRead(TEST_BUTTON) == LOW);

  if (isPressed && !wasPressed)
  {
    // Button just pressed
    pressStartTime = millis();
    wasPressed = true;
  }
  else if (!isPressed && wasPressed)
  {
    // Button just released
    unsigned long pressDuration = millis() - pressStartTime;
    wasPressed = false;

    if (pressDuration >= 2000)
    {
      return LONG_PRESS;
    }
    else if (pressDuration >= 50)
    {
      return SHORT_PRESS;
    }
  }

  return NONE;
}

void printDiagnostics()
{
  DateTime now = rtc.now();
  float temp = readTemperatureC();
  uint16_t moisture = readSoil_mV();

  Serial.print("Time: ");
  Serial.print(now.timestamp());
  Serial.print(" | Temp: ");
  Serial.print(temp);
  Serial.print(" C | Moisture: ");
  Serial.print(moisture);
  Serial.println(" mV");
}