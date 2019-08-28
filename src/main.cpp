#include <Arduino.h>
#include <ESP8266WiFi.h>

static const uint8_t PIN_BAT_VOLT  = A0;
static const uint8_t PIN_PWR_ALARM = D6;
static const uint8_t PIN_PWR_STATE = D7;

int cnt = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT);     // alarm LED setup
  digitalWrite(LED_BUILTIN, 1);     // alarm LED off
  
  pinMode(PIN_PWR_ALARM, OUTPUT);   // alarm beeper setup
  analogWriteFreq(8820);            // alarm beeper freq (Hz)
  analogWrite(PIN_PWR_ALARM, 0);    // alarm beeper off

  Serial.begin(115200);

  WiFi.mode(WIFI_OFF);              // Turn off WiFi to save some battery

  analogWrite(PIN_PWR_ALARM, 128);  // Initial beep to state start
  delay(500);
  analogWrite(PIN_PWR_ALARM, 0);
}

void loop() {
  int    pwr  = digitalRead(PIN_PWR_STATE);
  float  volt = analogRead(PIN_BAT_VOLT) / 206.4729; // Measured 874/4,233v on ESP_5E95F1

  if (!pwr) {
    // TODO: let a timer do the beeping
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));       // let it blink
    analogWrite(PIN_PWR_ALARM, 128 * digitalRead(LED_BUILTIN)); // let it beep
  } else {
    digitalWrite(LED_BUILTIN, 1);                               // switch off LED
    analogWrite(PIN_PWR_ALARM, 0);                              // silence beeper
  }

  // output to serial every 1s
  // TODO: to be removed... serial will be needed for SIM800L communication
  if (++cnt > 9)
  {
    cnt = 0;
    Serial.print("status: ");
    if (pwr) { Serial.print("external"); }
    else     { Serial.print("battery ");  }
    Serial.println(" power supply, Vbat: " + String(volt, 2) + "v");
  }
    
  delay(100);
}