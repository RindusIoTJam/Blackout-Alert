#include <Arduino.h>
#include <ESP8266WiFi.h>

int    cnt = 0;

void setup() {
  pinMode(LED_BUILTIN, OUTPUT); // alarm LED setup
  digitalWrite(LED_BUILTIN, 1); // alarm LED off
  
  pinMode(D6, OUTPUT);          // alarm beeper setup
  analogWriteFreq(8820);        // alarm beeper freq (Hz)
  analogWrite(D6, 0);           // alarm beeper off

  Serial.begin(115200);

  // Turn off WiFi to save some battery
  WiFi.mode(WIFI_OFF);

  analogWrite(D6, 128);
  delay(500);
  analogWrite(D6, 0);
}

void loop() {
  int    pwr  = digitalRead(D7);
  float  volt = analogRead(A0) / 206.4729; // Measured 874/4,233v on ESP_5E95F1

  if (!pwr) {
    // let it blink
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    analogWrite(D6, 128 * digitalRead(LED_BUILTIN));
  } else {
    // switch off
    digitalWrite(LED_BUILTIN, 1);
    analogWrite(D6, 0);
  }

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