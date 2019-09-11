#define SERIAL_BUFFER_SIZE 256

#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <SoftwareSerial.h>

#define BEEP true

#define GSM_RX D1
#define GSM_TX D2

// Measured 874 : 4,233v = 206.4729 on ESP_5E95F1
#define BAT_DIVIDER 206.4729

// 1538 * 13ms = 20s
#define INPUT_TIMEOUT 1538

static const uint8_t PIN_BAT_VOLT  = A0; // On A0 the battery voltage is monitored
static const uint8_t PIN_PWR_ALARM = D6; // On D6 the beeper is connected
static const uint8_t PIN_PWR_STATE = D7; // On D7 the power state is monitored

volatile uint8_t power  = digitalRead(PIN_PWR_STATE);
volatile uint8_t beeper = false;

uint8_t alerted   = false;
uint8_t recovered = power;
String  alarm_destination;
String  alarm_message;

SoftwareSerial SerialGsm(GSM_RX, GSM_TX);

/*
 * An beeper OK confirmation ... beep beep
 */
void inline beepOK() {
  analogWrite(PIN_PWR_ALARM, 128);  // Initial beep to state start
  delay(100);
  analogWrite(PIN_PWR_ALARM, 0);
  delay(100);
  analogWrite(PIN_PWR_ALARM, 128);  // Initial beep to state start
  delay(100);
  analogWrite(PIN_PWR_ALARM, 0);
}

/*
 * An beeper confirmation ... beeeeeep
 */
void inline beepERROR() {
  analogWrite(PIN_PWR_ALARM, 128);  // Initial beep to state start
  delay(300);
  analogWrite(PIN_PWR_ALARM, 0);
}

/*
 * read from SIM800L
 * 
 * @return String the data read from SIM800L
 */
String inline readSIM800L(){
  int timeout=0;
  while(!SerialGsm.available() && timeout < 769) { // timeout 10s
    delay(13);
    timeout++;
  }
  if(SerialGsm.available()) {
 	  return SerialGsm.readString();
  }
  return "";
}

/*
 * Init SIM800L
 * 
 * @return true  if SIM800L was found
 *         false if SIM800L wans't found
 */
uint8_t inline initSIM800L() {
  String response;

  SerialGsm.print(F("AT\r\n"));
  int tries = 0;
  while(readSIM800L().indexOf("OK")==-1 ) {
    if(++tries>6) break;
    SerialGsm.print(F("AT\r\n"));
    Serial.print(".");
  }
  Serial.println();
  if(tries>6) {
    Serial.println("CRIT: NO RESPONE ON SerialGsm() IN 60 seconds!");
    return false;
  }

  SerialGsm.print(F("AT+GMM\r\n"));
  response = readSIM800L();
  if(response.indexOf("SIMCOM_SIM800L\r\n\r\nOK")==-1) {
    Serial.println("WARN: GOT ...\r\n"+response+"... but expected 'SIMCOM_SIM800L'.");
    return false;
  } else {
    Serial.println("INFO: FOUND GSM SIM800L.");
  }

  SerialGsm.print(F("AT+CPIN?\r\n"));
  response = readSIM800L();
  if(response.indexOf("+CPIN: READY\r\n\r\nOK")==-1) {
    Serial.println("CRIT: GOT ...\r\n"+response+"... but expected '+CPIN: READY'.");
    return false;
  } else {
    Serial.println("      - SIM unlocked and ready.");
  }

  SerialGsm.print(F("AT+CREG?\r\n"));
  response = readSIM800L();
  if(response.indexOf("+CREG: 0,1\r\n\r\nOK")==-1) {
    Serial.println("CRIT: GOT ...\r\n"+response+"... but expected '+CREG: 0,1'.");
    return false;
  } else {
    Serial.println("      - Registered at home network w/ unsolicited result codes disabled.");
  }

  Serial.println("INFO: GSM SIM800L READY");

  return true;
}

void poweroffSIM800L() {
  SerialGsm.print(F("AT+CPOWD=1\r\n"));
}

/*
 * Send SMS by SIM800L
 * 
 * @param  String number: the number to send the SMS to
 *         String text:   the text to send
 * @return true  if SMS was sent
 *         false if SMS wans't sent
 */
uint8_t inline sendSMS(String number, String text) {
  SerialGsm.print(F("AT+CMGF=1\r\n"));
  Serial.println(readSIM800L());
  SerialGsm.print(F("AT+CMGS=\""));  // command to send sms
  SerialGsm.print(number);        
  SerialGsm.print(F("\"\r\n"));       
  Serial.println(readSIM800L());
  SerialGsm.print (text);
  SerialGsm.print((char)26);
  String _buffer = readSIM800L();
  //expect CMGS:xxx   , where xxx is a number,for the sending sms.
  if(((_buffer.indexOf("CMGS") ) != -1 ) ){
    return true;
  }
  return false;
}

void ICACHE_RAM_ATTR inline ISR_power() {
  power = digitalRead(PIN_PWR_STATE);
}

/*
 * ISR let it blink and maybe beep if power gets lost 
 */
void ICACHE_RAM_ATTR inline ISR_timer0() {
  
  if(!power) {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));         // let it blink
#if (BEEP)
    if(beeper) {
      analogWrite(PIN_PWR_ALARM, 128 * digitalRead(LED_BUILTIN)); // let it beep
    }
#endif
  } else {
    digitalWrite(LED_BUILTIN, 1);                                 // alarm LED off
#if (BEEP)
    analogWrite(PIN_PWR_ALARM, 0);                                // alarm beeper off
#endif
  }

  timer0_write(ESP.getCycleCount() + 20000000L);
}

void writeEEPROM(int addr,String data)
{
  int _size = data.length();
  int i;
  for(i=0;i<_size;i++)
  {
    EEPROM.write(addr+i,data[i]);
  }
  EEPROM.write(addr+_size,'\0');   //Add termination null character for String Data
  EEPROM.commit();
}
 
 
String readEEPROM(int addr)
{
  char data[100]; //Max 100 Bytes
  int len=0;
  unsigned char k;
  k=EEPROM.read(addr);
  while(k != '\0' && len<500)   //Read until null character
  {    
    k=EEPROM.read(addr+len);
    if(k==255) break;
    data[len]=k;
    len++;
  }
  data[len]='\0';
  return String(data);
}

void setAlertShortMessage() {
  Serial.println("! Enter alert message (e.g. BLACKOUT AT HOME!)");
  Serial.println("! and end you input with # (20s timeout!)");
  Serial.println("! Current alert message is: " + alarm_message);
  beepERROR();
  Serial.print("> ");

  int timeout=0;
  uint max_num=64;
  String new_alarm_message = "";
  while(!Serial.available() && timeout < INPUT_TIMEOUT) { // timeout 20s
    delay(13);
    timeout++;
    int input_char = Serial.read();
    if(input_char!=-1) {
      if(new_alarm_message.length()>max_num) {
        break;
      }
      if(input_char=='#') {
        break;
      }
      Serial.write(input_char);
      new_alarm_message += (char) input_char;
    }
  }
  if(new_alarm_message.length()>max_num) {
    Serial.println("WARN: Too many characters (max: "+String(max_num)+"). Alert message unchanged!");
    beepERROR();
  } else if(new_alarm_message.length()==0) {
    Serial.println("WARN: No input. Alert message unchanged!");
    beepERROR();
  } else if(timeout>=INPUT_TIMEOUT) {
    Serial.println("WARN: response timeout ocurred. Alert message unchanged!");
    beepERROR();
  } else {
    alarm_message = new_alarm_message;
    Serial.println("");
    Serial.println("INFO: new alert mobile number: " + alarm_message);
    writeEEPROM(20, alarm_message);
    beepOK();
  }
}

void printHelp() {
  Serial.print("INFO: PRESS ");
  Serial.println(            "'b' (b)oot reason.");
  Serial.println("            'h' show this (h)elp.");
  Serial.println("            'm' to set new alert short (m)essage.");
  Serial.println("                e.g. BLACKOUT AT HOME!");
  Serial.println("            'n' to set new alert mobile (n)umber for alert message.");
  Serial.println("                e.g. 612345678 or 00491517346592");
  Serial.println("            's' for (s)tatus information (Vbat, alert mobile number and msgs).");
  Serial.println("            't' to set send (t)est alert message.");
  Serial.println("      All other chars will be send to SIM800L (e.g. AT<CR>).");
}

void setAlertMobileNumber() {
  Serial.println("! Enter alert mobile number (e.g. 612345678)");
  Serial.println("! and end you input with # (20s timeout!)");
  Serial.println("! Current alert mobile number is: " + alarm_destination);
  beepERROR();
  Serial.print("> ");

  int timeout=0;
  uint max_num=19;
  String new_alarm_destination = "";
  while(!Serial.available() && timeout < INPUT_TIMEOUT) { // timeout 20s
    delay(13);
    timeout++;
    int input_char = Serial.read();
    if(input_char!=-1) {
      if((input_char>47)&(input_char<58)) {
        if(new_alarm_destination.length()>max_num) {
          break;
        }
        Serial.write(input_char);
        new_alarm_destination += (char) input_char;
      }
      if(input_char=='#') break;
    }
  }
  if(new_alarm_destination.length()>max_num) {
    Serial.println("WARN: Too many numbers (max: "+String(max_num)+"). Alert mobile number unchanged!");
    beepERROR();
  } else if(new_alarm_destination.length()==0) {
    Serial.println("WARN: No input. Alert mobile number unchanged!");
    beepERROR();
  } else if(timeout>=INPUT_TIMEOUT) {
    Serial.println("WARN: response timeout ocurred. Alert mobile number unchanged!");
    beepERROR();
  } else {
    alarm_destination = new_alarm_destination;
    Serial.println("");
    Serial.println("INFO: new alert mobile number: " + alarm_destination);
    writeEEPROM(0, alarm_destination);
    beepOK();
  }
}

void setup() {
  WiFi.mode(WIFI_OFF);              // Turn off WiFi to save battery
  Serial.begin(115200);
  SerialGsm.begin(9600);
  EEPROM.begin(512);

  pinMode(LED_BUILTIN, OUTPUT);     // alarm LED setup
  digitalWrite(LED_BUILTIN, 1);     // alarm LED off

  pinMode(PIN_PWR_ALARM, OUTPUT);   // alarm beeper setup
#if (BEEP)
  analogWriteFreq(8820);            // alarm beeper freq (Hz)
  analogWrite(PIN_PWR_ALARM, 0);    // alarm beeper off
#endif

  noInterrupts();

  // interupt to sense power state
  pinMode(PIN_PWR_STATE, INPUT);
  attachInterrupt(digitalPinToInterrupt(PIN_PWR_STATE), ISR_power, CHANGE);

  // interupt to blink/beep every 250ms (80M/20M*1sec)
  timer0_isr_init();
  timer0_attachInterrupt(ISR_timer0);
  timer0_write(ESP.getCycleCount() + 20000000L);

  interrupts();

  if(initSIM800L()) {
#if (BEEP)
    beepOK();
#endif
  } else {
    while(true) {
      analogWrite(PIN_PWR_ALARM, 128); delay( 50);
      analogWrite(PIN_PWR_ALARM,   0); delay(100);
      analogWrite(PIN_PWR_ALARM, 128); delay( 50);
      analogWrite(PIN_PWR_ALARM,   0); delay(100);
      analogWrite(PIN_PWR_ALARM, 128); delay( 50);
      analogWrite(PIN_PWR_ALARM,   0); delay(650);
      yield();
    }
  }

  alarm_destination = "";
  alarm_message     = "";

  while(alarm_destination.length()==0) {
    alarm_destination = readEEPROM(0);
    yield();
  }

  while(alarm_message.length()==0 ) {
    alarm_message     = readEEPROM(20);
    yield();
  }

  if(power) {
    Serial.println("INFO: POWER: OK, Vbat: " + String(analogRead(PIN_BAT_VOLT) / BAT_DIVIDER, 2) + "v");
  } else {
    Serial.println("CRIT: NO POWER!, Vbat: " + String(analogRead(PIN_BAT_VOLT) / BAT_DIVIDER, 2) + "v");
    Serial.println("WARN: ONLY VISUAL ALERT (NO BEEPER, NO ALERT SMS)");
    alerted = true;
  }
  printHelp();
}

void loop() {
  
  if(SerialGsm.available()) { Serial.write(SerialGsm.read()); }
  if(Serial.available()) { 
    char cmd = Serial.read();

    switch (cmd) {
      case 'b': // (b)oot cause
        rst_info *resetInfo;
        resetInfo = ESP.getResetInfoPtr();
        Serial.print("INFO: Reset reason => ");
        switch(resetInfo->reason)
          {
            case 0 : Serial.println ("normal startup by power on");break;
            case 1 : Serial.println ("hardware watch dog reset");break;
            case 2 : Serial.println ("exception reset, GPIO status won’t change");break;
            case 3 : Serial.println ("software watch dog reset, GPIO status won’t change");break;
            case 4 : Serial.println ("software restart ,system_restart , GPIO status won’t change");break;
            case 5 : Serial.println ("wake up from deep-sleep");break;
            case 6 : Serial.println ("external system reset");break;
            default : Serial.println ("UNKNOWN");
          }
        beepOK();
        break;
      case 'h': // show (h)elp menu
        printHelp();
        beepOK();
        break;
      case 's': // (s)tatus
        Serial.println("INFO: battery volt.: " + String(analogRead(PIN_BAT_VOLT) / BAT_DIVIDER, 2) + "v");
        Serial.println("      alert mobile#: " + alarm_destination);
        Serial.println("      alert message: " + alarm_message);
        beepOK();
        break;
      case 'n': // set alert mobile (n)umber
        setAlertMobileNumber();
        break;
      case 'm': // set alert short (m)essage
        setAlertShortMessage();
        break;
      case 't': // send (t)est alert message
        Serial.println("INFO: Sending test alert message to " + alarm_destination);
        if(sendSMS(alarm_destination, alarm_message)) {
          Serial.println("INFO: SMS sent");
          beepOK();
        } else {
          Serial.println("ERRR: Couldn't send SMS!");
          beepERROR();
        }
        break;
      default:  // ! speak UPPERCASE to reach SIM800L (e.g. AT+CPIN?)
        SerialGsm.write(cmd);
        break;
    }
  }

  if(!power & !alerted) {
    beepERROR();
    float voltage = analogRead(PIN_BAT_VOLT) / BAT_DIVIDER;
    Serial.println("CRIT: POWERLOSS, Vbat: " + String(voltage, 2) + "v");
    if(voltage>3.3F) {
      // TODO: Ensure AT+CFUN=1 (Full Function)
      if(sendSMS(alarm_destination, alarm_message)) {
        Serial.println("INFO: SMS sent");
        beeper  = false;                             // beeper alarm off
        alerted = true;
        beepOK();                                    // beep confirmation
      } else  {
        Serial.println("ERRR: Couldn't send SMS!");
        beeper  = true;                              // beeper alarm on
        alerted = false;
      }
    } else {
      // TODO: can be removed when hardware protection is in place
      beeper = true;                                 // beeper alarm on
      Serial.println("CRIT: LOW BATTERY, Vbat: " + String(voltage, 2) + "v");
      Serial.println("                   SIM800L EMERGENCY SHUTDOWN");
      // TODO: Ensure AT+CFUN=4 instead (Disable phone both transmit and receive RF circuits.)
      poweroffSIM800L();
      alerted = true;
    }
    recovered = false;
  } else {
    if(power & alerted) {
      alerted = false;
    }
    if(power & !recovered) {
      beepOK();
      Serial.println("RECV: Power: OK, Vbat: " + String(analogRead(PIN_BAT_VOLT) / BAT_DIVIDER, 2) + "v");
      recovered = true;
    }
  }
}