#include <Arduino.h>
#include "SDL_Arduino_INA3221.h"
#include <GyverOLED.h>

#define SERIAL_SPEED          9600  //Serial speed
#define SERIAL_INTERVAL       2000  //and update interval (ms)

#define OLED_INTERVAL         3000  //Update channels on display (ms)

#define RELAY_LOAD_PIN_CH1    2     //Relay pin of load CH1
#define RELAY_LOAD_PIN_CH2    3     //Relay pin of load CH2
#define RELAY_CHARGE_PIN_CH1  4     //Relay pin of charger CH1
#define RELAY_CHARGE_PIN_CH2  5     //Relay pin of charger CH2
#define BTN_AUTO_PIN          6     //Button pin AUTO
#define BTN_VOFF_PIN          7     //Button pin VOFF
#define BTN_TEST_PIN          8     //Button pin TEST
#define BUZZ_PIN              9     //Buzzer pin

#define INA_CH1               1     
#define INA_CH2               2
#define INA_CH3               3
#define CHANNELS              3
#define ACC_ID_TIME           2000  // Time in ms to identify battery connected

#define NULL_MODE             0     //Battery not connected
#define PRESENT_MODE          1     //Battery connected
#define AUTO_MODE             2     //AUTO MODE: CHARGE->DISCHARGE->CHARGE->DONE
#define DISCHARGING_MODE      3     //DISCHARGE MODE: DISCHARGE->CHARGE->DONE
#define CHARGING_MODE         4     //CHARGING MODE: CHARGE->DONE
#define DONE_MODE             5     //DONE with HOLD DISPALY

#define VAVG                  1.114 //1.0 -- 1.2
#define Imin                  50   //CUT-OFF current in charging mode (mA)
float Vmin [] = {3.00, 2.90, 2.85, 2.80, 11.50, 10.80, 10.50};  // List turn off discharger
float Vmax [] = {4.15, 14.20};                                  // List turn off charger

byte activeChDisp = 0;
struct testerChannel{
  byte  chN;          // INA channel
  byte  loadPin;      // Load pin on relay
  byte  chargerPin;   // Charger pin on relay
  byte  mode;         // Channel mode
  float V;            // Volts
  float I;            // Current
  byte  idxVmin;      // MIN VOLTAGE
  byte  idxVmax;      // MAX VOLTAGE
  float capD;         // Discharge capacity
  float capC;         // Charge capacity
  float whD;          // Discharge capasity in watts
  float whC;          // Charge capasity in watts
};

SDL_Arduino_INA3221 ina_3221;
GyverOLED<SSD1306_128x64, OLED_NO_BUFFER> oled (0x3C); // OLED display
testerChannel tester [3] = {{INA_CH1,RELAY_LOAD_PIN_CH1,RELAY_CHARGE_PIN_CH1,NULL_MODE,0,0,0,0,0,0,0,0}, //Channel1
                          {INA_CH2,RELAY_LOAD_PIN_CH2,RELAY_CHARGE_PIN_CH2,NULL_MODE,0,0,0,0,0,0,0,0},  //Channel2
                          {INA_CH3,0,0,NULL_MODE,0,0,0,0,0,0,0,0}};                                     //Channel3

unsigned long prevMillis;
unsigned long testStart;

void batteryStatus();
void oledStatusBar(byte);
byte buttonsMenu();
void relayControl(byte);
void batteryData();
void sendDataSerial();
void sendOledData();
void oledBottomBar();
void oledActiveChannel();

void initDevices(){
  for (byte i = 0; i < CHANNELS-1; i++){  //init relay ch1 and ch2
    pinMode(tester[i].loadPin, OUTPUT);
    pinMode(tester[i].chargerPin, OUTPUT);
    digitalWrite(tester[i].loadPin, HIGH);
    digitalWrite(tester[i].chargerPin, HIGH);
  }
  pinMode(BTN_AUTO_PIN, INPUT_PULLUP);    // Automatic mode button
  pinMode(BTN_VOFF_PIN, INPUT_PULLUP);    // Choose cutoff Voltage
  pinMode(BTN_TEST_PIN, INPUT_PULLUP);    // Test mode button
  pinMode(BUZZ_PIN, OUTPUT);              // Buzzer init
  Serial.begin(SERIAL_SPEED);             // Open serial port
  ina_3221.begin();                       // INA3221 init
  oled.init();                            // Display init

}

void setup() {  
  initDevices();
  oled.clear();
  oled.setScale(2);
  oled.setCursor(7, 3);
  oled.println("LOADING...");
  delay(1000);
  oled.clear();
  oled.setScale(2);
  oled.setCursor(0, 1);
  oled.println("V: ");
  oled.println("I: ");
  oled.println("mA:");
  Serial.println("Start testing...");
  Serial.println("Channel - Time - V - A - mAh - Wh");
  testStart = millis();  //Time test start
  prevMillis = millis(); //First step's time
}

void loop() {
  batteryStatus();
  buttonsMenu();
  batteryData();
  sendDataSerial();
  sendOledData();
  oledActiveChannel();
}
/// @brief Check status of channels
void batteryStatus(){
  static unsigned long millisCH [CHANNELS] = {0,0,0};
  for (byte i = 0; i < CHANNELS; i++){
    if (tester[i].mode <= PRESENT_MODE){
      if (ina_3221.getBusVoltage_V(tester[i].chN) > 1){
        if (millisCH[i] == 0){
          millisCH[i] = millis();
        }
        if (millis() - millisCH[i] > ACC_ID_TIME && tester[i].mode != PRESENT_MODE){
          tester[i].mode = PRESENT_MODE;
          Serial.print("CHANNEL ");
          Serial.print(i+1);
          Serial.println(": BATTERY CONNECTED!");
          oledStatusBar(i);
        }
      } else if (tester[i].mode == PRESENT_MODE){
        tester[i].mode = NULL_MODE;
        Serial.print("CHANNEL ");
        Serial.print(i+1);
        Serial.println(": BATTERY DISCONNECTED!");
        oledStatusBar(i);
        millisCH[i] = 0;
      }
    }
  }
}

void oledStatusBar(byte i){
  oled.setScale(1);
  oled.invertText(0);
  switch (i)
  {
  case 0:
    oled.setCursor(0,0);
    (tester[i].mode) ? oled.print("CH1") : oled.print("   ");
    break;
  case 1:
    oled.setCursor(24,0);
    (tester[i].mode) ? oled.print("CH2") : oled.print("   ");
    break;
  case 2:
    oled.setCursor(48,0);
    (tester[i].mode) ? oled.print("CH3") : oled.print("   ");
    break;
  default:
    break;
  }
}

/// @brief Check buttons for connected channel
byte buttonsMenu(){
  if (tester[activeChDisp].mode == PRESENT_MODE || tester[activeChDisp].mode == DONE_MODE){
    if (digitalRead(BTN_TEST_PIN) == 0){
      tone(BUZZ_PIN,200,100);
      if (tester[activeChDisp].mode == DONE_MODE){
        if (tester[activeChDisp].capD){
          tester[activeChDisp].capD = 0;
          return 0;
        } else if (tester[activeChDisp].capC){
          tester[activeChDisp].capC = 0;
          return 0;
        } else {
          tester[activeChDisp].mode = PRESENT_MODE;
          return 0;
        }
      }
      tester[activeChDisp].mode = DISCHARGING_MODE;
      tester[activeChDisp].capD = 0;
      tester[activeChDisp].whD = 0;
      tester[activeChDisp].capC = 0;
      tester[activeChDisp].whC = 0;
      relayControl(activeChDisp);
      Serial.print("CHANNEL ");
      Serial.print(activeChDisp+1);
      Serial.println(": BATTERY DISCHARGING!");
    } else if (digitalRead(BTN_AUTO_PIN) == 0){
      tone(BUZZ_PIN,200,100);
      if (tester[activeChDisp].mode == DONE_MODE){
        if (tester[activeChDisp].capD){
          tester[activeChDisp].capD = 0;
          return 0;
        } else if (tester[activeChDisp].capC){
          tester[activeChDisp].capC = 0;
          return 0;
        } else {
          tester[activeChDisp].mode = PRESENT_MODE;
          return 0;
        }
      }
      tester[activeChDisp].mode = AUTO_MODE;
      tester[activeChDisp].capD = 0;
      tester[activeChDisp].whD = 0;
      tester[activeChDisp].capC = 0;
      tester[activeChDisp].whC = 0;
      relayControl(activeChDisp);
      Serial.print("CHANNEL ");
      Serial.print(activeChDisp+1);
      Serial.println(": BATTERY CHARGING (AUTO)!");
    } else if (digitalRead(BTN_VOFF_PIN) == 0){
      tone(BUZZ_PIN,200,200);
      if (tester[activeChDisp].mode == DONE_MODE){
        if (tester[activeChDisp].capD){
          tester[activeChDisp].capD = 0;
          return 0;
        } else if (tester[activeChDisp].capC){
          tester[activeChDisp].capC = 0;
          return 0;
        } else {
          tester[activeChDisp].mode = PRESENT_MODE;
          return 0;
        }
      }
      tester[activeChDisp].idxVmin++;
      if (tester[activeChDisp].idxVmin == sizeof(Vmin)/sizeof(float)){
        tester[activeChDisp].idxVmin = 0;
      }
      Serial.print("CHANNEL ");
      Serial.print(activeChDisp+1);
      Serial.print(": Vmin set to ");
      Serial.println(Vmin[tester[activeChDisp].idxVmin]);
    }
  }
return 0;
}
/// @brief Control channel's relay module
void relayControl(byte i){
    if (tester[i].mode == AUTO_MODE || tester[i].mode == CHARGING_MODE){
      digitalWrite(tester[i].loadPin, HIGH);
      tone(BUZZ_PIN,200,500);
      digitalWrite(tester[i].chargerPin, LOW);
    } else if (tester[i].mode == DISCHARGING_MODE){
      digitalWrite(tester[i].chargerPin, HIGH);
      tone(BUZZ_PIN,200,500);
      digitalWrite(tester[i].loadPin, LOW);
    } else {
      digitalWrite(tester[i].loadPin, HIGH);
      digitalWrite(tester[i].chargerPin, HIGH);
      tone(BUZZ_PIN,200,500);
    }
}
/// @brief Collecting battery data from tester channels
void batteryData(){
  for (byte i = 0; i < CHANNELS; i++){
    if (tester[i].mode > NULL_MODE){
      tester[i].V = ina_3221.getBusVoltage_V(tester[i].chN);
      tester[i].I = ina_3221.getCurrent_mA(tester[i].chN);
    } else {
      tester[i].V = 0;
      tester[i].I = 0;
    }
    if (tester[i].mode == AUTO_MODE || tester[i].mode == CHARGING_MODE){
      tester[i].capC += tester[i].I * (millis() - prevMillis) / 3600000;
      tester[i].whC += tester[i].I * tester[i].V * (millis()-prevMillis) / 3600000 / 1000;
      if ((tester[i].V > Vmax[tester[i].idxVmax] && tester[i].I < Imin) || tester[i].V < 1){
        if (tester[i].mode == AUTO_MODE){
          tester[i].mode = DISCHARGING_MODE;
          Serial.print("CHANNEL ");
          Serial.print(i+1);
          Serial.println(": BATTERY CHARGED (AUTO) - DISCHARGING!");
        } else {
          tester[i].mode = DONE_MODE;
          Serial.print("CHANNEL ");
          Serial.print(i+1);
          Serial.println(": BATTERY CHARGED - DONE!");
        }
        relayControl(i);
      }
    } else if (tester[i].mode == DISCHARGING_MODE){
      tester[i].capD += tester[i].I * (millis() - prevMillis) / 3600000;
      tester[i].whD += tester[i].I * tester[i].V * (millis()-prevMillis) / 3600000 / 1000;
      if (tester[i].V < Vmin[tester[i].idxVmin]){
        tester[i].capC = 0;
        tester[i].whC = 0;
        tester[i].mode = CHARGING_MODE;
        relayControl(i);
        Serial.print("CHANNEL ");
        Serial.print(i+1);
        Serial.println(": BATTERY DISCHARGED - CHARGING!");
      }
    }
  }
  prevMillis = millis();
}
/// @brief "Time -- Channel -- V -- A -- mAh -- Wh" ///
void sendDataSerial() {
  static uint32_t tmrS;
  if (millis() - tmrS >= SERIAL_INTERVAL) {
    for (byte i = 0; i < CHANNELS; i++){
      if (tester[i].mode > PRESENT_MODE && tester[i].mode < DONE_MODE){
        Serial.print((millis() - testStart) / 1000);
        Serial.print(" -- CH");
        Serial.print(i+1);
        Serial.print(" -- ");
        Serial.print(tester[i].V, 3);
        Serial.print(" -- ");
        Serial.print(tester[i].I, 0);
        Serial.print(" -- ");
        if (tester[i].mode == DISCHARGING_MODE){
          Serial.print(tester[i].capD, 0);
          Serial.print(" -- ");
          Serial.println(tester[i].whD, 3);
        } else {
          Serial.print(tester[i].capC, 0);
          Serial.print(" -- ");
          Serial.println(tester[i].whC, 3);
        }
      }
    }
    tmrS = millis();
  }
}
/// @brief Send data to oled display///
void sendOledData() {
  char V_out[6];
  char I_out[6];
  char Cap_out[7];
  oledBottomBar();
  switch (activeChDisp){
    case 0:
      oled.setScale(1);
      oled.setCursor(78,0);
      oled.print("   I   ");
      oled.setScale(2);
      oled.setCursor(65,1);
      (tester[activeChDisp].V < 10) ? dtostrf(tester[activeChDisp].V, 5, 3, V_out) : dtostrf(tester[activeChDisp].V, 5, 2, V_out);
      oled.print(V_out);
      oled.setCursor(65,3);
      dtostrf(tester[activeChDisp].I, 5, 0, I_out);
      oled.print(I_out);
      oled.setCursor(53,5);
      if (tester[activeChDisp].mode == DISCHARGING_MODE){
        dtostrf(tester[activeChDisp].capD, 6, 0, Cap_out);
      } else if (tester[activeChDisp].mode == AUTO_MODE || tester[activeChDisp].mode == CHARGING_MODE){
        dtostrf(tester[activeChDisp].capC, 6, 0, Cap_out);
      } else if (tester[activeChDisp].mode == DONE_MODE){
        if(tester[activeChDisp].capD){
          dtostrf(tester[activeChDisp].capD, 6, 0, Cap_out);
        } else {
          dtostrf(tester[activeChDisp].capC, 6, 0, Cap_out);
        }
      } else {
        dtostrf(0, 6, 0, Cap_out);
      }
      oled.print(Cap_out);
      break;
    case 1:
      oled.setScale(1);
      oled.setCursor(78,0);
      oled.print("  I I  ");
      oled.setScale(2);
      oled.setCursor(65,1);
      (tester[activeChDisp].V < 10) ? dtostrf(tester[activeChDisp].V, 5, 3, V_out) : dtostrf(tester[activeChDisp].V, 5, 2, V_out);
      oled.print(V_out);
      oled.setCursor(65,3);
      dtostrf(tester[activeChDisp].I, 5, 0, I_out);
      oled.print(I_out);
      oled.setCursor(53,5);
      if (tester[activeChDisp].mode == DISCHARGING_MODE){
        dtostrf(tester[activeChDisp].capD, 6, 0, Cap_out);
      } else if (tester[activeChDisp].mode == AUTO_MODE || tester[activeChDisp].mode == CHARGING_MODE){
        dtostrf(tester[activeChDisp].capC, 6, 0, Cap_out);
      } else if (tester[activeChDisp].mode == DONE_MODE){
        if(tester[activeChDisp].capD){
          dtostrf(tester[activeChDisp].capD, 6, 0, Cap_out);
        } else {
          dtostrf(tester[activeChDisp].capC, 6, 0, Cap_out);
        }
      } else {
        dtostrf(0, 6, 0, Cap_out);
      }
      oled.print(Cap_out);
      break;
    case 2:
      oled.setScale(1);
      oled.setCursor(78,0);
      oled.print(" I I I ");
      oled.setScale(2);
      oled.setCursor(65,1);
      (tester[activeChDisp].V < 10) ? dtostrf(tester[activeChDisp].V, 5, 3, V_out) : dtostrf(tester[activeChDisp].V, 5, 2, V_out);
      oled.print(V_out);
      oled.setCursor(65,3);
      dtostrf(tester[activeChDisp].I, 5, 0, I_out);
      oled.print(I_out);
      oled.setCursor(53,5);
      if (tester[activeChDisp].mode == DISCHARGING_MODE){
        dtostrf(tester[activeChDisp].capD, 6, 0, Cap_out);
      } else if (tester[activeChDisp].mode == AUTO_MODE || tester[activeChDisp].mode == CHARGING_MODE){
        dtostrf(tester[activeChDisp].capC, 6, 0, Cap_out);
      } else if (tester[activeChDisp].mode == DONE_MODE){
        if(tester[activeChDisp].capD){
          dtostrf(tester[activeChDisp].capD, 6, 0, Cap_out);
        } else {
          dtostrf(tester[activeChDisp].capC, 6, 0, Cap_out);
        }
      } else {
        dtostrf(0, 6, 0, Cap_out);
      }
      oled.print(Cap_out);
      break;
  }
}

void oledBottomBar(){
  oled.invertText(0);
  oled.setScale(1);
  oled.setCursor(4,7);
  if (tester[activeChDisp].mode == AUTO_MODE){
    oled.print(" CHARGING (AUTO)... ");
  } else if (tester[activeChDisp].mode == DISCHARGING_MODE){
    oled.print("   DISCHARGING...   ");
  } else if (tester[activeChDisp].mode == CHARGING_MODE){
    oled.print("     CHARGING...    ");
  } else if (tester[activeChDisp].mode == DONE_MODE){
    oled.invertText(1);
    oled.print("    !!! DONE !!!    ");
  } else if (tester[activeChDisp].mode == PRESENT_MODE){
    oled.print("  AUTO  ");
    oled.setCursor(52,7);
    if (Vmin[tester[activeChDisp].idxVmin] < 10){
      oled.print(Vmin[tester[activeChDisp].idxVmin],2);  
    } else {
      oled.print(Vmin[tester[activeChDisp].idxVmin],1);
    }
    oled.setCursor(76,7);
    oled.print("  TEST  ");
  } else {
    oled.print("                     ");
  }
}

void oledActiveChannel(){
  static uint32_t tmrD;
  if (millis() - tmrD >= OLED_INTERVAL) {
    tmrD = millis();
    if (tester[0].mode || tester[1].mode || tester[2].mode) {
      while(true) {
        if (tester[activeChDisp].mode == DONE_MODE || tester[activeChDisp].mode == PRESENT_MODE){
          break;
        } else {
          activeChDisp++;
          if (activeChDisp == CHANNELS){
            activeChDisp = 0;
          }
          if (tester[activeChDisp].mode) {
            break;
          }
        }
      }
    }
  }
}