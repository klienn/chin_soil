#include <WiFi.h>
#include <MQUnifiedsensor.h>
#include <WiFiClient.h>
#include <ESP32Servo.h>
#include "DHT.h"
#include <RtcDS1302.h>
#include <ArduinoJson.h>
#include <TFT_eSPI.h>
#include <BH1750.h>
#include <SPI.h>
#include <Wire.h>

#define BLYNK_PRINT Serial

#define BLYNK_TEMPLATE_ID "TMPL6pxKyoIck"
#define BLYNK_TEMPLATE_NAME "Soil Monitoring"
#define BLYNK_AUTH_TOKEN "hd-OiFPd8F3Mq-CCYynSFo9r3ykpHoOk"

#include <BlynkSimpleEsp32.h>

#define RelayPin 26
#define RelayPin2 27

#define pingPin 14
#define DHTPIN 13
#define gasPin 32
#define SOIL_PIN_1 35
#define SOIL_PIN_2 34
#define PUMP_THRESHOLD_DRY 25
#define PUMP_THRESHOLD_WET 80

#define DHTTYPE DHT22

#define Board ("ESP-32")
#define Type ("MQ-135")
#define Voltage_Resolution (5)
#define ADC_Bit_Resolution (12)
#define RatioMQ135CleanAir (3.6)

char ssid[] = "scam ni";
char pass[] = "Walakokabalo0123!";

unsigned long previousReadingMillis = 0;
const long readingInterval = 1000;

bool relayState = false;
bool relayState2 = false;
bool pirState = false;
bool smsSentWaterLevel = false;
bool smsSentSoilMoisture1 = false;
bool smsSentSoilMoisture2 = false;

int initialPosition = 0;
Servo myservo;

float waterLevel = 0;
float soilMoisture1 = 0;
float soilMoisture2 = 0;

byte omm = 99;
byte oss = 99;
bool initial = 1;
byte xcolon = 0;
unsigned int colour = 0;


TFT_eSPI tft = TFT_eSPI();
#define TFT_GREY 0x5AEB

ThreeWire myWire(4, 5, 2);  //dat, clk, rst
RtcDS1302<ThreeWire> rtc(myWire);

DHT dht(DHTPIN, DHTTYPE);
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, gasPin, Type);
BH1750 lightMeter;

void setup() {
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1);
  delay(3000);
  pinMode(RelayPin, OUTPUT);
  pinMode(RelayPin2, OUTPUT);
  Wire.begin();
  tft.init();
  tft.setRotation(3);
  tft.fillScreen(TFT_BLACK);
  tft.setTextColor(TFT_YELLOW, TFT_BLACK);  // Note: the new fonts do not draw the background colour
  MQ135.setRegressionMethod(1);
  MQ135.init();

  Serial.print("Calibrating please wait.");
  tft.print("Calibrating please wait.");
  float calcR0 = 0;

  for (int i = 1; i <= 10; i++) {
    MQ135.update();
    calcR0 += MQ135.calibrate(RatioMQ135CleanAir);
    Serial.print(".");
    tft.print(".");
  }

  MQ135.setR0(calcR0 / 10);
  Serial.println("  done!.");
  tft.print("  done!.");
  Serial.println(calcR0);
  tft.print(calcR0);

  if (isinf(calcR0)) {
    Serial.println("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    tft.print("Warning: Conection issue, R0 is infinite (Open circuit detected) please check your wiring and supply");
    while (1)
      ;
  }
  if (calcR0 == 0) {
    Serial.println("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    tft.print("Warning: Conection issue found, R0 is zero (Analog pin shorts to ground) please check your wiring and supply");
    while (1)
      ;
  }
  lightMeter.begin();
  Serial2.println("AT+CMGF=1");
  delay(500);

  Serial2.println("AT+CMGS=\"+639208771890\"");
  delay(500);
  rtc.Begin();

  // rtc.SetDateTime(RtcDateTime(__DATE__, __TIME__));
  // if (rtc.lostPower()) {
  //   Serial.println("RTC lost power, let's set the time!");
  //   rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
  // }

  dht.begin();
  tft.print("Connecting to ");
  tft.println(ssid);
  Blynk.begin(BLYNK_AUTH_TOKEN, ssid, pass);
  tft.print("Connected to ");
  tft.println(ssid);
  Serial.println("** Values ****");
  Serial.println("|  humidity | temperature | waterLevel(cm) | motion |            date           | servo |");
  tft.fillScreen(TFT_BLACK);
}

void sendSensor() {
  RtcDateTime now = rtc.GetDateTime();
  DynamicJsonDocument dhtData = readDHT();
  DynamicJsonDocument dataGas = readGas();
  DynamicJsonDocument dateData = readRTC(now);

  String year = dateData["year"];
  String month = dateData["month"];
  String day = dateData["day"];
  int hour = dateData["hour"];
  int minute = dateData["minute"];
  int second = dateData["second"];
  int unixtime = dateData["unixtime"];

  String formattedDate = year + "/" + month + "/" + day;

  float CO = dataGas["CO"];
  float CO2 = dataGas["CO2"];
  float NH4 = dataGas["NH4"];

  // if (
  //   (unixtime >= sched1 && unixtime <= sched1 + 3) || (unixtime >= sched2 && unixtime <= sched2 + 3) || (unixtime >= sched3 && unixtime <= sched3 + 3)) {
  //   //turn on servo
  //   relayState = true;
  // }

  float humidity = dhtData["humidity"];
  float temperature = dhtData["temperature"];
  float lightIntensity = readLightSensor();
  soilMoisture1 = mapFloat(readSoil(SOIL_PIN_1), 0, 4095.0, 100.0, 0);
  soilMoisture2 = mapFloat(readSoil(SOIL_PIN_2), 0, 4095.0, 100.0, 0);
  waterLevel = mapFloat(readUltrasonic(), 0, 127.0, 100.0, 0);
  if (soilMoisture1 < PUMP_THRESHOLD_DRY) {
    if (!smsSentSoilMoisture1) {
      sendSMS("Soil is dry.", 11);
      relayState = true;
    }
  } else if (soilMoisture1 >= PUMP_THRESHOLD_WET) {
    smsSentSoilMoisture1 = false;
    relayState = false;
  }

  if (soilMoisture2 < PUMP_THRESHOLD_DRY) {
    if (!smsSentSoilMoisture2) {
      sendSMS("Soil2 is dry.", 12);
      relayState2 = true;
    }
  } else if (soilMoisture2 >= PUMP_THRESHOLD_WET) {
    smsSentSoilMoisture2 = false;
    relayState2 = false;
  }


  if (waterLevel < 25) {
    if (!smsSentWaterLevel) {
      sendSMS("Water level less than 25%", 2);
    }
  } else if (waterLevel >= 90) {
    smsSentWaterLevel = false;
  }

  Blynk.virtualWrite(V0, humidity);
  Blynk.virtualWrite(V1, temperature);
  Blynk.virtualWrite(V2, waterLevel);
  Blynk.virtualWrite(V3, relayState);
  Blynk.virtualWrite(V4, lightIntensity);
  Blynk.virtualWrite(V5, soilMoisture1);
  Blynk.virtualWrite(V6, soilMoisture2);
  Blynk.virtualWrite(V7, CO);
  Blynk.virtualWrite(V8, CO2);
  Blynk.virtualWrite(V9, NH4);
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setCursor(4, 27);
  tft.println(formattedDate);
  tft.setTextFont(2);
  tft.println("Soil Monitoring");
  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextFont(1);
  tft.println("");
  tft.print("Soil Moisture 1 = ");
  tft.println(soilMoisture1);
  tft.print("Soil Moisture 2 = ");
  tft.println(soilMoisture2);
  tft.print("Temperature = ");
  tft.println(temperature);
  tft.print("Humidity = ");
  tft.println(humidity);
  tft.print("Light = ");
  tft.println(lightIntensity);
  tft.print("Water Level = ");
  tft.println(waterLevel);
  tft.print("Motion = ");
  tft.println(pirState ? "Detected    " : "Not Detected");
  tft.print("Relay State = ");
  tft.println(relayState ? "Closed   " : "Open");

  byte xpos = 6;
  byte ypos = 0;
  if (omm != second) {  // Only redraw every minute to minimise flicker
    // Uncomment ONE of the next 2 lines, using the ghost image demonstrates text overlay as time is drawn over it
    tft.setTextColor(0x39C4, TFT_BLACK);  // Leave a 7 segment ghost image, comment out next line!
    //tft.setTextColor(TFT_BLACK, TFT_BLACK); // Set font colour to black to wipe image
    // Font 7 is to show a pseudo 7 segment display.
    // Font 7 only contains characters [space] 0 1 2 3 4 5 6 7 8 9 0 : .
    tft.drawString("88:88:88", xpos, ypos, 4);  // Overwrite the text to clear it
    tft.setTextColor(0xFBE0);                   // Orange
    omm = minute;

    if (hour < 10) xpos += tft.drawChar('0', xpos, ypos, 4);
    xpos += tft.drawNumber(hour, xpos, ypos, 4);
    xcolon = xpos;
    xpos += tft.drawChar(':', xpos, ypos, 4);
    if (minute < 10) xpos += tft.drawChar('0', xpos, ypos, 4);
    xpos += tft.drawNumber(minute, xpos, ypos, 4);
    xcolon = xpos;
    xpos += tft.drawChar(':', xpos, ypos, 4);
    if (second < 10) xpos += tft.drawChar('0', xpos, ypos, 4);
    tft.drawNumber(second, xpos, ypos, 4);
  }

  Serial.print("|   ");
  Serial.print(humidity);
  Serial.print("   |   ");
  Serial.print(NH4);
  Serial.print("   |   ");
  Serial.print(CO2);
  Serial.print("   |   ");
  Serial.print(CO);
  Serial.print("   |   ");
  serializeJson(dateData, Serial);
  Serial.print("   |   ");
  Serial.print(relayState ? "Open" : "Closed");
  Serial.println("   |");
}

void loop() {
  unsigned long currentMillis = millis();
  Blynk.run();

  if (currentMillis - previousReadingMillis >= readingInterval) {
    previousReadingMillis = currentMillis;
    sendSensor();
    controlPump();
  };
}

void controlPump() {
  digitalWrite(RelayPin, relayState ? HIGH : LOW);
  digitalWrite(RelayPin2, relayState2 ? HIGH : LOW);
}

DynamicJsonDocument readRTC(const RtcDateTime& dt) {
  uint32_t unixTime = dt.Epoch32Time();

  DynamicJsonDocument doc(200);
  doc["year"] = dt.Year();
  doc["month"] = dt.Month();
  doc["day"] = dt.Day();
  doc["hour"] = dt.Hour();
  doc["minute"] = dt.Minute();
  doc["second"] = dt.Second();
  doc["unixtime"] = unixTime;

  return doc;
}

double readUltrasonic() {
  long duration, inches, cm;

  pinMode(pingPin, OUTPUT);
  digitalWrite(pingPin, LOW);
  delayMicroseconds(2);
  digitalWrite(pingPin, HIGH);
  delayMicroseconds(5);
  digitalWrite(pingPin, LOW);

  pinMode(pingPin, INPUT);
  duration = pulseIn(pingPin, HIGH);

  cm = microsecondsToCentimeters(duration);
  if (cm >= 1200) cm = 0;

  return cm > 127.0 ? 127.0 : cm;
}

long microsecondsToCentimeters(long microseconds) {
  return microseconds / 29 / 2;
}

DynamicJsonDocument readDHT() {
  float h = dht.readHumidity();
  float t = dht.readTemperature();
  DynamicJsonDocument doc(200);
  doc["humidity"] = h;
  doc["temperature"] = t;
  return doc;
}

int readSoil(int soil) {
  //4095 = dry = no contact
  int value = analogRead(soil);  // read the analog value from sensor
  return value;
}

float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void sendSMS(String message, int mode) {
  Serial2.print(message);
  Serial2.write(26);
  delay(500);
  if (mode == 11) {
    smsSentSoilMoisture1 = true;
  } else if (mode == 12) {
    smsSentSoilMoisture2 = true;
  } else {
    smsSentWaterLevel = true;
  };
}

DynamicJsonDocument readGas() {
  MQ135.update();

  MQ135.setA(605.18);
  MQ135.setB(-3.937);             // Configure the equation to calculate CO concentration value
  float CO = MQ135.readSensor();  // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(110.47);
  MQ135.setB(-2.862);                    // Configure the equation to calculate CO2 concentration value
  float CO2 = MQ135.readSensor() + 400;  // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  MQ135.setA(102.2);
  MQ135.setB(-2.473);              // Configure the equation to calculate NH4 concentration value
  float NH4 = MQ135.readSensor();  // Sensor will read PPM concentration using the model, a and b values set previously or from the setup

  DynamicJsonDocument doc(200);
  doc["CO"] = CO;
  doc["CO2"] = CO2;
  doc["NH4"] = NH4;
  delay(500);
  return doc;
}

double readLightSensor() {
  float lux = lightMeter.readLightLevel();
  return lux;
}