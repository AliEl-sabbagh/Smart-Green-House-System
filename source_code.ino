#define BLYNK_TEMPLATE_ID ""
#define BLYNK_TEMPLATE_NAME "smart agriculture"
#define BLYNK_AUTH_TOKEN ""

#include <WiFi.h>
#include <BlynkSimpleEsp32.h>
#include <DHT.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <SPI.h>
#include <MFRC522.h>

#define SS_PIN  5  
#define RST_PIN 15 
MFRC522 rfid(SS_PIN, RST_PIN);
byte allowedUID[4] = { 147, 131, 49, 228 }; 
bool systemActive = false; 

LiquidCrystal_I2C lcd(0x27, 16, 2);

#define DHT_PIN 4
#define DHT_TYPE DHT11
#define SOIL_PIN 34
#define WATER_PIN 33

#define PUMP_IN1 26
#define PUMP_IN2 27
#define PUMP_ENA 14

#define FAN_IN3 16 
#define FAN_IN4 17
#define FAN_ENB 25

DHT dht(DHT_PIN, DHT_TYPE);

const float c_temp = -0.1539, c_humidity = 0.0818, c_soil = 0.2001, c_water = 0.0201, c_smoke = -0.0499, intercept = 10.0117;
const float W_TEMP = 0.1385, W_HUM = 0.0910, W_SOIL = 0.0419, W_SMOKE = 0.0630, W_TIME = 0.6656;

const int DRY_THRESHOLD = 3000;
const float TEMP_THRESHOLD = 30.0;

unsigned long lastSensorRead = 0;
const unsigned long sensorInterval = 2000; 

unsigned long lastBlynkAttempt = 0;
const unsigned long blynkRetryInterval = 30000;

float temp = 0.0, humidity = 0.0, phi = 0.0, hoursToDry = 0.0;
int soilRaw = 0, soilPercent = 0, waterLevel = 0, waterPercent = 0;
int smokePercent = 0;
String waterStatus = "";

unsigned long lastWateringTime = 0;
float hoursSinceWatering = 0.0;

const int freq = 1000;
const int resolution = 8;
const int pumpChannel = 0;
const int fanChannel = 1;

bool autoMode = true;
bool manualPump = false;
bool manualFan = false;

char ssid[] = "";
char pass[] = "";

float predictDryingHours(float t, float h, int sP, int wP, int smP) {
    float res = (c_temp * t) + (c_humidity * h) + (c_soil * sP) + (c_water * wP) + (c_smoke * smP) + intercept;
    return (res < 0) ? 0 : res;
}

float predictPHI(float t, float h, float soil, float smoke, float hours) {
    return (t * W_TEMP) + (h * W_HUM) + (soil * W_SOIL) + (smoke * W_SMOKE) + (hours * W_TIME);
}

void pumpOn() {
  digitalWrite(PUMP_IN1, HIGH);
  digitalWrite(PUMP_IN2, LOW);
  ledcWrite(pumpChannel, 255);
  if (Blynk.connected()) Blynk.virtualWrite(V4, 1);
}

void pumpOff() {
  digitalWrite(PUMP_IN1, LOW);
  digitalWrite(PUMP_IN2, LOW);
  ledcWrite(pumpChannel, 0);
  if (Blynk.connected()) Blynk.virtualWrite(V4, 0);
}

void fanOn() {
  digitalWrite(FAN_IN3, HIGH);
  digitalWrite(FAN_IN4, LOW);
  ledcWrite(fanChannel, 255);
  if (Blynk.connected()) Blynk.virtualWrite(V5, 1);
}

void fanOff() {
  digitalWrite(FAN_IN3, LOW);
  digitalWrite(FAN_IN4, LOW);
  ledcWrite(fanChannel, 0);
  if (Blynk.connected()) Blynk.virtualWrite(V5, 1);
}

void shutDownSystem() {
  pumpOff();
  fanOff();
}

BLYNK_WRITE(V6) { manualPump = param.asInt(); }
BLYNK_WRITE(V8) { manualFan  = param.asInt(); }
BLYNK_WRITE(V9) { autoMode   = param.asInt(); }

void setup() {
  Serial.begin(115200);

  pinMode(PUMP_IN1, OUTPUT); 
  pinMode(PUMP_IN2, OUTPUT);
  pinMode(FAN_IN3, OUTPUT);   
  pinMode(FAN_IN4, OUTPUT);
  pinMode(SOIL_PIN, INPUT);   
  pinMode(WATER_PIN, INPUT);

  ledcSetup(pumpChannel, freq, resolution);
  ledcAttachPin(PUMP_ENA, pumpChannel);
  ledcSetup(fanChannel, freq, resolution);
  ledcAttachPin(FAN_ENB, fanChannel);

  shutDownSystem(); 

  lcd.init(); 
  lcd.backlight();
  SPI.begin(); 
  rfid.PCD_Init();

  lcd.clear();
  lcd.setCursor(0, 0); 
  lcd.print("SYSTEM LOCKED");
  lcd.setCursor(0, 1); 
  lcd.print("Scan Key Card");

  dht.begin();
  WiFi.begin(ssid, pass);
  Blynk.config(BLYNK_AUTH_TOKEN);
}

void loop() {
  if (rfid.PICC_IsNewCardPresent() && rfid.PICC_ReadCardSerial()) {
    bool match = true;
    for (byte i = 0; i < 4; i++) {
      if (rfid.uid.uidByte[i] != allowedUID[i]) { 
        match = false; 
        break; 
      }
    }

    if (match) {
      systemActive = !systemActive; 
      lcd.clear();
      if (systemActive) {
        lcd.print("ACCESS GRANTED");
        shutDownSystem();          
        delay(1500); 
        lcd.clear();
        lcd.print("Welcome House");
        if (WiFi.status() == WL_CONNECTED) Blynk.connect();
      } else {
        lcd.print("SYSTEM LOCKED");
        shutDownSystem(); 
        delay(2000);
        lcd.setCursor(0, 1); 
        lcd.print("Scan Key Card");
      }
    }
    rfid.PICC_HaltA(); 
    rfid.PCD_StopCrypto1();
  }

  if (!systemActive) return; 

  unsigned long now = millis();

  if (Blynk.connected()) {
    Blynk.run();
  } else if (WiFi.status() == WL_CONNECTED && (now - lastBlynkAttempt >= blynkRetryInterval)) {
    lastBlynkAttempt = now;
    Blynk.connect();
  }

  if (now - lastSensorRead >= sensorInterval) {
    lastSensorRead = now;

    float newTemp = dht.readTemperature();
    float newHum = dht.readHumidity();
    if (!isnan(newTemp)) temp = newTemp;
    if (!isnan(newHum)) humidity = newHum;

    soilRaw = analogRead(SOIL_PIN);
    waterLevel = analogRead(WATER_PIN);
    soilPercent = constrain(map(soilRaw, 4095, 1500, 0, 100), 0, 100);
    waterPercent = constrain(map(waterLevel, 0, 2000, 0, 100), 0, 100);

    waterStatus = (waterPercent <= 10) ? "EMPTY" : (waterPercent <= 40) ? "LOW" : "OK";

    bool wantPump = (soilRaw > DRY_THRESHOLD && waterPercent > 10); 
    bool wantFan  = (temp > TEMP_THRESHOLD);

    if (autoMode) {
      if (soilRaw < 100) wantPump = false;
      if (wantPump) { 
        pumpOn(); 
        fanOff(); 
      } else { 
        pumpOff(); 
        if (wantFan) fanOn(); 
        else fanOff(); 
      }
    } else {
      if (manualPump) { 
        pumpOn(); 
        fanOff(); 
      } else { 
        pumpOff(); 
        if (manualFan) fanOn(); 
        else fanOff(); 
      }
    }

    if (wantPump) lastWateringTime = now;
    hoursSinceWatering = (now - lastWateringTime) / 3600000.0;
    
    phi = predictPHI(temp, humidity, soilPercent, smokePercent, hoursSinceWatering);
    hoursToDry = predictDryingHours(temp, humidity, soilPercent, waterPercent, smokePercent);

    if (Blynk.connected()) {
      Blynk.virtualWrite(V0, soilPercent);
      Blynk.virtualWrite(V1, temp);
      Blynk.virtualWrite(V3, waterPercent);
      Blynk.virtualWrite(V7, humidity);
      Blynk.virtualWrite(V11, waterStatus);
      Blynk.virtualWrite(V13, phi);
      Blynk.virtualWrite(V14, hoursToDry);
    }
  }
}
