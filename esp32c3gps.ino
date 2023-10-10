#include <TinyGPS++.h>
#include <WiFi.h>
#include "FirebaseESP32.h"
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include "TimeLib.h"


#define FIREBASE_HOST "esp32-e513c-default-rtdb.firebaseio.com"   //Change to your Firebase RTDB project ID e.g. Your_Project_ID.firebaseio.com
#define FIREBASE_AUTH "wpwc1VOmlcsPsBm0mcoL9UyrEZYNvDSpqULlUBKH"  //Change to your Firebase RTDB secret password
#define WIFI_SSID "Net2.4"
#define WIFI_PASSWORD "12345678"

#define RXD2 5
#define TXD2 2
#define LED 10
#define LED2 4

HardwareSerial gpsSerial(1);

TinyGPSPlus gps;
const int offset = 1;

FirebaseData firebaseData1;
FirebaseData firebaseData2;

String path = "/GPS";
String nodeID = "Node1";  //This is this node ID to receive control
String stats = "STATS";
String lat = "LAT";
String lang = "LONG";
String alt = "ALT";
String speed = "SPEED";
String date = "DATE";



void setup() {
  Serial.begin(115200);
  Serial.println("boot complete");
  pinMode(LED, OUTPUT);
  digitalWrite(LED, HIGH);
  analogWrite(LED2, 20);
  delay(1000);

  Serial.println("setting gps");
  gpsSerial.begin(9600, SERIAL_8N1, RXD2, TXD2);
  gpsSerial.print("$PMTK251,57600*2C\r\n");

  gpsSerial.flush();
  gpsSerial.end();

  gpsSerial.begin(57600, SERIAL_8N1, RXD2, TXD2);
  gpsSerial.print("$PMTK220,200*1C\r\n");
  Serial.println("gps setup complete");

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to Wi-Fi");

  int t = 0;
  while (WiFi.status() != WL_CONNECTED && t < 10) {
    Serial.print(".");
    digitalWrite(LED, HIGH);
    delay(800);
    digitalWrite(LED, 0);
    t++;
  }
  Serial.println();


  if (WiFi.status() == WL_CONNECTED) {
    Serial.print("Connected with IP: ");
    Serial.println(WiFi.localIP());
    Serial.println();
    Firebase.begin(FIREBASE_HOST, FIREBASE_AUTH);
    Firebase.reconnectWiFi(true);
    setOTA();
    delay(500);
  }
  digitalWrite(LED, 0);
}
void setOTA() {
  ArduinoOTA
    .onStart([]() {
      String type;
      if (ArduinoOTA.getCommand() == U_FLASH)
        type = "sketch";
      else  // U_SPIFFS
        type = "filesystem";

      // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
      Serial.println("Start updating " + type);
    })
    .onEnd([]() {
      Serial.println("\nEnd");
    })
    .onProgress([](unsigned int progress, unsigned int total) {
      Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    })
    .onError([](ota_error_t error) {
      Serial.printf("Error[%u]: ", error);
      if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
      else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
      else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
      else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
      else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

  ArduinoOTA.begin();

  Serial.println("Ready");
}
static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gpsSerial.available())
      gps.encode(gpsSerial.read());
  } while ((millis() - start) > ms);
}
int d = 1000;
int st = millis();
bool historyEnabled = false;
int ch = millis();
bool stand = false;
bool status = true;  // means full on

void standBy(bool cond) {
  if (cond) {
    digitalWrite(LED, 0);
    delay(1000);
    Serial.println("Stand-by mode activated");
    digitalWrite(LED, HIGH);
    gpsSerial.print("$PMTK161,0*28\r\n");
    digitalWrite(LED, 0);
    status = false;
  } else {
    digitalWrite(LED, HIGH);
    delay(1000);
    Serial.println("Stand-by mode de-activated");
    digitalWrite(LED, 0);
    gpsSerial.print("$PMTK161,1*28\r\n");
    digitalWrite(LED, HIGH);
    status = true;
  }
}

void goSleep() {
  digitalWrite(LED, HIGH);
  delay(1000);
  Serial.println("Stand-by mode activated");
  gpsSerial.print("$PMTK161,0*28\r\n");
  delay(1000);
  digitalWrite(LED, 0);
  Serial.println("Going to sleep for 30 mins");
  Firebase.setString(firebaseData2, "/mode/status", String("sleep"));
  Firebase.setBool(firebaseData2, "/config/SLEEP", false);
  // esp_deep_sleep_enable_gpio_wakeup(1 << 2, ESP_GPIO_WAKEUP_GPIO_LOW);
  // esp_sleep_enable_timer_wakeup(timer * 1000 * 1000);
  esp_deep_sleep_start();
}


void loop() {
  if (gps.satellites.value() >= 8)
    analogWrite(LED2, 20);
  else
    analogWrite(LED2, 0);


  if (WiFi.status() == WL_CONNECTED) {
    ArduinoOTA.handle();
    if ((millis() - ch) > 8000) {
      Firebase.setBool(firebaseData2, "/mode/fix", (gps.location.age() < 2000));
      if (status)
        Firebase.setString(firebaseData2, "/mode/status", String("full-on"));
      else
        Firebase.setString(firebaseData2, "/mode/status", String("stand-by"));

      if (Firebase.getInt(firebaseData2, "/config/DELAY")) {
        d = firebaseData2.intData();
      }
      if (Firebase.getBool(firebaseData2, "/config/SLEEP")) {
        if (firebaseData2.boolData())
          goSleep();
      }
      if (Firebase.getBool(firebaseData2, "/config/HISTORY")) {
        historyEnabled = firebaseData2.boolData();
      }
      if (Firebase.getBool(firebaseData2, "/config/STANDBY")) {
        stand = firebaseData2.boolData();
        if (stand && status)
          standBy(true);

        if (!stand && !status)
          standBy(false);
      }
      ch = millis();
    }
    smartDelay(10);
    if (((millis() - st) > d) && status) {
     
      if (gps.location.isValid() && !stand) {
        Serial.println("Valid");
        digitalWrite(LED, HIGH);
        String a = String(String(gps.location.lat(), 6) + "," + String(gps.location.lng(), 6));

        FirebaseJson json;
        setTime(gps.time.hour(), gps.time.minute(), gps.time.second(), gps.date.day(), gps.date.month(), gps.date.year());
        unsigned long timestamp = now();

        json.add(stats, String(gps.satellites.value()));
        json.add(lat, String(gps.location.lat(), 8));
        json.add(lang, String(gps.location.lng(), 8));
        json.add(speed, String(gps.speed.kmph()));
        json.add(date, String(String(gps.date.day()) + "/" + String(gps.date.month()) + "/" + String(gps.date.year())));
        json.add("HOUR", String(timestamp));
        json.add(alt, String(gps.altitude.meters()));
        json.add("FIX", gps.location.age());
        json.add("COURSE", String(gps.course.deg(), 2));
        json.add("HDOP", String(gps.hdop.hdop(), 2));
        
        Serial.println("---");
        Firebase.RTDB.setJSON(&firebaseData2, "/GPS", &json);

        digitalWrite(LED, 0);
        if (historyEnabled && (gps.location.age() < 1000) && (gps.hdop.hdop() < 2) && (gps.satellites.value() > 6))
          Firebase.setString(firebaseData2, String(String("/history/") + String(timestamp)), a);

        st = millis();
      } else {
        Firebase.RTDB.setBool(&firebaseData2, "/config/VALID", gps.location.isValid());
        Serial.println("Invalid");
        digitalWrite(LED, HIGH);
      }
    }
  }
}
