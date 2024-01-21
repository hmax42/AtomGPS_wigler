//previous original version 1.2 rebased, modified

//use M5 lib or use Adafruit NeoPixel
//#define M5
//#define DEBUG

#define NO_OF_NODES 3
#define NODE_ID 3

#if NODE_ID==1
//scan all wifi channels
#define TYPEA
#define ATOMLITE
#elif NODE_ID==2
#define ATOMLITE
//workes fine with BT scanning
#define enableBLE
//scan wifi channels 1, 6, 11, 14 and ble or just wifi (code taken from j.hewitt)
#define enableSelectiveWifi
#elif NODE_ID==3
#define ATOMS3LITE
//crashes with BT scanning
//#define enableBLE
//scan wifi channels 1, 6, 11, 14 and ble or just wifi (code taken from j.hewitt)
//#define enableSelectiveWifi
#endif

#ifdef TYPEB
#define enableBLE
#define enableSelectiveWifi
#endif

#ifdef ATOMLITE
#define PIN_BTN 39
#define PIN_RGB 27
#define PIN_GPS_RX 22
#define PIN_SD_MISO 33
#define PIN_SD_MOSI 19
#define PIN_SD_CLK 23
#define PIN_SD_CS -1
#endif

#ifdef ATOMS3LCD
#define ATOMS3COMMON
#define PIN_RGB -1

#define PIN_LCD_MISO -1
#define PIN_LCD_MOSI 21
#define PIN_LCD_CLK 17
#define PIN_LCD_CS 15
#define PIN_LCD_BL 16
#define PIN_LCD_RS 33
#define PIN_LCD_RST 34
#endif

#ifdef ATOMS3LITE
#define ATOMS3COMMON
#define PIN_RGB 35
#endif

#ifdef ATOMS3COMMON
#include "M5AtomS3.h"
#define PIN_BTN 41
#define PIN_GPS_RX 5
#define PIN_SD_MISO 8
#define PIN_SD_MOSI 6
#define PIN_SD_CLK 7
#define PIN_SD_CS -1
#endif

#ifdef M5
#include "M5Atom.h"
#else
#include <Adafruit_NeoPixel.h>
#endif
#include <SPI.h>
#include "FS.h"
#include "SD.h"
#include <WiFi.h>
#include <TinyGPS++.h>

// LED
bool ledState = false;
bool buttonLedState = true;

#define COLOR_RED 0xff0000
#define COLOR_GREEN 0x00ff00
#define COLOR_BLUE 0x0000ff
#define COLOR_YELLOW 0xffff00
#define COLOR_PURPLE 0x800080
#define COLOR_CYAN 0x00ffff
#define COLOR_WHITE 0xffffff
#define COLOR_OFF 0x000000

TinyGPSPlus gps;

#ifdef enableBLE
//copied from j.hewitt rev3

#include <BLEDevice.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

BLEScan* pBLEScan;

#define mac_history_len 256

struct mac_addr {
  unsigned char bytes[6];
};

struct mac_addr mac_history[mac_history_len];
unsigned int mac_history_cursor = 0;

void save_mac(unsigned char* mac) {
  if (mac_history_cursor >= mac_history_len) {
    mac_history_cursor = 0;
  }
  struct mac_addr tmp;
  for (int x = 0; x < 6 ; x++) {
    tmp.bytes[x] = mac[x];
  }

  mac_history[mac_history_cursor] = tmp;
  mac_history_cursor++;
}

boolean seen_mac(unsigned char* mac) {

  struct mac_addr tmp;
  for (int x = 0; x < 6 ; x++) {
    tmp.bytes[x] = mac[x];
  }

  for (int x = 0; x < mac_history_len; x++) {
    if (mac_cmp(tmp, mac_history[x])) {
      return true;
    }
  }
  return false;
}

void print_mac(struct mac_addr mac) {
  for (int x = 0; x < 6 ; x++) {
    Serial.print(mac.bytes[x], HEX);
    Serial.print(":");
  }
}

boolean mac_cmp(struct mac_addr addr1, struct mac_addr addr2) {
  for (int y = 0; y < 6 ; y++) {
    if (addr1.bytes[y] != addr2.bytes[y]) {
      return false;
    }
  }
  return true;
}

void clear_mac_history() {
  struct mac_addr tmp;
  for (int x = 0; x < 6 ; x++) {
    tmp.bytes[x] = 0;
  }

  for (int x = 0; x < mac_history_len; x++) {
    mac_history[x] = tmp;
  }

  mac_history_cursor = 0;
}

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
      unsigned char mac_bytes[6];
      int values[6];

      if (6 == sscanf(advertisedDevice.getAddress().toString().c_str(), "%x:%x:%x:%x:%x:%x%*c", &values[0], &values[1], &values[2], &values[3], &values[4], &values[5])) {
        for (int i = 0; i < 6; ++i ) {
          mac_bytes[i] = (unsigned char) values[i];
        }

        if (!seen_mac(mac_bytes)) {
          save_mac(mac_bytes);

#ifdef DEBUG
          Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
#endif
          char utc[20];
          sprintf(utc, "%04d-%02d-%02d %02d:%02d:%02d",
                  gps.date.year(), gps.date.month(), gps.date.day(),
                  gps.time.hour(), gps.time.minute(), gps.time.second());
          float lat = gps.location.lat();
          float lon = gps.location.lng();
          float altitude = gps.altitude.meters();
          float accuracy = gps.hdop.hdop();

          String out = advertisedDevice.getAddress().toString().c_str() + String(",") + advertisedDevice.getName().c_str() + String(",") + "[BLE]," + utc + String(",") + String("0") + String(",") + String(advertisedDevice.getRSSI()) + String(",") + String(lat, 6) + String(",") + String(lon, 6) + String(",") + String(altitude, 2) + String(",") + String(accuracy, 2) + String(",") + String("BLE");
          logData(out);
        }
      }
    }
};

boolean sd_lock = false; //Set to true when writting to sd card

void await_sd() {
  while (sd_lock) {
#ifdef DEBUG
    Serial.println("await");
#endif
    delay(1);
  }
}
#endif

String fileName;

#ifndef M5
#if PIN_RGB>0
Adafruit_NeoPixel led = Adafruit_NeoPixel(1, PIN_RGB, NEO_GRB + NEO_KHZ800);
#endif
#endif

static unsigned long previousBlinkMillis = 0;

void blinkLED(uint32_t color, unsigned long interval) {
#if PIN_RGB>0
  unsigned long currentMillis = millis();

  if (currentMillis - previousBlinkMillis >= interval) {
    ledState = !ledState;
#ifdef M5
#ifdef ATOMLITE
    M5.drawpix(0, ledState ? color : COLOR_OFF);
#endif
#ifdef ATOMS3LITE
    AtomS3.dis.drawpix(0, ledState ? color : COLOR_OFF);  // Toggle red LED
#endif
#else
    led.setPixelColor(0, ledState ? color : COLOR_OFF);
    led.show();
#endif
    previousBlinkMillis = currentMillis;
  }
#endif
}

const int maxMACs = 75;
String macAddressArray[maxMACs];
int macArrayIndex = 0;

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");

#ifdef M5
  AtomS3.begin(true);
#else
  AtomS3.begin();
  led.begin();
  led.setPixelColor(0, 0x000000);
  led.show();
#endif
  SPI.begin(PIN_SD_CLK, PIN_SD_MISO, PIN_SD_MOSI, PIN_SD_CS);
  Serial.println("SPI Starting...");

  unsigned long startMillis = millis();
  const unsigned long blinkInterval = 500;
  bool ledState = false;

  if (!SD.begin()) {
    Serial.println("SD Card initialization failed!");
    while (millis() - startMillis < 5000) {  // Continue blinking for 5 seconds
      if (millis() - startMillis > blinkInterval) {
        startMillis = millis();
        blinkLED(COLOR_RED, blinkInterval);
      }
    }
    return;
  }

#ifdef M5
  AtomS3.dis.clear();  // Clear LED after blinking
#else
  led.setPixelColor(0, 0x000000);  // Clear LED after blinking
  led.show();
#endif
  Serial.println("SD Card initialized.");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("WiFi initialized.");

  Serial1.begin(9600, SERIAL_8N1, PIN_GPS_RX, -1);
  Serial.println("GPS Serial initialized.");

  waitForGPSFix();

  initializeFile();

#ifdef enableBLE
  Serial.println("Setting up Bluetooth scanning");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
//  pBLEScan->setExtendedScanCallback(new MyBLEExtAdvertisingCallbacks());
//  pBLEScan->setExtScanParams(); // use with pre-defined/default values, overloaded function allows to pass parameters
  pBLEScan->setActiveScan(false); //active scan uses more power, but get results faster
  pBLEScan->setInterval(50);
  pBLEScan->setWindow(40);  // less or equal setInterval value
#endif

  Serial.println("M5 Atom GPS Wigler started!");
}

int sat = -1;

void waitForGPSFix() {
  unsigned long lastBlink = 0;
  const unsigned long blinkInterval = 300;  // Time interval for LED blinking
  bool ledState = false;

  Serial.println("Waiting for GPS fix...");
  while (!gps.location.isValid()) {
    if (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
    if (gps.satellites.isValid() && sat != gps.satellites.value()) {
      sat = gps.satellites.value();
      Serial.println("Seeing " + String(sat) + " satellites!");
    }
    blinkLED(COLOR_PURPLE, 250);
  }
  Serial.println("GPS fix obtained.");
  blinkLED(COLOR_GREEN, 150);
}

void initializeFile() {
  int fileNumber = 0;
  bool isNewFile = false;

  // create a date stamp for the filename
  char fileDateStamp[16];
  sprintf(fileDateStamp, "%04d-%02d-%02d-",
          gps.date.year(), gps.date.month(), gps.date.day());

  do {
#if NO_OF_NODES > 1
    fileName = String("/") + String(NODE_ID) + "-wifi-scans-" + String(fileDateStamp) + String(fileNumber) + ".csv";
#else
    fileName = "/wifi-scans-" + String(fileDateStamp) + String(fileNumber) + ".csv";
#endif
    isNewFile = !SD.exists(fileName);
    fileNumber++;
  } while (!isNewFile);

  if (isNewFile) {
    File dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("WigleWifi-1.4,appRelease=1.300000,model=GPS Kit,release=1.100000F+00,device=M5ATOM,display=NONE,board=ESP32,brand=M5");
      dataFile.println("MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type");
      dataFile.close();
      Serial.println("New file created: " + fileName);
    }
  } else {
    Serial.println("Using existing file: " + fileName);
  }
}

#ifdef enableBLE
int wifi_scan_channel = 1; //The channel to scan (increments automatically)
#endif

void loop() {
#ifdef DEBUG
  Serial.println("Loop, PrintHeap");
  Serial.println(ESP.getFreeHeap());
#endif
  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  if (gps.location.isValid()) {
#ifdef DEBUG
    Serial.println("Valid");
#endif
    if (gps.satellites.isValid() && sat != gps.satellites.value()) {
      sat = gps.satellites.value();
      Serial.println("Seeing " + String(sat) + " satellites!");
    }
    blinkLED(COLOR_GREEN, 180);

    float lat = gps.location.lat();
    float lon = gps.location.lng();
    float altitude = gps.altitude.meters();
    float accuracy = gps.hdop.hdop();

    char utc[20];
    sprintf(utc, "%04d-%02d-%02d %02d:%02d:%02d",
            gps.date.year(), gps.date.month(), gps.date.day(),
            gps.time.hour(), gps.time.minute(), gps.time.second());
#ifdef DEBUG
    Serial.println("GPS data prepared");
#endif

    int savedNetworks = 0;
#ifdef enableBLE
    BLEScanResults foundDevices = pBLEScan->start(2.5, false);
    Serial.print("Devices found: ");
    Serial.println(mac_history_cursor);
    savedNetworks += mac_history_cursor;
    Serial.println("Scan done!");
    pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
#endif

#ifdef enableSelectiveWifi
    for (int y = 0; y < 4; y++) {
      Serial.print("Switching channel to ");
      switch (wifi_scan_channel) {
        case 1:
          wifi_scan_channel = 6;
          break;
        case 6:
          wifi_scan_channel = 11;
          break;
        case 11:
          wifi_scan_channel = 14;
          break;
        default:
          wifi_scan_channel = 1;
      }
      Serial.println(wifi_scan_channel);
#endif

      //scanNetworks(bool async, bool show_hidden, bool passive, uint32_t max_ms_per_chan)
#ifdef enableSelectiveWifi
      int numNetworks = WiFi.scanNetworks(false, true, false, 110, wifi_scan_channel);  //credit J.Hewitt
#else
      int numNetworks = WiFi.scanNetworks(false, true, false, 110);  //credit J.Hewitt
#endif
      for (int i = 0; i < numNetworks; i++) {
        String currentMAC = WiFi.BSSIDstr(i);
        if (isMACSeen(currentMAC)) {
          continue;
        }
        macAddressArray[macArrayIndex++] = currentMAC;
        if (macArrayIndex >= maxMACs) macArrayIndex = 0;

        String ssid = "\"" + WiFi.SSID(i) + "\"";  //sanitize SSID
        String capabilities = getAuthType(WiFi.encryptionType(i));
        int channel = WiFi.channel(i);
        int rssi = WiFi.RSSI(i);

        String dataString = currentMAC + String(",") + ssid + String(",") + capabilities + String(",") + utc + String(",") + String(channel) + String(",") + String(rssi) + String(",") + String(lat, 6) + String(",") + String(lon, 6) + String(",") + String(altitude, 2) + String(",") + String(accuracy, 2) + ",WIFI";

        savedNetworks++;
        logData(dataString);
      }
#ifdef enableSelectiveWifi
    }
#endif
    if (savedNetworks > 0) {
      blinkLED(COLOR_BLUE, 150);
    }
  } else {
    blinkLED(COLOR_PURPLE, 150);
  }
}

bool isMACSeen(const String& mac) {
  for (int i = 0; i < macArrayIndex; i++) {
    if (macAddressArray[i] == mac) {
      return true;
    }
  }
  return false;
}

void logData(const String& data) {
#ifdef enableBLE
  await_sd();
  sd_lock = true;
  int i = logData1(data);
  sd_lock = false;
  return i;
}

int logData1(const String& data) {
#endif
  File dataFile = SD.open(fileName, FILE_APPEND);
  if (dataFile) {
    dataFile.println(data);
    dataFile.close();
    Serial.println("Data written: " + data);
  } else {
    Serial.println("Error opening " + fileName);
    blinkLED(COLOR_RED, 500);
  }
}

const char* getAuthType(uint8_t wifiAuth) {
  switch (wifiAuth) {
    case WIFI_AUTH_OPEN:
      return "[OPEN]";
    case WIFI_AUTH_WEP:
      return "[WEP]";
    case WIFI_AUTH_WPA_PSK:
      return "[WPA_PSK]";
    case WIFI_AUTH_WPA2_PSK:
      return "[WPA2_PSK]";
    case WIFI_AUTH_WPA_WPA2_PSK:
      return "[WPA_WPA2_PSK]";
    case WIFI_AUTH_WPA2_ENTERPRISE:
      return "[WPA2_ENTERPRISE]";
    case WIFI_AUTH_WPA3_PSK:
      return "[WPA3_PSK]";
    case WIFI_AUTH_WPA2_WPA3_PSK:
      return "[WPA2_WPA3_PSK]";
    case WIFI_AUTH_WAPI_PSK:
      return "[WAPI_PSK]";
    default:
      return "[UNDEFINED]";
  }
}
