//use M5 lib or use Adafruit NeoPixel
//#define M5
//scan wifi channels 1, 6, 11, 14 and ble or just wifi (code taken from j.hewitt)
//#define TYPEB
#include "M5Atom.h"
#include <SD.h>
#ifdef M5
#else
#include <Adafruit_NeoPixel.h>
#endif
#include <SPI.h>
#include <TinyGPS++.h>
#include <WiFi.h>

// LED
bool ledState = false;
bool buttonLedState = true;

#define RED 0xff0000
#define GREEN 0x00ff00
#define BLUE 0x0000ff
#define YELLOW 0xffff00
#define PURPLE 0x800080
#define CYAN 0x00ffff
#define WHITE 0xffffff
#define OFF 0x000000

// GPS and Filesys
TinyGPSPlus gps;
char fileName[50];
const int maxMACs = 400;  // TESTING: buffer size
char macAddressArray[maxMACs][20];

#ifdef TYPEB
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

          //Serial.printf("Advertised Device: %s \n", advertisedDevice.toString().c_str());
          char utc[20];
          sprintf(utc, "%04d-%02d-%02d %02d:%02d:%02d",
                  gps.date.year(), gps.date.month(), gps.date.day(),
                  gps.time.hour(), gps.time.minute(), gps.time.second());
          float lat = gps.location.lat();
          float lon = gps.location.lng();
          float altitude = gps.altitude.meters();
          float accuracy = gps.hdop.hdop();

          String out = advertisedDevice.getAddress().toString().c_str() + String(",") + advertisedDevice.getName().c_str() + String(",") + "[BLE]," + utc + String(",0,") + String(advertisedDevice.getRSSI()) + String(",") + String(lat, 6) + String(",") + String(lon, 6) + String(",") + String(altitude, 2) + String(",") + String(accuracy, 2) + String(",BLE");
          logData(out);
        }
      }
    }
};

boolean sd_lock = false; //Set to true when writting to sd card

void await_sd() {
  while (sd_lock) {
    Serial.println("await");
    delay(1);
  }
}
#endif

String fileName;

#ifdef M5
#else
Adafruit_NeoPixel led = Adafruit_NeoPixel(1, 27, NEO_GRB + NEO_KHZ800);
#endif

const int maxMACs = 75;
String macAddressArray[maxMACs];
int macArrayIndex = 0;

// Network Scanning
const int popularChannels[] = { 1, 6, 11 };
const int standardChannels[] = { 2, 3, 4, 5, 7, 8, 9, 10 };
const int rareChannels[] = { 12, 13, 14 };  // Depending on region
int timePerChannel[14] = { 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200, 200 };

void setup() {
  Serial.begin(115200);
  Serial.println("Starting...");
#ifdef M5
  M5.begin(true, false, true);
#else
  led.begin();
  led.setPixelColor(0, 0x000000);
  led.show();
#endif
  SPI.begin(23, 33, 19, -1);

  while (!SD.begin(-1, SPI, 40000000)) {
    Serial.println("SD Card initialization failed! Retrying...");
    blinkLED(RED, 500);  // will hang here until SD is readable
  }
  Serial.println("SD Card initialized.");

  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  delay(100);
  Serial.println("WiFi initialized.");

  Serial1.begin(9600, SERIAL_8N1, 22, -1);
  Serial.println("GPS Serial initialized.");
  waitForGPSFix();
  initializeFile();

#ifdef TYPEB
  Serial.println("Setting up Bluetooth scanning");
  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan(); //create new scan
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setActiveScan(false); //active scan uses more power, but get results faster
  pBLEScan->setInterval(50);
  pBLEScan->setWindow(40);  // less or equal setInterval value

  Serial.println("M5 Atom GPS Wigler (Type B) started!");
#else
  Serial.println("M5 Atom GPS Wigler (Type A) started!");
#endif
}

void loop() {
  // Non-blocking blinks
  static unsigned long lastBlinkTime = 0;
  const unsigned long blinkInterval = 3000;  // GPS fix LED delay

  M5.update();

  if (M5.Btn.wasPressed()) {
    buttonLedState = !buttonLedState;
#ifdef M5		
    M5.dis.drawpix(0, buttonLedState ? BLUE : OFF);  // flash blue when toggled on
#else
	led.setPixelColor(0, buttonLedState ? BLUE : OFF);
    led.show();
#endif	
    delay(80);
  }

  while (Serial1.available() > 0) {
    gps.encode(Serial1.read());
  }

  if (gps.location.isValid()) {
    unsigned long currentMillis = millis();  //get the time here for accurate blinks
    if (currentMillis - lastBlinkTime >= blinkInterval && buttonLedState) {
#ifdef M5		
      M5.dis.drawpix(0, GREEN);  // Flash green without a static blink
      delay(120);
      M5.dis.clear();
#else
	led.setPixelColor(0, GREEN);
    led.show();
      delay(120);
	led.setPixelColor(0, OFF);
    led.show();
#endif	
      lastBlinkTime = currentMillis;
    }

    float lat = gps.location.lat();
    float lon = gps.location.lng();
    float altitude = gps.altitude.meters();
    float accuracy = gps.hdop.hdop();
    char utc[21];
    sprintf(utc, "%04d-%02d-%02d %02d:%02d:%02d", gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());

#ifdef TYPEB
    BLEScanResults foundDevices = pBLEScan->start(2.5, false);
    Serial.print("Devices found: ");
    Serial.println(mac_history_cursor);
    Serial.println("Scan done!");
    pBLEScan->clearResults();   // delete results fromBLEScan buffer to release memory
#endif

    // Dynamic async per-channel scanning
#ifdef TYPEB
    int channel = 1; 
    for (int y = 1; y <= 4; y++) {
      switch (channel) {
        case 1:
          channel = 6;
          break;
        case 6:
          channel = 11;
          break;
        case 11:
          channel = 14;
          break;
        default:
          channel = 1;
      }
#else
    for (int channel = 1; channel <= 14; channel++) {
#endif
      int numNetworks = WiFi.scanNetworks(true, true, false, timePerChannel[channel - 1], channel);
      for (int i = 0; i < numNetworks; i++) {
        char currentMAC[20];
        strcpy(currentMAC, WiFi.BSSIDstr(i).c_str());
        if (!isMACSeen(currentMAC)) {
          strcpy(macAddressArray[macArrayIndex++], currentMAC);
          if (macArrayIndex >= maxMACs) macArrayIndex = 0;
          char dataString[300];
          snprintf(dataString, sizeof(dataString), "%s,\"%s\",%s,%s,%d,%d,%.6f,%.6f,%.2f,%.2f,WIFI", currentMAC, WiFi.SSID(i).c_str(), getAuthType(WiFi.encryptionType(i)), utc, WiFi.channel(i), WiFi.RSSI(i), WiFi.RSSI(i), lat, lon, altitude, accuracy);
          logData(dataString);
          macArrayIndex = (macArrayIndex + 1) % maxMACs;
        }
      }
      // Update the scan duration for this channel based on the results
      updateTimePerChannel(channel, numNetworks);
    }
  } else {
    blinkLED(PURPLE, 500);
  }
}

void blinkLED(uint32_t color, unsigned long interval) {
  static unsigned long previousBlinkMillis = 0;
  unsigned long currentMillis = millis();

  if (currentMillis - previousBlinkMillis >= interval) {
    ledState = !ledState;
#ifdef M5
    M5.dis.drawpix(0, ledState ? color : OFF);
#else
	led.setPixelColor(0, ledState ? color : OFF);
    led.show();
#endif
    previousBlinkMillis = currentMillis;
  }
}

void waitForGPSFix() {
  Serial.println("Waiting for GPS fix...");
  while (!gps.location.isValid()) {
    if (Serial1.available() > 0) {
      gps.encode(Serial1.read());
    }
    blinkLED(PURPLE, 250);
  }
  Serial.println("GPS fix obtained.");
}

void initializeFile() {
  int fileNumber = 0;
  bool isNewFile = false;
  char fileDateStamp[16];
  sprintf(fileDateStamp, "%04d-%02d-%02d-", gps.date.year(), gps.date.month(), gps.date.day());
  do {
    snprintf(fileName, sizeof(fileName), "/AtomWigler-%s%d.csv", fileDateStamp, fileNumber);
    isNewFile = !SD.exists(fileName);
    fileNumber++;
  } while (!isNewFile);
  if (isNewFile) {
    File dataFile = SD.open(fileName, FILE_WRITE);
    if (dataFile) {
      dataFile.println("WigleWifi-1.4,appRelease=1.300000,model=GPS Kit,release=1.100000F+00,device=M5ATOM,display=NONE,board=ESP32,brand=M5");
      dataFile.println("MAC,SSID,AuthMode,FirstSeen,Channel,RSSI,CurrentLatitude,CurrentLongitude,AltitudeMeters,AccuracyMeters,Type");
      dataFile.close();
      Serial.println("New file created: " + String(fileName));
    }
  } else {
    Serial.println("Using existing file: " + String(fileName));
  }
}

bool isMACSeen(const char* mac) {
  for (int i = 0; i < macArrayIndex; i++) {
    if (strcmp(macAddressArray[i], mac) == 0) {
      return true;
    }
  }
  return false;
}

int logData(const char* data) {
#ifdef TYPEB
  await_sd();
  sd_lock = true;
  int i = logData1(data);
  sd_lock = false;
  return i;
}

int logData1(const char* data) {
#endif
  File dataFile = SD.open(fileName, FILE_APPEND);
  if (dataFile) {
    dataFile.println(data);
    dataFile.close();
    return 1;
  } else {
    Serial.println("Error opening " + String(fileName));
    blinkLED(RED, 500);
    return 0;
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

// TESTING: algo for timePerChan
void updateTimePerChannel(int channel, int networksFound) {
  const int FEW_NETWORKS_THRESHOLD = 1;
  const int MANY_NETWORKS_THRESHOLD = 5;
  const int TIME_INCREMENT = 50;  // how many ms to adjust by
  const int MAX_TIME = 400;
  const int MIN_TIME = 100;

  if (networksFound >= MANY_NETWORKS_THRESHOLD) {
    timePerChannel[channel - 1] += TIME_INCREMENT;
    if (timePerChannel[channel - 1] > MAX_TIME) {
      timePerChannel[channel - 1] = MAX_TIME;
    }
  } else if (networksFound <= FEW_NETWORKS_THRESHOLD) {
    timePerChannel[channel - 1] -= TIME_INCREMENT;
    if (timePerChannel[channel - 1] < MIN_TIME) {
      timePerChannel[channel - 1] = MIN_TIME;
    }
  }
}
