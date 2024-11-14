// TinyGPS Includes
#include <TinyGPSPlus.h>

// BLE Includes
#include <BLEDevice.h>

// GAP API Includes
#include "gap.h"
#include "gap_le.h"

// Software Serial Include
#include <SoftwareSerial.h>

// Configured from docs; url: https://www.amebaiot.com/en/amebad-bw16-arduino-getting-started/
static const int RXPin = 5, TXPin = 4;
static const uint32_t GPSBaud = 9600;
SoftwareSerial SerialPort(RXPin, TXPin);
// GPS data...
double lat;
double lng;
int month;
int day;
int year;
int hour;
int minute;
int second;
int alt;
double acc;
char timestamp_string[] = "%04d-%02d-%02d %02d:%02d:%02d";

// BLE Variables
//int scanTime = 5;  //In seconds
//int scanTime_ms = 5000;   //In milliseconds
//int pauseTime_ms = 2000;
//BLEScan *pBLEScan;
// Variable for holding BLE Advert Data that is present within Callback Functions(???)
//BLEAdvertData foundDevice;
// Count of Devices Seen
//int dataCount = 0;

// The TinyGPSPlus object
TinyGPSPlus gps;

// Structure for GPS Data
void dumpInfo() {

  lat = gps.location.lat();
  lng = gps.location.lng();
  alt = gps.altitude.meters();
  month = gps.date.month();
  day = gps.date.day();
  year = gps.date.year();
  hour = gps.time.hour();
  minute = gps.time.minute();
  second = gps.time.second();
  acc = gps.hdop.hdop();

  return;
}

//// BLE GATT Server Configuration and Setup
// Complete Server Name
const char* device_complete_name = "BLE GPS Reporter";
// Short Server Name
const char* device_short_name = "BGR Fox";
// Variable for Tracking Notification
bool device_notify_flag = false;

/// Callback Functions for BLE GATT Server
// Function for Alerting that a Characteristic has been Read
void read_alert__callback(BLECharacteristic* device_char, uint8_t connection_id) {
  printf("[!] Read Alert::Characteristic %s read by connection %d\n", device_char->getUUID().str(), connection_id);
  // Proof of read having happened
  device_char->writeString("Read Me");
}

// Function for Alerting on Change of Notification
void notify_alert__callback(BLECharacteristic* device_char, uint8_t connection_id, uint16_t device_cccd) {
  if (device_cccd & GATT_CLIENT_CHAR_CONFIG_NOTIFY) {
    printf("[!] Notify Alert::Notifications enabled on Characteristic %s for connection %d\n", device_char->getUUID().str(), connection_id);
    device_notify_flag = true;
  } else {
    printf("[!] Notify Alert::Notifications disabled on Characteristic %s for connection %d\n", device_char->getUUID().str(), connection_id);
    device_notify_flag = false;
  }
}

// Function for Providing GPS Error Status
void gps_error_status__callback(BLECharacteristic* device_char, uint8_t connection_id) {
  printf("[!] GPS Error Status::Characteristic %s read by connection %d\n", device_char->getUUID().str(), connection_id);
  //device_char->writeData8(systemErrors);
}

// Bluetooth Define Definitions
#define BLE_APPEARANCE_GENERIC_LOCATION_NAVIGATION_DISPLAY 0x1419

// Bluetooth SIG Standard UUIDs
#define LN_SERVICE_UUID "1819"    // Location and Navigation Service UUID: 0x1819
#define LN_FEATURE_CHAR_UUID "2A6A" // Location and Navigation Feature Characteristic UUID: 0x2A6A
#define LN_POSITION_CHAR_UUID "2A67" // Location and Navigation Position Characteristic UUID: 0x2A67
#define DEVICE_INFO_SERVICE_UUID "180A" // Device Information Service UUID: 0x180A
#define ALERT_STATUS_CHAR_UUID "2A3F" // Alert Status Characteristic UUID: 0x2A3F
#define ERROR_STATUS_CHAR_UUID "A7E7" // Error Status Characteristic UUID: 0xA7E7

// Custom service for system status (using a random UUID)
#define SYSTEM_STATUS_SERVICE_UUID "A7E6" // System Status Service UUID: 0xA7E6

// GATT Server components
BLEService locationService(LN_SERVICE_UUID);
BLEService deviceInfoService(DEVICE_INFO_SERVICE_UUID);
BLEService systemStatusService(SYSTEM_STATUS_SERVICE_UUID);

BLECharacteristic locationFeatureChar(LN_FEATURE_CHAR_UUID);
BLECharacteristic locationPositionChar(LN_POSITION_CHAR_UUID);
BLECharacteristic alertStatusChar(ALERT_STATUS_CHAR_UUID);
BLECharacteristic errorStatusChar(ERROR_STATUS_CHAR_UUID);

// Error tracking
uint8_t systemErrors = 0x00;
#define ERROR_GPS_DISCONNECTED 0x01
#define ERROR_GPS_NO_FIX      0x02
#define ERROR_LOW_MEMORY      0x04

// Structures for Advertising the BLE GATT Server
BLEAdvertData advert_data;  // Stucture skeleton for holding BLE GATT Server info; configured in init
BLEAdvertData scan_data;    // Structure skeleton for hodling BLE GATT Scan Response?

// Combine related characteristics setup
void setupLocationService() {
    locationFeatureChar.setReadProperty(true);
    locationFeatureChar.setReadPermissions(GATT_PERM_READ);
    locationFeatureChar.setReadCallback(read_alert__callback);
    
    locationPositionChar.setReadProperty(true);
    locationPositionChar.setNotifyProperty(true);
    locationPositionChar.setReadPermissions(GATT_PERM_READ);
    locationPositionChar.setCCCDCallback(notify_alert__callback);
    locationPositionChar.setReadCallback(read_alert__callback);
    
    locationService.addCharacteristic(locationFeatureChar);
    locationService.addCharacteristic(locationPositionChar);
}

// Function for Setting Up the Device
//  - Note: Added several 100ms(?) delys that appear to fix issues with configuring the BLE GATT Server
void setup() {
    Serial.begin(115200);
    SerialPort.begin(GPSBaud);
    delay(1000);

    // Initialize BLE and immediately set name
    BLE.init();
    delay(100);
    
    // Force name setting through multiple methods
    BLE.setDeviceName(device_complete_name);

    // Primary advert data
    advert_data.clear();
    advert_data.addFlags(GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);
    advert_data.addShortName(device_short_name);

    // Scan response
    scan_data.clear();
    scan_data.addCompleteName(device_complete_name);
    delay(100);
    //scan_data.addCompleteServices(BLEUUID(LN_SERVICE_UUID));  // Note: This forces not revealing the rest of the services?

    BLE.configAdvert()->setAdvData(advert_data);
    BLE.configAdvert()->setMinInterval(100);
    BLE.configAdvert()->setMaxInterval(200);
    BLE.configAdvert()->setScanRspData(scan_data);
    
    // Debug print
    Serial.print("Device name set to: ");
    Serial.println(device_complete_name);
    
    setupLocationService();

    // Add these lines:
    BLE.addService(locationService);
    
    // Start peripheral mode last
    BLE.beginPeripheral();
}

void loop() {
    // Check for GPS errors
    if (millis() > 5000 && gps.charsProcessed() < 10) {
        systemErrors |= ERROR_GPS_DISCONNECTED;
        errorStatusChar.writeData8(systemErrors);  // Changed from writeValue
    }

    // Update GPS position if available
    if (gps.location.isValid()) {
        // Pack GPS data into characteristic
        // Note: Using individual writes since BW16 doesn't support direct buffer writes
        locationPositionChar.writeData32((uint32_t)lat);
        locationPositionChar.writeData32((uint32_t)lng);
    } else {
        systemErrors |= ERROR_GPS_NO_FIX;
        errorStatusChar.writeData8(systemErrors);  // Changed from writeValue
    }

    // ... rest of loop code ...
}
