// TinyGPS Includes
#include <TinyGPSPlus.h>

// BLE Includes
#include <BLEDevice.h>

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
int bt_count = 0;
char my_timestamp[23] = {0};
int oled_mode = 0;

// BLE Variables
int scanTime = 5;  //In seconds
//int scanTime_ms = 5000;   //In milliseconds
int scanTime_ms = 2000;
int pauseTime_ms = 2000;
//BLEScan *pBLEScan;
// Variable for holding BLE Advert Data that is present within Callback Functions(???)
BLEAdvertData foundDevice;
// Count of Devices Seen
int dataCount = 0;

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

// Bluetooth SIG Standard UUIDs
// Location and Navigation Service UUID: 0x1819
#define LN_SERVICE_UUID "1819"
// Location and Navigation Feature Characteristic UUID: 0x2A6A
#define LN_FEATURE_CHAR_UUID "2A6A"
// Location and Navigation Position Characteristic UUID: 0x2A67
#define LN_POSITION_CHAR_UUID "2A67"

// Device Information Service UUID: 0x180A
#define DEVICE_INFO_SERVICE_UUID "180A"
// Alert Status Characteristic UUID: 0x2A3F
#define ALERT_STATUS_CHAR_UUID "2A3F"

// Custom service for system status (using a random UUID)
#define SYSTEM_STATUS_SERVICE_UUID "A7E6"
#define ERROR_STATUS_CHAR_UUID "A7E7"

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

// Function for Setting Up the Device
//  - Note: Added several 100ms(?) delys that appear to fix issues with configuring the BLE GATT Server
void setup() {
    Serial.begin(115200);
    SerialPort.begin(GPSBaud);
    delay(1000);  // Give more time for initialization

    // Initialize BLE with the device name directly
    BLE.init();
    //BLE.init(device_complete_name);
    delay(100);

    // Set advertising parameters
    BLE.configAdvert()->setMinInterval(100); // 100ms interval
    //BLE.configAdvert()->setAdvTimeout(0);    // 0 = advertise forever

    // Clear and reconfigure advertising data
    advert_data.clear();
    // Setup Advert Data with proper flags
    //advert_data.addFlags(GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);
    advert_data.addFlags(GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);
    //advert_data.addCompleteName(device_complete_name);
    //advert_data.addShortName(device_short_name);
    // Directly setting the device name using lower level functions
    BLE.setDeviceName(device_complete_name);
    //BLE.setDeviceAppearance(BLE_APPEARANCE_GENERIC_LOCATION_NAVIGATION_DISPLAY);  // Optional
    // Note: The above line should allow for masquerading as a different BLE device type

    // Setup Scan Data with proper service advertising
    scan_data.addCompleteServices(BLEUUID(LN_SERVICE_UUID));
    scan_data.addCompleteServices(BLEUUID(DEVICE_INFO_SERVICE_UUID));
    scan_data.addCompleteServices(BLEUUID(SYSTEM_STATUS_SERVICE_UUID));
    //scan_data.addCompleteServices(0);    // Note: Blank entry indiciates that there are no services with a certain UUID length
    // Not sure what the above really means..... Needs some uint8_t, can NOT just be blank

    // Configure the Advert Datas into the BLE object
    BLE.configAdvert()->setAdvData(advert_data);
    BLE.configAdvert()->setScanRspData(scan_data);
    
    // Configure BLE server before adding services
    BLE.configServer(3);
    delay(100);

    // Configure characteristics after services are added
    locationFeatureChar.setReadProperty(true);
    locationFeatureChar.setReadPermissions(GATT_PERM_READ);
    locationFeatureChar.setReadCallback(read_alert__callback);
    delay(100);
        
    locationPositionChar.setReadProperty(true);
    locationPositionChar.setNotifyProperty(true);
    locationPositionChar.setReadPermissions(GATT_PERM_READ);
    locationPositionChar.setCCCDCallback(notify_alert__callback);
    locationPositionChar.setReadCallback(read_alert__callback);
    delay(100);
    
    locationService.addCharacteristic(locationFeatureChar);
    locationService.addCharacteristic(locationPositionChar);
    // Note: One can use .setBufferLen() to set a specific buffer size
    delay(100);

    // Configure Error Reporting
    errorStatusChar.setReadProperty(true);
    errorStatusChar.setNotifyProperty(true);
    errorStatusChar.setReadPermissions(GATT_PERM_READ);
    errorStatusChar.setReadCallback(gps_error_status__callback);
    systemStatusService.addCharacteristic(errorStatusChar);
    delay(100);

    // Configure Alert Reporting
    alertStatusChar.setReadProperty(true);
    alertStatusChar.setNotifyProperty(true);
    alertStatusChar.setReadPermissions(GATT_PERM_READ);
    alertStatusChar.setCCCDCallback(notify_alert__callback);
    alertStatusChar.setReadCallback(read_alert__callback);
    deviceInfoService.addCharacteristic(alertStatusChar);
    delay(100);

    // Add services one by one with delays; May need to be at the end of the setup
    BLE.addService(locationService);
    delay(100);
    BLE.addService(deviceInfoService);
    delay(100);
    BLE.addService(systemStatusService);
    delay(100);

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
