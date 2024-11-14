// SQLite Includes
//#include <sqlite3.h>

// TinyGPS Includes
#include <TinyGPSPlus.h>

// SSD1306/OLED Library Includes
//#include <Adafruit_SSD1306.h>
//#define OLED
//#define OLED_OFFSET 5
//#ifdef OLED
//#include <Wire.h>               // Only needed for Arduino 1.6.5 and earlier
//#include "SSD1306Wire.h"        // legacy: #include "SSD1306.h"
//#endif

// SD Libraries
//#include <SD.h>

// Bluetooth Libraries for BW16 AMBE IoT
//#include <BLERemoteDescriptor.h>
//#include <BLEUUID.h>
//#include <HID.h>
//#include <BLEHIDKeyboard.h>
//#include <BLERemoteCharacteristic.h>
//#include <BLEBeacon.h>
#include <BLEDevice.h>
//#include <BLEScan.h>
//#include <BLEHIDDevice.h>
//#include <BLECharacteristic.h>
//#include <BLEWifiConfigService.h>
//#include <BLEClient.h>
//#include <BLEService.h>
//#include <BLERemoteService.h>
//#include <BLEHIDMouse.h>
//#include <BLEAddr.h>
//#include <BLEConnect.h>
//#include <BLEHIDGamepad.h>
//#include <BLEAdvert.h>
//#include <BLESecurity.h>
//#include <BLEAdvertData.h>

// Additional Include
//#include "FS.h"
//#include "SPI.h"

// Software Serial Include
#include <SoftwareSerial.h>

// Variable Definitions
char sql[1024];
//sqlite3 *db1;
//sqlite3_stmt *res;
const char *tail;
int rc;
//HardwareSerial SerialPort(2);
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

// Structure for BLE Device Data
String device_name;
uint16_t device_appearance;
uint16_t device_manufacturer;
uint8_t device_manu_data_length;
uint8_t* device_manu_data;
uint8_t device_flags;
uint8_t device_service_count;
BLEUUID* device_service_list;
T_GAP_ADV_EVT_TYPE device_advert_type;
T_GAP_REMOTE_ADDR_TYPE device_addr_type;
BLEAddr device_address;
int8_t device_rssi;
int8_t device_tx_power;

// Function for Extracting BLE Advert Data
void extractBLE(BLEAdvertData deviceAdvert) {
    //++dataCount;
    // Pretty prints Advert Scan Info
    //BLE.configScan()->printScanInfo(p_data);
    //  - Note: Columns are (1) ADVType, (2) AddrType, (3) BT_Addr, (4) rssi

    // Scan and Parse the Found Device Information
    //deviceAdvert.parseScanInfo(p_data);   // Necessary to call BEFORE extracting BLE information?
    //// Enumerate through each device's information [ DEBUGGING ]
    // Check Device Name
    if (deviceAdvert.hasName()) {
      device_name = deviceAdvert.getName();
    } else {
      device_name = "-=[None]=-";
    }
    // Check Device UUID
    if (deviceAdvert.hasUUID()) {
      Serial.println("Device UUID exists.... Not sure how to access");
    } else {
      Serial.println("Device does not have a UUID");
    }
    // Check Device Manufacturer
    if (deviceAdvert.hasManufacturer()) {
      device_manufacturer = deviceAdvert.getManufacturer();
      device_manu_data_length =  deviceAdvert.getManufacturerDataLength();
      device_manu_data = deviceAdvert.getManufacturerData();
    } else {
      device_manufacturer = (uint16_t) 65536;  // Set to Max for 16-bit number
      device_manu_data_length =  (uint8_t) 255;   // Set to Max and HOPEFULLY un-used value? Check BT Spec...
      device_manu_data = (uint8_t*) 1337;
    }
    // Check BLE Advert Data Flags
    if (deviceAdvert.hasFlags()) {
      device_flags = deviceAdvert.getFlags();
    } else {
      device_flags = (uint8_t) 255;
    }
    // Check Advert Type
    device_advert_type = deviceAdvert.getAdvType();
    // Check Address Type
    device_addr_type = deviceAdvert.getAddrType();
    // Check Device Address
    device_address = deviceAdvert.getAddr();
    // Check Device RSSI
    device_rssi = deviceAdvert.getRSSI();
    // Check Device Service Count
    device_service_count = deviceAdvert.getServiceCount();
    device_service_list = deviceAdvert.getServiceList();
    // Check Device TX Power
    device_tx_power = deviceAdvert.getTxPower();
    // Check Device Appearance
    device_appearance = deviceAdvert.getAppearance();
}

// Function for Printing Collected GPS Information
void gpsInfoDump() {
  Serial.println(gps.location.lat(), 6); // Latitude in degrees (double)
  Serial.println(gps.location.lng(), 6); // Longitude in degrees (double)
  Serial.print(gps.location.rawLat().negative ? "-" : "+");
  Serial.println(gps.location.rawLat().deg); // Raw latitude in whole degrees
  Serial.println(gps.location.rawLat().billionths);// ... and billionths (u16/u32)
  Serial.print(gps.location.rawLng().negative ? "-" : "+");
  Serial.println(gps.location.rawLng().deg); // Raw longitude in whole degrees
  Serial.println(gps.location.rawLng().billionths);// ... and billionths (u16/u32)
  Serial.println(gps.date.value()); // Raw date in DDMMYY format (u32)
  Serial.println(gps.date.year()); // Year (2000+) (u16)
  Serial.println(gps.date.month()); // Month (1-12) (u8)
  Serial.println(gps.date.day()); // Day (1-31) (u8)
  Serial.println(gps.time.value()); // Raw time in HHMMSSCC format (u32)
  Serial.println(gps.time.hour()); // Hour (0-23) (u8)
  Serial.println(gps.time.minute()); // Minute (0-59) (u8)
  Serial.println(gps.time.second()); // Second (0-59) (u8)
  Serial.println(gps.time.centisecond()); // 100ths of a second (0-99) (u8)
  Serial.println(gps.speed.value()); // Raw speed in 100ths of a knot (i32)
  Serial.println(gps.speed.knots()); // Speed in knots (double)
  Serial.println(gps.speed.mph()); // Speed in miles per hour (double)
  Serial.println(gps.speed.mps()); // Speed in meters per second (double)
  Serial.println(gps.speed.kmph()); // Speed in kilometers per hour (double)
  Serial.println(gps.course.value()); // Raw course in 100ths of a degree (i32)
  Serial.println(gps.course.deg()); // Course in degrees (double)
  Serial.println(gps.altitude.value()); // Raw altitude in centimeters (i32)
  Serial.println(gps.altitude.meters()); // Altitude in meters (double)
  Serial.println(gps.altitude.miles()); // Altitude in miles (double)
  Serial.println(gps.altitude.kilometers()); // Altitude in kilometers (double)
  Serial.println(gps.altitude.feet()); // Altitude in feet (double)
  Serial.println(gps.satellites.value()); // Number of satellites in use (u32)
  Serial.println(gps.hdop.value()); // Horizontal Dim. of Precision (100ths-i32)
}

// Function to Trim White Space
void trim(char *str) {
  int i;
  int begin = 0;
  int end = strlen(str) - 1;

  while (isspace((unsigned char)str[begin]))
    begin++;

  while ((end >= begin) && isspace((unsigned char)str[end]))
    end--;

  // Shift all characters back to the start of the string array.
  for (i = begin; i <= end; i++)
    str[i - begin] = str[i];

  str[i - begin] = '\0';  // Null terminate string.
}

// Callback Function for Debugging Scans
void scanDebugging(T_LE_CB_DATA* p_data) {
    Serial.print("Scan Data ");
    Serial.println(++dataCount);
    // Pretty prints Advert Scan Info
    BLE.configScan()->printScanInfo(p_data);
    //  - Note: Columns are (1) ADVType, (2) AddrType, (3) BT_Addr, (4) rssi

    // Scan and Parse the Found Device Information
    foundDevice.parseScanInfo(p_data);
    //// Enumerate through each device's information [ DEBUGGING ]
    // Check Device Name
    if (foundDevice.hasName()) {
      Serial.print("Device Name: ");
      Serial.println(foundDevice.getName());
    } else {
      Serial.print("Device [ ");
      Serial.print(foundDevice.getAddr().str());
      Serial.println("] has no Name");
    }
    // Check Device UUID
    if (foundDevice.hasUUID()) {
      Serial.println("Device UUID exists.... Not sure how to access");
    } else {
      Serial.println("Device does not have a UUID");
    }
    // Check Device Manufacturer
    if (foundDevice.hasManufacturer()) {
      Serial.print("Device Manufacturer: ");
      Serial.println(foundDevice.getManufacturer());
      Serial.print("Device Manufacturer Data Length: ");
      Serial.println(foundDevice.getManufacturerDataLength());
      Serial.print("Device Manufacturer Data:");
      //Serial.println(sizeof(foundDevice.getManufacturerData()));
      int manuSize = sizeof(foundDevice.getManufacturerData());
      uint8_t *manuData = foundDevice.getManufacturerData();
      //Serial.println(foundDevice.getManufacturerData());
      for (int i = 0; i < manuSize; i++) {
        Serial.print(" ");
        Serial.print(manuData[i]);
      }
      Serial.println();
    } else {
      Serial.println("Device does not have manufacturer information");
    }
    // Check BLE Advert Data Flags
    if (foundDevice.hasFlags()) {
      //Serial.println("Device Advert Data Flags exists.... Not sure how to access");
      Serial.print("Device Flags: ");
      Serial.println(foundDevice.getFlags());
    } else {
      Serial.println("Device Advert Data does not have flags");
    }
    // Check Advert Type
    Serial.print("Device Advert Type: ");
    Serial.println(foundDevice.getAdvType());
    // Check Address Type
    Serial.print("Device Address Type: ");
    Serial.println(foundDevice.getAddrType());
    // Check Device Address
    Serial.print("Device Address: ");
    Serial.println(foundDevice.getAddr().str());
    // Check Device RSSI
    Serial.print("Device RSSI: ");
    Serial.println(foundDevice.getRSSI());
    // Check Device Service Count
    uint8_t serviceCount = foundDevice.getServiceCount();
    if (serviceCount > 0) {
      Serial.print("Device Service Count: ");
      Serial.println(serviceCount);
      // Check Device Service List
      Serial.print("Device Service List: ");
      BLEUUID* serviceList = foundDevice.getServiceList();
      for (uint8_t i = 0; i < serviceCount; i++) {
        Serial.print("\n\tService #");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(serviceList[i].str());
      }
      //Serial.println(foundDevice.getServiceList());
    } else {
      Serial.println("Device has no services");
    }
    // Check Device TX Power
    Serial.print("Device TX Power: ");
    Serial.println(foundDevice.getTxPower());
    // Check Device Appearance
    Serial.print("Device Appearance: ");
    Serial.println(foundDevice.getAppearance());
}

// Callback Function for Parsing Scans
void scanFunction(T_LE_CB_DATA* p_data) {
    ++dataCount;
    // Pretty prints Advert Scan Info
    //BLE.configScan()->printScanInfo(p_data);
    //  - Note: Columns are (1) ADVType, (2) AddrType, (3) BT_Addr, (4) rssi
    Serial.print("-=[ Device #");
    Serial.print(dataCount);
    Serial.println(" - Examination ]=-");

    // Scan and Parse the Found Device Information
    foundDevice.parseScanInfo(p_data);
    // Extract BLE information into expected structures
    extractBLE(foundDevice);
    //// Enumerate through each device's information [ DEBUGGING ]
    // Check Device Name
    if (foundDevice.hasName()) {
      Serial.print("Device Name: ");
      Serial.println(device_name);
    } else {
      Serial.print("Device [ ");
      Serial.print(device_address.str());
      Serial.println("] has no Name");
    }
    // Check Device UUID
    if (foundDevice.hasUUID()) {
      Serial.println("Device UUID exists.... Not sure how to access");
    } else {
      Serial.println("Device does not have a UUID");
    }
    // Check Device Manufacturer
    if (foundDevice.hasManufacturer()) {
      Serial.print("Device Manufacturer: ");
      Serial.println(device_manufacturer);
      Serial.print("Device Manufacturer Data Length: ");
      Serial.println(device_manu_data_length);
      Serial.print("Device Manufacturer Data:");
      //Serial.println(sizeof(foundDevice.getManufacturerData()));
      //Serial.println(foundDevice.getManufacturerData());
      for (int i = 0; i < device_manu_data_length; i++) {
        Serial.print(" ");
        Serial.print(device_manu_data[i]);
      }
      Serial.println();
    } else {
      Serial.println("Device does not have manufacturer information");
    }
    // Check BLE Advert Data Flags
    if (foundDevice.hasFlags()) {
      //Serial.println("Device Advert Data Flags exists.... Not sure how to access");
      Serial.print("Device Flags: ");
      Serial.println(device_flags);
    } else {
      Serial.println("Device Advert Data does not have flags");
    }
    // Check Advert Type
    Serial.print("Device Advert Type: ");
    Serial.println(device_advert_type);
    // Check Address Type
    Serial.print("Device Address Type: ");
    Serial.println(device_addr_type);
    // Check Device Address
    Serial.print("Device Address: ");
    Serial.println(device_address.str());
    // Check Device RSSI
    Serial.print("Device RSSI: ");
    Serial.println(device_rssi);
    // Check Device Service Count
    if (device_service_count > 0) {
      Serial.print("Device Service Count: ");
      Serial.println(device_service_count);
      // Check Device Service List
      Serial.print("Device Service List: ");
      for (uint8_t i = 0; i < device_service_count; i++) {
        Serial.print("\n\tService #");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(device_service_list[i].str());
      }
      //Serial.println(foundDevice.getServiceList());
    } else {
      Serial.println("Device has no services");
    }
    // Check Device TX Power
    Serial.print("Device TX Power: ");
    Serial.println(device_tx_power);
    // Check Device Appearance
    Serial.print("Device Appearance: ");
    Serial.println(device_appearance);
}

//// BLE GATT Server Configuration and Setup
// Complete Server Name
const char* device_complete_name = "BLE GPS Reporter";
// Short Server Name
const char* device_short_name = "BGR Fox";

// Maximum Count of Services
uint8_t device_max_service_count = 1;   // Used to Configure the Number of Services the GATT Server will have
// Service UUID Definitions
#define SERVICE_UUID__SYSTEM_CHECK "575C"     // 575tem Check
#define SERVICE_UUID__DATA_EXTRACT "DA74"     // DAta
#define SERVICE_UUID__NOTIFY_SIGNAL "C516"    // Change 516nal
// Characteristic UUID Definitions
#define CHAR_UUID__DATABASE_HASH "DB00"       // DataBase hash 00
#define CHAR_UUID__DATA_GPS "695D"            // GPS Data
#define CHAR_UUID__DATA_BLE "B1ED"            // BLE Data
#define CHAR_UUID__DATA_BLE_GPS "B695"        // BLE + GPS Data

// Configure BLE Services
BLEService system_check_service(SERVICE_UUID__SYSTEM_CHECK);
BLEService data_extract_service(SERVICE_UUID__DATA_EXTRACT);
BLEService notify_signal_service(SERVICE_UUID__NOTIFY_SIGNAL);
// Configure BLE Characteristics
BLECharacteristic database_hash_char(CHAR_UUID__DATABASE_HASH);
BLECharacteristic gps_data_char(CHAR_UUID__DATA_GPS);
BLECharacteristic ble_data_char(CHAR_UUID__DATA_BLE);
BLECharacteristic mixed_data_char(CHAR_UUID__DATA_BLE_GPS);

// Structures for Advertising the BLE GATT Server
BLEAdvertData advert_data;  // Stucture skeleton for holding BLE GATT Server info; configured in init
BLEAdvertData scan_advert_data;

// Variable for Tracking Notification
bool device_notify_flag = false;

/// Callback Functions for BLE GATT Server
// Function for Alerting that a Characteristic has been Read
void read_alert__callback(BLECharacteristic* device_char, uint8_t connection_id) {
  printf("[!] Read Alert::Characteristic %s read by connection %d\n", device_char->getUUID().str(), connection_id);
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

//// Setup and Main Code Functions

// Setup Function for ESP32 WROOM DA
void setup() {
  Serial.begin(115200);
  //SerialPort.begin(GPSBaud, SERIAL_8N1, 16, 17);
  SerialPort.begin(GPSBaud);

  // Short Delay
  delay(100);

  //// Configuration to Search for Other BLE Devices -[ Central Device ]-
  // Setup BLE Device for Scanning
  BLE.init();
  BLE.configScan()->setScanMode(GAP_SCAN_MODE_ACTIVE);  // Active mode requests for scan response packets
  BLE.configScan()->setScanInterval(500);   // Start a scan every 500ms
  BLE.configScan()->setScanWindow(250);     // Each scan lasts for 250ms
  BLE.configScan()->updateScanParams();
  // Provide a callback function to process scan data.
  // If no function is provided, default BLEScan::printScanInfo is used
  BLE.setScanCallback(scanFunction);
  //BLE.setScanCallback(scanDebugging);
  BLE.beginCentral(0);

  BLE.configScan()->startScan(5000);    // Repeat scans for 5 seconds, then stop
  // Note: This runs BEFORE the rest of the main loop

  //// Configuration to Run a GATT Server -[ Peripheral Device ]-
  // Setup Advert Data
  advert_data.addFlags();
  advert_data.addCompleteName(device_complete_name);
  advert_data.addShortName(device_short_name);
  
  // Configure Characteristic Data
  database_hash_char.setReadProperty(true);
  database_hash_char.setReadPermissions(GATT_PERM_READ);
  database_hash_char.setReadCallback(read_alert__callback);
  database_hash_char.setNotifyProperty(true);
  database_hash_char.setCCCDCallback(notify_alert__callback);

  // Configure Service Data
  system_check_service.addCharacteristic(database_hash_char);

  // Initialize and Configure the GATT Server
  BLE.configAdvert()->setAdvData(advert_data);
  BLE.configAdvert()->setScanRspData(scan_advert_data);
  BLE.configServer(device_max_service_count);
  BLE.addService(system_check_service);

  // Start BLE Peripheral
  BLE.beginPeripheral();
}

// Main Function
void loop() {
  // GPS Interaction
  while (SerialPort.available() > 0) {
    if (gps.encode(SerialPort.read())) {
      //Serial.println("[*] Dumping GPS Info");
      dumpInfo();
      //Serial.println("[*] Printing GPS Info");
      //gpsInfoDump();
    }
  }
  // No GPS Device Detected
  if (millis() > 5000 && gps.charsProcessed() < 10) {
    Serial.println(F("No GPS detected: check wiring."));
  }
  // No GPS Data
  if (gps.charsProcessed() < 10) {
    Serial.println(F("No GPS data received: check wiring"));
  }

  // Read New BLE Information
  BLE.configScan()->startScan(scanTime_ms);   // Scan for Two Seconds

  // Cleaning
  //pBLEScan->clearResults();  // delete results fromBLEScan buffer to release memory
  delay(pauseTime_ms);
}
