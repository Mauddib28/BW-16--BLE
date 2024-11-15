// TinyGPS Includes
//#include <TinyGPSPlus.h>
// Arduino GPS Includes
#include <Adafruit_GPS.h>

// BLE Includes
#include <BLEDevice.h>

// GAP API Includes
#include "gap.h"
#include "gap_le.h"

// Software Serial Include
#include <SoftwareSerial.h>
//#include "Arduino.h"
//#include "HardwareSerial.h"

////
// DEBUGGING VARIABLE
////
#define DEBUG__BLE_SERVER  false
#define DEBUG__GPS_DATA    false

// Configured from docs; url: https://www.amebaiot.com/en/amebad-bw16-arduino-getting-started/
static const int RXPin = 5, TXPin = 4;
static const uint32_t GPSBaud = 9600;
//SoftwareSerial SerialPort(RXPin, TXPin);
//HardwareSerial gpsSerial(TXPin, RXPin);    // Create hardware serial instance
//HardwareSerial SerialPort(2); //(1);
// Due to configuration of the BW-16 RealTek board, the following defition can occur
SoftwareSerial gpsSerial(PB2, PB1); // RX, TX
//RealTek_Serial SerialPort;
// GPS data...
double lat;
double lng;
int month;
int day;
int year;
int hour;
int minute;
int second;
int millisecond;
int alt;
//double acc;
int fix;
double fixquality;
int satellites;
double speed;
double angle;
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
//TinyGPSPlus gps;
// The Adafruit_GPS object
Adafruit_GPS gps(&gpsSerial);

//// Configuration for GPS and GPS Data Structures

// Set GPSECHO to 'false' to turn off echoing the GPS data to the Serial console
// Set to 'true' if you want to debug and listen to the raw GPS sentences. 
#if DEBUG__GPS_DATA
#define GPSECHO  true
#else
#define GPSECHO  false
#endif

// Structure for GPS Data
void dumpInfo(Adafruit_GPS gps) {

  lat = gps.latitudeDegrees;
  lng = gps.longitudeDegrees;
  alt = gps.altitude;
  month = gps.month;
  day = gps.day;
  year = gps.year;
  hour = gps.hour;
  minute = gps.minute;
  second = gps.seconds;
  millisecond = gps.milliseconds;
  fix = gps.fix;
  fixquality = gps.fixquality;
  satellites = gps.satellites;
  speed = gps.speed;
  angle = gps.angle;

  return;
}

// Function for Printing GPS Time Data
void print_gps_time() {
    Serial.print("\nTime: ");
    Serial.print(hour, DEC); Serial.print(':');
    Serial.print(minute, DEC); Serial.print(':');
    Serial.print(second, DEC); Serial.print('.');
    Serial.println(millisecond);
    Serial.print("Date: ");
    Serial.print(day, DEC); Serial.print('/');
    Serial.print(month, DEC); Serial.print("/20");
    Serial.println(year, DEC);
}

// Function for Printing GPS Location Data
void print_gps_location() {
    Serial.print("Location: ");
    Serial.print(gps.latitude, 4); Serial.print(gps.lat);
    Serial.print(", "); 
    Serial.print(gps.longitude, 4); Serial.println(gps.lon);
    Serial.print("Location (in degrees, works with Google Maps): ");
    Serial.print(lat, 4);
    Serial.print(", "); 
    Serial.println(lng, 4);
} 

// Function for Printing GPS Fix and Satellites Information
void print_gps_fix() {
    Serial.print("Fix: "); Serial.print((int)fix);
    Serial.print(" quality: "); Serial.println((int)fixquality);   
    Serial.print("Satellites: "); Serial.println((int)satellites);    
}
  
// Function for Printing GPS Speed and Angle
void print_gps_speed_angle() {
    Serial.print("Angle: "); Serial.println(angle);
    Serial.print("Altitude: "); Serial.println(alt);
}

//// BLE GATT Server Variables and Definitions
// Error tracking
uint8_t systemErrors = 0x00;
#define ERROR_GPS_DISCONNECTED 0x01
#define ERROR_GPS_NO_FIX      0x02
#define ERROR_LOW_MEMORY      0x04

// Bluetooth Define Definitions
#define BLE_APPEARANCE_GENERIC_LOCATION_NAVIGATION_DISPLAY 0x1419

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
  if (DEBUG__BLE_SERVER) {
    // Proof of read having happened
    device_char->writeString("Read Me");
  }
}

// Functions for Alterting that a Characteristic has been Written to; NOTE: There is no write callback function for BLE...
void write_alert__callback(BLECharacteristic* device_char, uint8_t connection_id) {
  printf("[!] Write Alert::Characteristic %s written to by connection %d\n", device_char->getUUID().str(), connection_id);
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
  // Define the Error Strings Array
  const char* error_strings[] = {
    "Nothing is Wrong",
    "GPS Disconnected",
    "GPS No Fix",
    "Low Memory",
    "Unknown GPS Error"
  };
  char error_buffer[20];
  char serial_error_buffer[20];
  sprintf(error_buffer, "%s", error_strings[systemErrors]);
  // Write the Associated Error String to Error Bitmask into the Characteristic Buffer
  device_char->writeString(error_buffer);
  // Copy write of error string buffer to the serial line
  Serial.print("System Error: ");
  Serial.println(error_buffer);
  // Write out to the serial line the exact system error bitmask
  sprintf(serial_error_buffer, "Sys Err: [ %02X ]", systemErrors);
  Serial.println(serial_error_buffer);
  // Clear systemErrors variable
  systemErrors = 0x00;
}

// GATT Server components
BLEService locationService(LN_SERVICE_UUID);
BLEService deviceInfoService(DEVICE_INFO_SERVICE_UUID);
BLEService systemStatusService(SYSTEM_STATUS_SERVICE_UUID);

BLECharacteristic locationFeatureChar(LN_FEATURE_CHAR_UUID);
BLECharacteristic locationPositionChar(LN_POSITION_CHAR_UUID);
BLECharacteristic alertStatusChar(ALERT_STATUS_CHAR_UUID);
BLECharacteristic errorStatusChar(ERROR_STATUS_CHAR_UUID);

// Structures for Advertising the BLE GATT Server
BLEAdvertData advert_data;  // Stucture skeleton for holding BLE GATT Server info; configured in init
BLEAdvertData scan_data;    // Structure skeleton for hodling BLE GATT Scan Response?

// Function for Setting Up the Device
//  - Note: Added several 100ms(?) delys that appear to fix issues with configuring the BLE GATT Server
void setup() {
    Serial.begin(115200);
    //SerialPort.begin(GPSBaud, SERIAL_8N1, RXPIN, TXPIN);
    //delay(1000);  // Give more time for initialization
    // Initialize UART for GPS with 8N1 configuration
    //gps_uart = uart_init(TXPin, RXPin, GPSBaud);
    //uart_config(gps_uart, GPSBaud, UART_8N1); // Configure the UART with 8N1 configuration
    //delay(1000);
    //Serial2.begin(GPSBaud);
    //gpsSerial.begin(GPSBaud);
    //SerialPort.begin(GPSBaud, SERIAL_8N1, RXPin, TXPin);

    //// GPS Setup and Configuration
    // Note: 9600 NMEA is the default baud rate for Adafruit MTK GPS's - some use 4800
    gpsSerial.begin(GPSBaud);
    delay(1000);
    // uncomment this line to turn on RMC (recommended minimum) and GGA (fix data) including altitude
    gps.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCGGA);
    // uncomment this line to turn on only the "minimum recommended" data
    //GPS.sendCommand(PMTK_SET_NMEA_OUTPUT_RMCONLY);
    // For parsing data, we don't suggest using anything but either RMC only or RMC+GGA since
    // the parser doesn't care about other sentences at this time

    // Set the update rate
    gps.sendCommand(PMTK_SET_NMEA_UPDATE_1HZ);   // 1 Hz update rate
    // For the parsing code to work nicely and have time to sort thru the data, and
    // print it out we don't suggest using anything higher than 1 Hz

    // Request updates on antenna status, comment out to keep quiet
    gps.sendCommand(PGCMD_ANTENNA);

    delay(1000);
    // Ask for firmware version
    gpsSerial.println(PMTK_Q_RELEASE);

    //// BLE GATT Server Setup and Configuration
    // Initialize BLE with the device name directly
    BLE.init();
    //BLE.init(device_complete_name);
    delay(100);

    // Set the Device Name earlier in the setup configuration
    BLE.setDeviceName(device_complete_name);

    // Set device name using RTL API directly
    //le_set_gap_param(GAP_PARAM_DEVICE_NAME, strlen(device_complete_name), (void *)device_complete_name);
    //le_set_gap_param(GAP_PARAM_APPEARANCE, sizeof(uint16_t), (void *)BLE_APPEARANCE_GENERIC_LOCATION_NAVIGATION_DISPLAY);
    
    // Set advertising parameters
    //BLE.configAdvert()->setMinInterval(100); // 100ms interval
    //BLE.configAdvert()->setAdvTimeout(0);    // 0 = advertise forever
    //le_adv_set_param(GAP_PARAM_ADV_EVENT_TYPE, sizeof(uint8_t), (void *)GAP_ADTYPE_ADV_IND);
    //le_adv_set_param(GAP_PARAM_ADV_DIRECT_ADDR_TYPE, sizeof(uint8_t), (void *)GAP_REMOTE_ADDR_LE_PUBLIC);
    //le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MIN, sizeof(uint16_t), (void *)100);
    //le_adv_set_param(GAP_PARAM_ADV_INTERVAL_MAX, sizeof(uint16_t), (void *)100);

    // Clear and reconfigure advertising data
    advert_data.clear();
    // Setup Advert Data with proper flags
    //advert_data.addFlags(GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);
    advert_data.addFlags(GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);
    //advert_data.addCompleteName(device_complete_name);
    //advert_data.addShortName(device_short_name);
    // Directly setting the device name using lower level functions
    //BLE.setDeviceName(device_complete_name);
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

uint32_t timer = millis();
void loop() {
    // Delay Wait to Have Polling be once a Second
    delay(500);    // One second due to recommended 1Hz operation
    //// GPS Interaction
    // Char to hold data read from the GPS Serial
    char c = gps.read();
    // Debugging raw output
    if (GPSECHO) {
      if (c) {
        Serial.print(c);
      }
    }
    // if a sentence is received, we can check the checksum, parse it...
    if (gps.newNMEAreceived()) {
        // a tricky thing here is if we print the NMEA sentence, or data
        // we end up not listening and catching other sentences! 
        // so be very wary if using OUTPUT_ALLDATA and trytng to print out data
        //Serial.println(GPS.lastNMEA());   // this also sets the newNMEAreceived() flag to false

        // this also sets the newNMEAreceived() flag to false
        if (!gps.parse(gps.lastNMEA())) {
            // we can fail to parse a sentence in which case we should just wait for another
            return;
        }
    }

    // if millis() or timer wraps around, we'll just reset it
    if (timer > millis()) {
        timer = millis();
    }

    // Approximately every 2 seconds or so, print out the current state
    if ((millis() - timer) > 2000) { 
        timer = millis(); // reset the timer

        // Parse the GPS Data; Note: Probably a better place to perform this action
        if (c) {
            dumpInfo(gps);
        } else {
            systemErrors |= ERROR_GPS_DISCONNECTED;
            errorStatusChar.writeData8(systemErrors);  // Changed from writeValue
            Serial.println("[-] GPS Disconnected - Lacking Complete Data");
        }

        // Print GPS Time Data
        print_gps_time();

        // Print Fix Information
        print_gps_fix();
        // GPS Data is Valid
        if (gps.fix) {
            // Print GPS Location Data
            print_gps_location();

            // Print GPS Speed and Angle Data
            print_gps_speed_angle();

            Serial.println("[+] GPS Fix Established");

            // Pack GPS data into characteristic
            // Note: Using individual writes since BW16 doesn't support direct buffer writes
            locationPositionChar.writeData32((uint32_t)lat);
            locationPositionChar.writeData32((uint32_t)lng);
        } else {  // GPS Data is NOT Valid
            systemErrors |= ERROR_GPS_NO_FIX;
            errorStatusChar.writeData8(systemErrors);  // Changed from writeValue
            Serial.println("[-] No GPS Fix");
        }
    }
}
