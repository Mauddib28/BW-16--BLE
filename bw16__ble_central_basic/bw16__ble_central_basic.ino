
#include "BLEDevice.h"

// Debugging Variables
char emptyChar = '\0';
bool debug_flag = false;
bool verbose_flag = false;

// BLE GATT Server Configuration Variables
const char* device_complete_name = "BLE Central Hub";

#define UART_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define STRING_BUF_SIZE 100

// BLE Advert Data for Presenting Advertising and Services
BLEAdvertData advert_data;      // Advert Data Structure for Containing Advertising/Beacon Info
BLEAdvertData scan_data;        // Advert Data Structure for Containing Adv. Scan Response Info
// BLE Advert Data for Scanning Existing BLE Devices in Proximity
BLEAdvertData foundDevice;      // Advert Data Structure for Holding Found Device Advert Info
BLEAdvertData targetDevice;     // Advert Data Structure for Holding Target Device Advert Info
// BLE Client Structure for Interacting with BLE Client Devices (i.e. Devices Connecting to this Central Device)
BLEClient* client;
// Structures Used to Present Varioud BLE Services and Characteristics
BLERemoteService* UartService;
BLERemoteCharacteristic* Rx;
BLERemoteCharacteristic* Tx;

// Function for Configuring the BLE GATT Server's Advert Data
void configure_advert(BLEAdvertData advert_data) {
    // Clear and reconfigure the advertising data
    advert_data.clear();

    // Setup the advert data with proper flags
    advert_data.addFlags(GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);
}

// Callback Function for Scanning when Devices are Found (via advertised data)
void scanCB(T_LE_CB_DATA* p_data) {
    foundDevice.parseScanInfo(p_data);
    if (foundDevice.hasName()) {
        if (debug_flag) {
            Serial.print("[+] scanCB::Device Found: ");
            Serial.print(foundDevice.getName());
            Serial.print(" [ ");
            Serial.print(foundDevice.getAddr().str());
            Serial.println(" ] ");
        }
        if (foundDevice.getName() == String("BLE GPS Reporter")) {
            Serial.print("Found the GPS Reporting BLE Device at address ");
            Serial.println(foundDevice.getAddr().str());
            targetDevice = foundDevice;
        }
    } else {
        // Nota Bene: This can lead to seeing devices twice, because one can see a device and not have the name
        T_LE_SCAN_INFO *scan_info = p_data->p_le_scan_info;
        if (BLEAddr(scan_info->bd_addr).str()[0] != emptyChar) {
            if (debug_flag) {
                Serial.print("[+] scanCB::Device Found with Address: ");
                Serial.println(foundDevice.getAddr().str());
            }
        } else {
            if (debug_flag & verbose_flag) {
                Serial.println("[-] scanCB::False Positive for Device Detection"); 
            }
        }
    }
}

// Callback Function for Notification (instances?)
void notificationCB (BLERemoteCharacteristic* chr, uint8_t* data, uint16_t len) {
    char msg[len+1] = {0};
    memcpy(msg, data, len);
    Serial.print("Notification received for chr UUID: ");
    Serial.println(chr->getUUID().str());
    Serial.print("Received string: ");
    Serial.println(String(msg));
}

// Function for Setting up the RTL8720DN Device
void setup() {
    Serial.begin(115200);

    Serial.println("[*] Initializing BLE and Configuring Scan Call Back");

    BLE.init();
    delay(200);
    BLE.setDeviceName(device_complete_name);

    // Clear any previous data; requires inclusion of advert_data
    
    
    BLE.setScanCallback(scanCB);
    Serial.println("[*] Beginning Central");
    BLE.beginCentral(2);    // Set the maximum number of connections to TWO
    
    Serial.println("[*] Starting Scan of Devices");
    BLE.configScan()->startScan(2000);
    Serial.print("[*] Attempting to Connect to Target Device: ");
    Serial.println(targetDevice.getName());
    BLE.configConnection()->connect(targetDevice, 2000);
    Serial.println("[*] Time wait of 2000 milliseconds");
    delay(2000);
    int8_t connID = BLE.configConnection()->getConnId(targetDevice);
    if (!BLE.connected(connID)) {
        Serial.println("BLE not connected");
    } else {
        BLE.configClient();
        client = BLE.addClient(connID);
        client->discoverServices();
        Serial.print("Discovering services of connected device");
        do {
            Serial.print(".");
            delay(1000);
        } while (!(client->discoveryDone()));
        Serial.println();

        UartService = client->getService(UART_SERVICE_UUID);
        if (UartService != nullptr) {
            Tx = UartService->getCharacteristic(CHARACTERISTIC_UUID_TX);
            if (Tx != nullptr) {
                Serial.println("TX characteristic found");
                Tx->setBufferLen(STRING_BUF_SIZE);
                Tx->setNotifyCallback(notificationCB);
                Tx->enableNotifyIndicate();
            }
            Rx = UartService->getCharacteristic(CHARACTERISTIC_UUID_RX);
            if (Rx != nullptr) {
                Serial.println("RX characteristic found");
                Rx->setBufferLen(STRING_BUF_SIZE);
            }
        }
    }
}

void loop() {
    if (Serial.available()) {
        Rx->writeString(Serial.readString());
    }
    delay(100);
    // Testing Repeated Scans
    delay(2000);
    // Try another scan
    BLE.configScan()->startScan(2000);

    // Test to see if the target device was found
    //  - Note: The DEFAULT for the Advert Data Name is "", HENCE the .hasName() check for validity
    if (targetDevice.hasName()) {
        Serial.println("[+] DEVICE HAS BEEN FOUND!");
    } else {
        Serial.println("[-] DEVICE NOT FOUND!");
    }
}
