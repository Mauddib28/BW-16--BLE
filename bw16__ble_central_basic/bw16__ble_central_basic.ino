#include "BLEDevice.h"
#ifdef min
// Whatever
#else
#include <map>
#endif

// Debugging Variables
char emptyChar = '\0';
bool debug_flag = false;
bool verbose_flag = false;

// BLE GATT Server Configuration Variables
const char* device_complete_name = "BLE Central Hub";
//String device_complete_name = String("BLE Central Hub");

#define UART_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define STRING_BUF_SIZE 100

#define MAX_CONNECTIONS 10  // Define the maximum number of connections

//// Structure Definitions for Bluetooth Low Energy related things
// Structure Definition for the BLEConnection Object; tracks a connection for a given device
struct BLEConnection {
    BLEClient* client;       // Pointer to the BLEClient
    BLEAdvertData advertData; // Advert data for the connection
    bool isConnected;         // Connection status
};

// Structure to hold multiple BLE connections
struct BLEConnectionManager {
    BLEConnection connections[MAX_CONNECTIONS]; // Array of connections
    int count; // Current number of active connections

    BLEConnectionManager() : count(0) {} // Constructor to initialize count
};

// Define Service UUIDs for Audio Capabilities
BLEUUID audioServiceUUIDs[] = {
    BLEUUID("0000110B-0000-1000-8000-00805F9B34FB"), // Audio Sink
    BLEUUID("0000110A-0000-1000-8000-00805F9B34FB"), // Audio Source
    BLEUUID("0000110C-0000-1000-8000-00805F9B34FB"), // AV Remote Control Target
    BLEUUID("0000110E-0000-1000-8000-00805F9B34FB"), // AV Remote Control
    BLEUUID("00001108-0000-1000-8000-00805F9B34FB"), // Headset
    BLEUUID("0000111E-0000-1000-8000-00805F9B34FB"), // Hands-Free
    BLEUUID("0000110D-0000-1000-8000-00805F9B34FB")  // Audio Stream
};

// Define Characteristic UUIDs for Audio Capabilities
BLEUUID audioCharacteristicUUIDs[] = {
    BLEUUID("00002A9D-0000-1000-8000-00805F9B34FB"), // Audio Control Point
    BLEUUID("00002A9E-0000-1000-8000-00805F9B34FB"), // Audio Data
    BLEUUID("00002A7D-0000-1000-8000-00805F9B34FB")  // Volume Control
};


//// BLE GATT Server Specific Definitions
// Structure for Holding Local BLE Address
uint8_t local_addr[6];
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

// BLE Connection Manager Declaration
BLEConnectionManager connectionManager; // Create an instance of the connection manager

//// Function Definitions for Device Connection Tracking
// Function for Adding a Connection to the Connection Manager
bool addConnection(BLEClient* client, BLEAdvertData advertData) {
    if (connectionManager.count < MAX_CONNECTIONS) {
        connectionManager.connections[connectionManager.count].client = client;
        connectionManager.connections[connectionManager.count].advertData = advertData;
        connectionManager.connections[connectionManager.count].isConnected = true;
        connectionManager.count++;
        return true; // Successfully added
    }
    return false; // Connection limit reached
}

// Function for Searching for a Specific Connection Based on a String Name (i.e. Device Name)
BLEClient* findConnectionByName(const String& name) {
    for (int i = 0; i < connectionManager.count; i++) {
        if (connectionManager.connections[i].advertData.getName() == name) {
            return connectionManager.connections[i].client; // Return the client if found
        }
    }
    return nullptr; // Not found
}

// Function for Printing the Current Active Connections
void printConnections() {
    Serial.println("Active Connections:");
    for (int i = 0; i < connectionManager.count; i++) {
        Serial.print("Connection ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(connectionManager.connections[i].advertData.getName());
    }
}


//// Function Definitions for BLE GATT Server
// Function for Configuring the BLE GATT Server's Advert Data
void configure_advert(BLEAdvertData advert_data) {
    // Clear and reconfigure the advertising data
    advert_data.clear();
    delay(100);

    // Setup the advert data with proper flags
    advert_data.addFlags(GAP_ADTYPE_FLAGS_GENERAL | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);

    // Add the Compelte Name to the Advert Data
    advert_data.addCompleteName(device_complete_name);
}

// Function for Configuring the BLE GATT Server's Advanced 
void configure_adv_scan_resp(BLEAdvertData scan_data) {
    // Clear and reconfigure the adv. scan response data
    scan_data.clear();
    delay(100);
  
    // Setup Scan Data with proper service advertising
    scan_data.addCompleteServices(BLEUUID(UART_SERVICE_UUID));    // Adding the UART Service

    // Add the Complete Name to the Adv. Scan Response Data
    scan_data.addCompleteName(device_complete_name);
}

// Function for Printing all sub-details (i.e. Services, Characteristics, Descriptors) for a BLEClient Device
void printDeviceDetails(BLEClient* client) {
    // Call the printServices() function of the BLEClient objcet
    client->printServices();
}

// Function for Enumerating Services of a Provided BLEClient
void enumerateServices(BLEClient* client, BLEAdvertData& advertData) {
    if (client == nullptr) {
        Serial.println("Client is null, cannot enumerate services.");
        return;
    }

    // Discover services on the client
    client->discoverServices();
    Serial.println("Enumerating services...");

    // Wait until service discovery is done
    do {
        Serial.print(".");
        delay(1000);
    } while (!(client->discoveryDone()));   // Continue to wait until the Service Discovery Status becomes True
    Serial.println();

    // Obtain the List of Services Related to the Client Device
    BLEUUID* services_list = advertData.getServiceList();
    int serviceCount = advertData.getServiceCount();

    // Loop through the Service List
    for (int i = 0; i < serviceCount; i++) {
        BLEUUID uuid = services_list[i];    // Obtain the current Service UUID
        BLERemoteService* service = client->getService(uuid);   // Get the Service UUID's information from the BLEClient

        // Enumerate the Service; Note: Will have to be done via a whitelist of known UUIDS
        //  - There is NO DEFAULT PUBLIC FUNCTIONALITY for enumerating a Characteristic or later Descriptor lists
        //  - Due to public/private sectioning of the BLERemoteService's functionality
        for (auto& audio_serv_uuid : audioServiceUUIDs) {
            Serial.println(audio_serv_uuid.str());
            // Check if the Current Service UUID matches a known Audio Service UUID
            if (BLEUUID(audio_serv_uuid.str()) == uuid) {
                Serial.print("[+] Found a Known Audio UUID of [ ");
                Serial.print(uuid.str());
                Serial.println(" ]");
                // Continue to enumerate the Service's Characteristics to Look for Specific Audio Functionality
                Serial.println("Audio Characteristic UUIDs:");
                for (auto& audio_char_uuid : audioCharacteristicUUIDs) {
                    Serial.println(audio_char_uuid.str());
                    // Check to see if the Current Characteristic has the Specific Audio Functionality
                    BLERemoteCharacteristic* characteristic = service->getCharacteristic(audio_char_uuid);
                    // Check if a Characteristic was Returned
                    if (characteristic != nullptr) {
                        Serial.print("[+] Found a Known Audio Characteristic UUID of [ ");
                        Serial.print(audio_char_uuid.str());
                        Serial.println(" ]");
                    } else {
                        if (debug_flag) {
                            Serial.print("[-] Characteristic UUID [ ");
                            Serial.print(audio_char_uuid.str());
                            Serial.println(" ] is NOT a Knonw Characteristic Audio UUID");
                        }
                    }
                }
            } else {
                if (debug_flag) {
                    Serial.print("[-] UUID [ ");
                    Serial.print(uuid.str());
                    Serial.println(" ] is NOT a Known Audio UUID");
                }
            }
        }
    }
}

// Function to Enumerate a Provided BLEClient Device
void enumerateDevice(BLEClient* client, BLEAdvertData& advertData) {
    // Enumerate the Services (and recursive Characteristics + Descriptors)
    enumerateServices(client, advertData);
}

// Function for Processing a New Client Device for the BLE GATT Server's Tracking
void processNewClient(int8_t connection_id) {
    // Perform initialization for the New Client
    BLE.configClient();
    // Create the New Client Object based on the Device Connection ID
    BLEClient* new_client = BLE.addClient(connection_id);
    // Discover/Enumerate information about the New Device

    // Perform Connection-Type (e.g. Trusted, Bonded) Examination of the New Client

    // Incorporate the New Client device into the appropriate structures for tracking the new client device
    //  - Note: The purpose of this is to provide ease of search, enumeration, association, and recall for later BLE GATT Server functionality
}

//// Function Definitions for Callback Functions
// Callback Function for Scanning when Devices are Found (via advertised data)
void scanCB(T_LE_CB_DATA* p_data) {
    uint8_t target_addr[6] = {0x41, 0x2D, 0xAE, 0x60, 0xC9, 0x94 };   // Note: The target address must be written in little endian
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
            // Debug found device information
            if (debug_flag) {
                Serial.print("[+] scanCB::Device Found with Address: ");
                Serial.println(foundDevice.getAddr().str());
                // Testing the direct access to variables
                //Serial.println(scan_info->bd_addr);
                if (verbose_flag) {
                    for (int i = 0; i < 6; i++) {
                        Serial.print(scan_info->bd_addr[i]);
                        Serial.print(" - ");
                    }
                    Serial.println();
                    //array_print(scan_info->bd_addr);
                    Serial.print("Searching for: ");
                    for (int i = 0; i < 6; i++) {
                        Serial.print(target_addr[i]);
                        Serial.print(" = ");
                    }
                }
            }
            // Compare Addresses Byte by Byte to Confirm Target
            //  - Note: Target BT ADDR is "94:C9:60:AE:2D:41"
            bool found_target_flag = true;
            for (int i = 0; i < 6; i++) {
                // Test if the bytes match
                bool byte_test = scan_info->bd_addr[i] == target_addr[i];
                // Perform bitwise AND between two bools
                found_target_flag = found_target_flag & byte_test;
                //if (scan_info->bd_addr[i] == target_addr[i]) {
                //    Serial.println("MATCHING BYTE");
                //}
                if (debug_flag & verbose_flag) {
                    Serial.print("Found Flag Value: ");
                    Serial.println(found_target_flag);
                }
            }
            // Check if the Address is a hardcoded address we want
            if (found_target_flag) {
                Serial.print("[+] scanCB::Target Address Found [ ");
                Serial.print(foundDevice.getAddr().str());
                Serial.println(" ] ");            
                targetDevice = foundDevice;
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

//// -=[MAIN CODE FUNCTIONS]=-
// Function for Setting up the RTL8720DN Device
void setup() {
    Serial.begin(115200);

    Serial.println("[*] Configuring Adertisement and Response Data");
    advert_data.clear();
    delay(100);
    // Prepare the Advertising Data for the BLE GATT Server
    configure_advert(advert_data);
    // Prepare the Adv. Scanning Response Advert Data for the BLE GATT Server
    configure_adv_scan_resp(scan_data);
    
    Serial.println("[*] Initializing BLE and Configuring Scan Call Back");
    BLE.init();
    delay(200);
    BLE.setDeviceName(device_complete_name);
    delay(100);
    // Configure the Advert Datas into the BLE Object
    BLE.configAdvert()->setAdvData(advert_data);
    BLE.configAdvert()->setScanRspData(scan_data);

    // Configure the BLE Server before adding services
    BLE.configServer(1);    // Note: The reason for the 1 is that current there is only ONE service
    delay(100);
    
    BLE.setScanCallback(scanCB);
    Serial.println("[*] Beginning Central");
    // Start Central Device
    BLE.beginCentral(2);    // Set the maximum number of connections to TWO

    Serial.print("[*] BLE Server Address: ");
    BLE.getLocalAddr(local_addr);
    for (int i = 5; i >= 0; i--) {
        Serial.print(local_addr[i], HEX);
        if (i > 0) Serial.print(":"); // Print colon between bytes
    }
    Serial.println();
    
    Serial.println("[*] Starting Scan of Devices");
    // Scan for Devices in Proximity
    BLE.configScan()->startScan(2000);      // Scan for 2000 milliseconds
    Serial.print("[*] Attempting to Connect to Target Device: ");
    if (targetDevice.hasName()) {
        Serial.println(targetDevice.getName());
    } else {
        Serial.println(targetDevice.getAddr().str());
    }
    // Connect to the Target Device; assuming it was seen
    BLE.configConnection()->connect(targetDevice, 2000);        // Attempt to Connect with 2000 millisecond timeout
    // Note: The .connect() attribute of BLEConnect Objcets allows for direct passing of BLEAdvertData
    Serial.println("[*] Time wait of 2000 milliseconds");
    delay(2000);
    // Create a connection ID from the established aforementioned connection
    int8_t connID = BLE.configConnection()->getConnId(targetDevice);
    // Check if a connection ID was returned by the above function calls
    if (!BLE.connected(connID)) {
        Serial.println("BLE not connected");
    } else {
        // Initialize a BLEClient object and register default client callback; based on internal default?
        BLE.configClient();
        // Create and Return a new Client Connection by configuring a client ID and connection ID
        client = BLE.addClient(connID);
        // Obtain Discovering of Services on the Connected Device
        client->discoverServices();
        Serial.print("Discovering services of connected device");
        do {
            Serial.print(".");
            delay(1000);
        } while (!(client->discoveryDone()));   // Continue to wait until the Service Discovery Status becomes True
        Serial.println();

        // Search and Return the specfic UART_SERVICE_UUID
        UartService = client->getService(UART_SERVICE_UUID);
        // Check if the UART Service was found
        if (UartService != nullptr) {
            // Search and Return the specific CHARACTERISTIC_UUID_TX
            Tx = UartService->getCharacteristic(CHARACTERISTIC_UUID_TX);
            // Check if the TX Characteristic was found
            if (Tx != nullptr) {
                Serial.println("TX characteristic found");
                Tx->setBufferLen(STRING_BUF_SIZE);
                Tx->setNotifyCallback(notificationCB);
                Tx->enableNotifyIndicate();
            }
            // Search and Return the specific CHARACTERISTIC_UUID_RX
            Rx = UartService->getCharacteristic(CHARACTERISTIC_UUID_RX);
            // Check if the RX Characteristic was found
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
