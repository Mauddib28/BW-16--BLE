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
// Forcing Bluetooth Debugging value?
#ifdef BTDEBUG
#undef BTDEBUG
#endif
#define BTDEBUG 1;   // Not sure this actually does anything....

// Timing Variables
uint32_t millis_timer;    // Variable for Tracking Time in milliseconds; for NON-BLOCKING timing

// Target Tracking Variables
bool target_identified = false;

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
int8_t target_conn_id;
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
        Serial.println("[-] enumerateServices::Client is null, cannot enumerate services.");
        return;
    }

    // Check is a discovery has already been done?
    if (!(client->discoveryDone())) {
        // Discover services on the client
        client->discoverServices();
    } else {
        if (debug_flag) {
            Serial.println("[!] enumerateServices::Discovery of Services Already Performed");
        }
    }
    Serial.println("[*] enumerateServices::Enumerating services...");

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
            if (!(debug_flag & verbose_flag)) {
                Serial.print("[*] enumerateServices::Comparing UUID [ ");
                Serial.print(uuid.str());
                Serial.print(" ] against the UUID: ");
                Serial.println(audio_serv_uuid.str());
            }
            // Check if the Current Service UUID matches a known Audio Service UUID
            #if (BLEUUID(audio_serv_uuid.str()) == uuid) {							# TODO: Redundant since audio_serv_uuid should be a BLEUUID object??
	    if (audio_serv_uuid == uuid) {
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
                        Serial.print("[+] enumerateServices::Found a Known Audio Characteristic UUID of [ ");
                        Serial.print(audio_char_uuid.str());
                        Serial.println(" ]");
                    } else {
                        if (debug_flag) {
                            Serial.print("[-] enumerateServices::Characteristic UUID [ ");
                            Serial.print(audio_char_uuid.str());
                            Serial.println(" ] is NOT a Knonw Characteristic Audio UUID");
                        }
                    }
                }
            } else {
                if (debug_flag) {
                    Serial.print("[-] enumerateServices::UUID [ ");
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

// Function to Time out a Wait Time
void timer_wait(uint16_t time_to_wait_ms) {
    millis_timer = millis();
    do {
        if (debug_flag) {
            Serial.print(".");
        }
    } while ((millis() - millis_timer) > time_to_wait_ms);
    //Serial.println();
}

// Function to Create a Connection to a Device
int8_t connect_to_target(BLEDevice& ble_device, BLEAdvertData& ble_target) {
    int timeout_wait = 2000;    // in milliseconds
    Serial.println("[*] connect_to_target::Connecting to Target Device");
    //delay(100);
    ble_device.configConnection()->connect(ble_target, timeout_wait);        // Attempt to Connect with 2000 millisecond timeout
    // Note: The .connect() attribute of BLEConnect Objcets allows for direct passing of BLEAdvertData
    Serial.print("[*] connect_to_target::Time wait of ");
    Serial.print(timeout_wait);
    Serial.print(" milliseconds");
    //delay(timeout_wait);    // Note: For some reason this causes a hang....
    timer_wait(timeout_wait); Serial.println();
    Serial.println("[*] connect_to_target::Obtaining the Connection ID");
    //delay(100);
    // Create a connection ID from the established aforementioned connection
    int8_t connID = ble_device.configConnection()->getConnId(ble_target);
    if (!debug_flag) {
        Serial.print("[*] connect_to_target::Connection ID: ");
        Serial.println(connID);
    }
    // Attempt to wait until the device is connected?
    Serial.print("[*] connect_to_target::Confirming connection to target");
    //do {
        //Serial.print(".");
        //delay(1000);
        //timer_wait(1000);
    //} while (!ble_device.connected(connID));
    Serial.println();

    if (!ble_device.connected(connID)) {
        Serial.println("[-] connect_to_target::BLE not connected");
    } else {
        Serial.println("[+] connect_to_target::BLE connected");

        Serial.println("[*] connect_to_target::Processing Target Device");
        // Process the New Client from the New Connection
        client = processNewClient(ble_device, ble_target, connID);

        Serial.println("[*] connect_to_target::Enumerating the Target Device");
        // Enumerate the New Client
        enumerateDevice(client, ble_target);

        Serial.println("[*] connect_to_target::Searching for Ancient Artifacts");
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

    Serial.println("[*] connect_to_target::Returning the Connection ID");
    // Return the connection ID
    return connID;
}

// Function for Scanning for Devices; Note: Moving this here might be breaking the central BLE CPU threa/operation
void scan_for_target_device(BLEDevice ble_device, BLEAdvertData target_device) {
    // Scan for Devices in Proximity
    ble_device.configScan()->startScan(2000);      // Scan for 2000 milliseconds
    Serial.print("[*] Attempting to Connect to Target Device: ");
    if (target_device.hasName()) {
        Serial.println(target_device.getName());
    } else {
        Serial.println(target_device.getAddr().str());
    }
    // Connect to the Target Device; assuming it was seen
    int8_t connID = connect_to_target(ble_device, target_device);
}

// Function for Processing a New Client Device for the BLE GATT Server's Tracking
BLEClient* processNewClient(BLEDevice ble_device, BLEAdvertData advert_data, int8_t connection_id) {
    int discovery_wait_time = 1000;
    // Perform initialization for the New Client
    ble_device.configClient();
    // Create the New Client Object based on the Device Connection ID
    BLEClient* new_client = ble_device.addClient(connection_id);
    // Discover/Enumerate information about the New Device

    enumerateDevice(new_client, advert_data);
    
    new_client->discoverServices();
    Serial.print("[*] processNewClient::Discovering services of the connected device");
    // Wait until the Services are Resolved
    do {
        Serial.print(".");
        //delay(discovery_wait_time);
        timer_wait(discovery_wait_time);
    } while (!(new_client->discoveryDone()));   // Continue to wait until the Service Discovery Status becomes True
    Serial.println();
    // Perform Connection-Type (e.g. Trusted, Bonded) Examination of the New Client

    // Incorporate the New Client device into the appropriate structures for tracking the new client device
    //  - Note: The purpose of this is to provide ease of search, enumeration, association, and recall for later BLE GATT Server functionality

    return new_client;
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
                target_identified = true;
            }
        } else {
            if (debug_flag & verbose_flag) {
                Serial.println("[-] scanCB::False Positive for Device Detection"); 
            }
            target_identified = false;
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
    
    Serial.println("[*] Setup::Starting Scan of Devices");
    //scan_for_target_device(BLE, targetDevice);
    // Scan for Devices in Proximity
    BLE.configScan()->startScan(2000);      // Scan for 2000 milliseconds
    //delay(2000);
    timer_wait(2000);
    if (target_identified) {
        Serial.print("[*] Setup::Attempting to Connect to Target Device: ");
        if (targetDevice.hasName()) {
            Serial.println(targetDevice.getName());
        } else {
            Serial.println(targetDevice.getAddr().str());
        }
        // Connect to the Target Device; assuming it was seen
        //int8_t connID = connect_to_target(BLE, targetDevice);
        target_conn_id = connect_to_target(BLE, targetDevice);
        // Debug returned Connection ID
        Serial.print("[*] Setup::Returned Connection ID of ");
        Serial.println(target_conn_id);

//        int timeout_wait = 2000;    // in milliseconds
//        Serial.println("[*] connect_to_target::Connecting to Target Device");
//        delay(100);
//        BLE.configConnection()->connect(targetDevice, timeout_wait);        // Attempt to Connect with 2000 millisecond timeout
//        // Note: The .connect() attribute of BLEConnect Objcets allows for direct passing of BLEAdvertData
//        Serial.print("[*] connect_to_target::Time wait of ");
//        Serial.print(2000);   //timeout_wait);
//        Serial.println(" milliseconds");
//        delay(timeout_wait);    // Note: For some reason this causes a hang....
//        Serial.println("[*] connect_to_target::Obtaining the Connection ID");
//        delay(100);
//        // Create a connection ID from the established aforementioned connection
//        int8_t connID = BLE.configConnection()->getConnId(targetDevice);
//        if (!debug_flag) {
//            Serial.print("[*] Connection ID: ");
//            Serial.println(connID);
//        }
//
//        if (!BLE.connected(connID)) {
//            Serial.println("BLE not connected");
//        } else {
//            Serial.println("BLE connected");
//
//            Serial.println("[*] Processing Target Device");
//            // Process the New Client from the New Connection
//            client = processNewClient(BLE, targetDevice, connID);
//            
//            Serial.println("[*] Enumerating the Target Device");
//            // Enumerate the New Client
//            enumerateDevice(client, targetDevice);
//
//            Serial.println("[*] Searching for Ancient Artifacts");
//            // Search and Return the specfic UART_SERVICE_UUID
//            UartService = client->getService(UART_SERVICE_UUID);
//            // Check if the UART Service was found
//            if (UartService != nullptr) {
//                // Search and Return the specific CHARACTERISTIC_UUID_TX
//                Tx = UartService->getCharacteristic(CHARACTERISTIC_UUID_TX);
//                // Check if the TX Characteristic was found
//                if (Tx != nullptr) {
//                    Serial.println("TX characteristic found");
//                    Tx->setBufferLen(STRING_BUF_SIZE);
//                    Tx->setNotifyCallback(notificationCB);
//                    Tx->enableNotifyIndicate();
//                }
//                // Search and Return the specific CHARACTERISTIC_UUID_RX
//                Rx = UartService->getCharacteristic(CHARACTERISTIC_UUID_RX);
//                // Check if the RX Characteristic was found
//                if (Rx != nullptr) {
//                    Serial.println("RX characteristic found");
//                    Rx->setBufferLen(STRING_BUF_SIZE);
//                }
//            }
//        }
    } else {
        Serial.println("[-] Target Device NOT identified during setup");
    }
}

void loop() {
    if (Serial.available()) {
        Rx->writeString(Serial.readString());
    }
    delay(100);
    // Testing Repeated Scans
    //delay(2000);
    timer_wait(2000);
    // Try another scan
    //BLE.configScan()->startScan(2000);
    //  - Note: The scanning below leverages the function calls, where setup has all the code written within the same function
    //    -> Reason: BLE operation is.... ethemeral....
    if (target_identified) {
        if (!BLE.connected(target_conn_id)) {
            Serial.print("[-] Loop::Device Connection ID [ ");
            Serial.print(target_conn_id);
            Serial.println(" ] is NOT connected.... Searching for Target Device");
            // Re-attempt the scanning process
            Serial.println("[*] Loop::Starting Scan of Devices");
            //scan_for_target_device(BLE, targetDevice);
            // Scan for Devices in Proximity
            BLE.configScan()->startScan(2000);      // Scan for 2000 milliseconds
            Serial.print("[*] Loop::Attempting to Connect to Target Device: ");
            if (targetDevice.hasName()) {
                Serial.println(targetDevice.getName());
            } else {
                Serial.println(targetDevice.getAddr().str());
            }
            // Connect to the Target Device; assuming it was seen
            //int8_t connID = connect_to_target(BLE, targetDevice);
            target_conn_id = connect_to_target(BLE, targetDevice);
            // Debug returned Connection ID
            Serial.print("[*] Loop::Returned Connection ID of ");
            Serial.println(target_conn_id);
        } else {
            Serial.print("[*] Loop::Device Connection ID [ ");
            Serial.print(target_conn_id);
            Serial.println(" ] is connected");
        }
    } else {
        Serial.println("[-] Loop::Target has NOT been identified");
        // Rescan
        BLE.configScan()->startScan(2000);      // Scan for 2000 milliseconds
    }

    // Test to see if the target device was found
    //  - Note: The DEFAULT for the Advert Data Name is "", HENCE the .hasName() check for validity
    if (targetDevice.hasName()) {
        Serial.println("[+] DEVICE HAS BEEN FOUND!");
    } else {
        Serial.println("[-] DEVICE NOT FOUND!");
    }
}
