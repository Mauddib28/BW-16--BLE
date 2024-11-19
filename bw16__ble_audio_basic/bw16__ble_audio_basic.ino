#include "BLEDevice.h"
#include "gap.h"
#include "gap_le.h"

// Standard Bluetooth SIG UUIDs
#define AUDIO_SERVICE_UUID        "1859" // LE Audio Service
#define AUDIO_CONTROL_UUID       "2BC3" // Audio Control Point
#define AUDIO_STATE_UUID         "2BC4" // Audio State
#define NUS_SERVICE_UUID         "6E400001-B5A3-F393-E0A9-E50E24DCCA9E" // Nordic UART Service
#define NUS_RX_UUID             "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define NUS_TX_UUID             "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

// Maximum number of simultaneous connections
#define MAX_CONNECTIONS 3

// UART Service Constants
#define UART_BUFFER_SIZE 256
#define UART_PACKET_SIZE 20

// Device tracking structure
struct ConnectedDevice {
    uint8_t conn_id;
    char name[32];
    char address[18];
    bool authenticated;
    bool bonded;
    uint8_t security_level;
    bool is_audio_source;
    bool is_audio_sink;
    bool has_uart;
};

// Connection event types
enum ConnectionEvent {
    DEVICE_CONNECTED,
    DEVICE_DISCONNECTED,
    SECURITY_UPDATED,
    PAIRING_STARTED,
    PAIRING_COMPLETE,
    PAIRING_FAILED
};

// Audio Profile Constants
#define LC3_SAMPLING_RATE_48KHZ    0x02
#define LC3_FRAME_DURATION_10MS    0x01
#define MAX_AUDIO_SDU_SIZE         155  // For 48kHz, 10ms frame
#define AUDIO_CONTEXT_TYPE_MEDIA   0x0001

// Audio Profile Structures
struct AudioConfiguration {
    uint8_t codec_id;          // LC3 = 0x06
    uint8_t sampling_rate;     // 48kHz = 0x02
    uint8_t frame_duration;    // 10ms = 0x01
    uint16_t octets_per_frame;
    uint8_t frame_blocks_per_sdu;
};

class BLECentralHub {
private:
    ConnectedDevice devices[MAX_CONNECTIONS];
    uint8_t active_connections;
    uint8_t audio_source_conn_id;
    uint8_t audio_sink_conn_id;

    // Security states
    enum SecurityState {
        UNPAIRED,
        PAIRING_IN_PROGRESS,
        PAIRED_NO_MITM,
        PAIRED_WITH_MITM,
        BONDED
    };

    AudioConfiguration audioConfig;
    
    struct UARTBuffer {
        uint8_t data[UART_BUFFER_SIZE];
        uint16_t read_index;
        uint16_t write_index;
    };

    UARTBuffer uart_rx_buffer;
    UARTBuffer uart_tx_buffer;

    void handleSecurityEvent(uint8_t conn_id, SecurityState new_state) {
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (devices[i].conn_id == conn_id) {
                switch (new_state) {
                    case PAIRED_WITH_MITM:
                        devices[i].authenticated = true;
                        break;
                    case BONDED:
                        devices[i].bonded = true;
                        break;
                }
                devices[i].security_level = static_cast<uint8_t>(new_state);
                break;
            }
        }
    }

    void configureSecurity() {
        // Enhanced security configuration
        gap_security_param_t security_param;
        
        // Require MITM protection and Secure Connections
        security_param.auth_req = GAP_AUTHEN_BIT_BONDING_FLAG |
                                GAP_AUTHEN_BIT_MITM_FLAG |
                                GAP_AUTHEN_BIT_SC_FLAG;
                                
        // Support both display and keyboard for flexible pairing
        security_param.io_cap = GAP_IO_CAP_KEYBOARD_DISPLAY;
        
        // Configure OOB and key sizes
        security_param.oob_enable = false;
        security_param.key_size = 16;  // Maximum key size
        security_param.initiator_key_dist = GAP_KDIST_ENCKEY | GAP_KDIST_IDKEY;
        security_param.responder_key_dist = GAP_KDIST_ENCKEY | GAP_KDIST_IDKEY;
        
        // Enable security request on connection
        uint8_t sec_req_enable = true;
        le_bond_set_param(GAP_PARAM_BOND_SEC_REQ_ENABLE, sizeof(uint8_t), &sec_req_enable);
        
        // Set security parameters
        le_bond_set_param(GAP_PARAM_BOND_SEC_PARAM, sizeof(gap_security_param_t), &security_param);
        
        // Register security callbacks
        le_bond_register_cb(bondCallback);
    }

    static void bondCallback(uint8_t conn_id, uint8_t status, uint8_t fail_reason) {
        switch (status) {
            case GAP_BOND_STATE_START:
                Serial.printf("Pairing started with device %d\n", conn_id);
                break;
                
            case GAP_BOND_STATE_COMPLETE:
                Serial.printf("Pairing complete with device %d\n", conn_id);
                // Update device security status
                getInstance().handleSecurityEvent(conn_id, BONDED);
                break;
                
            case GAP_BOND_STATE_FAIL:
                Serial.printf("Pairing failed with device %d, reason: %d\n", conn_id, fail_reason);
                break;
        }
    }

    bool addDevice(uint8_t conn_id, const char* name, const char* address) {
        if (active_connections >= MAX_CONNECTIONS) {
            return false;
        }

        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (devices[i].conn_id == 0) {
                devices[i].conn_id = conn_id;
                strncpy(devices[i].name, name, sizeof(devices[i].name) - 1);
                strncpy(devices[i].address, address, sizeof(devices[i].address) - 1);
                devices[i].authenticated = false;
                devices[i].bonded = false;
                devices[i].security_level = static_cast<uint8_t>(UNPAIRED);
                active_connections++;
                return true;
            }
        }
        return false;
    }

    bool removeDevice(uint8_t conn_id) {
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (devices[i].conn_id == conn_id) {
                memset(&devices[i], 0, sizeof(ConnectedDevice));
                active_connections--;
                return true;
            }
        }
        return false;
    }

    void initializeAudioProfile() {
        // Configure LC3 codec parameters
        audioConfig.codec_id = 0x06;  // LC3 codec
        audioConfig.sampling_rate = LC3_SAMPLING_RATE_48KHZ;
        audioConfig.frame_duration = LC3_FRAME_DURATION_10MS;
        audioConfig.octets_per_frame = MAX_AUDIO_SDU_SIZE;
        audioConfig.frame_blocks_per_sdu = 1;

        // Create Audio Service characteristics
        BLECharacteristic audioControlChar(AUDIO_CONTROL_UUID);
        audioControlChar.setProperties(BLERead | BLEWrite | BLENotify);
        audioControlChar.setPermissions(GATT_PERM_READ | GATT_PERM_WRITE);
        audioControlChar.setWriteCallback(handleAudioControl);
        
        BLECharacteristic audioStateChar(AUDIO_STATE_UUID);
        audioStateChar.setProperties(BLERead | BLENotify);
        audioStateChar.setPermissions(GATT_PERM_READ);
    }

    static void handleAudioControl(BLECharacteristic* characteristic) {
        uint8_t* data = characteristic->getData();
        uint8_t length = characteristic->getDataLength();
        
        if (length < 1) return;
        
        switch (data[0]) {
            case 0x01: // Start streaming
                startAudioStream();
                break;
            case 0x02: // Stop streaming
                stopAudioStream();
                break;
            case 0x03: // Configure codec
                configureCodec(data, length);
                break;
        }
    }

    void startAudioStream() {
        if (audio_source_conn_id == 0xFF || audio_sink_conn_id == 0xFF) {
            Serial.println("Audio source or sink not configured");
            return;
        }

        // Initialize LC3 encoder/decoder
        initializeLC3Codec();
        
        // Start audio stream between selected devices
        setupAudioStream(audio_source_conn_id, audio_sink_conn_id);
    }

    void stopAudioStream() {
        // Stop active audio stream
        terminateAudioStream();
        
        // Clean up codec resources
        cleanupLC3Codec();
    }

    void configureCodec(uint8_t* config_data, uint8_t length) {
        if (length < 4) return;
        
        // Parse configuration data
        audioConfig.sampling_rate = config_data[1];
        audioConfig.frame_duration = config_data[2];
        audioConfig.octets_per_frame = config_data[3];
        
        // Update codec configuration
        updateLC3Configuration();
    }

    void initializeLC3Codec() {
        // Initialize LC3 encoder/decoder with current configuration
        // Note: Implementation depends on available LC3 library
    }

    void cleanupLC3Codec() {
        // Clean up LC3 codec resources
    }

    void setupAudioStream(uint8_t source_id, uint8_t sink_id) {
        // Configure CIS (Connected Isochronous Stream)
        // Set up audio routing between source and sink
    }

    void terminateAudioStream() {
        // Stop CIS and clean up audio routing
    }

    void updateLC3Configuration() {
        // Update running codec with new configuration
    }

    void initializeUARTService() {
        // Create UART Service characteristics
        BLECharacteristic uartRxChar(NUS_RX_UUID);
        uartRxChar.setProperties(BLEWrite | BLEWriteWithoutResponse);
        uartRxChar.setPermissions(GATT_PERM_WRITE);
        uartRxChar.setWriteCallback(handleUARTRx);
        
        BLECharacteristic uartTxChar(NUS_TX_UUID);
        uartTxChar.setProperties(BLERead | BLENotify);
        uartTxChar.setPermissions(GATT_PERM_READ);
        uartTxChar.setNotifyCallback(handleUARTTx);
    }

    static void handleUARTRx(BLECharacteristic* characteristic) {
        uint8_t* data = characteristic->getData();
        uint8_t length = characteristic->getDataLength();
        uint8_t conn_id = characteristic->getConnId();

        // Forward data to paired UART device
        getInstance().forwardUARTData(conn_id, data, length);
    }

    static void handleUARTTx(BLECharacteristic* characteristic, uint8_t conn_id) {
        // Handle notification subscription changes
        if (characteristic->isSubscribed(conn_id)) {
            Serial.printf("UART notifications enabled for connection %d\n", conn_id);
        } else {
            Serial.printf("UART notifications disabled for connection %d\n", conn_id);
        }
    }

    void forwardUARTData(uint8_t source_conn_id, uint8_t* data, uint8_t length) {
        // Find paired UART device and forward data
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (devices[i].conn_id != source_conn_id && devices[i].has_uart) {
                // Send data in chunks if necessary
                uint8_t offset = 0;
                while (offset < length) {
                    uint8_t chunk_size = min(UART_PACKET_SIZE, length - offset);
                    sendUARTData(devices[i].conn_id, &data[offset], chunk_size);
                    offset += chunk_size;
                }
                break;
            }
        }
    }

    void sendUARTData(uint8_t conn_id, uint8_t* data, uint8_t length) {
        // Get UART TX characteristic and send data
        BLECharacteristic* txChar = getUARTTxCharacteristic();
        if (txChar && txChar->isSubscribed(conn_id)) {
            txChar->writeValue(data, length, conn_id);
        }
    }

    // Connection Management Methods
    void handleConnection(uint8_t conn_id, const char* name, const char* address) {
        if (addDevice(conn_id, name, address)) {
            // Discover services and characteristics
            discoverServices(conn_id);
            
            // Request security upgrade if needed
            if (!isDeviceAuthenticated(conn_id)) {
                initiateSecurityRequest(conn_id);
            }
        }
    }

    void handleDisconnection(uint8_t conn_id) {
        // Clean up device resources
        removeDevice(conn_id);
        
        // Update audio routing if necessary
        if (conn_id == audio_source_conn_id || conn_id == audio_sink_conn_id) {
            stopAudioStream();
            audio_source_conn_id = 0xFF;
            audio_sink_conn_id = 0xFF;
        }
    }

    void discoverServices(uint8_t conn_id) {
        // Start service discovery
        BLEDevice* device = BLE.central(conn_id);
        if (device) {
            device->discoverAttributes();
        }
    }

    void handleServiceDiscovery(uint8_t conn_id, BLEService* service) {
        // Check for audio and UART services
        if (service->getUUID().equals(AUDIO_SERVICE_UUID)) {
            devices[getDeviceIndex(conn_id)].is_audio_source = true;
        } else if (service->getUUID().equals(NUS_SERVICE_UUID)) {
            devices[getDeviceIndex(conn_id)].has_uart = true;
        }
    }

    int getDeviceIndex(uint8_t conn_id) {
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (devices[i].conn_id == conn_id) {
                return i;
            }
        }
        return -1;
    }

    void startScanning() {
        BLEScan* scan = BLE.getScan();
        scan->setActiveScan(true);
        scan->setInterval(100);
        scan->setWindow(99);
        scan->start(0); // Continuous scanning
    }

    static void scanCallback(BLEAdvertisedDevice* advertisedDevice) {
        // Process discovered devices
        if (advertisedDevice->haveServiceUUID()) {
            if (advertisedDevice->isAdvertisingService(BLEUUID(AUDIO_SERVICE_UUID)) ||
                advertisedDevice->isAdvertisingService(BLEUUID(NUS_SERVICE_UUID))) {
                
                // Found a relevant device
                Serial.printf("Found device: %s\n", advertisedDevice->getName().c_str());
                
                // Attempt connection if we have room
                if (getInstance().active_connections < MAX_CONNECTIONS) {
                    BLE.stopScan();
                    BLE.connect(advertisedDevice);
                }
            }
        }
    }

    static void connectionCallback(uint8_t conn_id) {
        Serial.printf("Connected to device %d\n", conn_id);
        getInstance().handleConnection(conn_id, BLE.getPeerName(conn_id).c_str(), 
                                    BLE.getPeerAddress(conn_id).c_str());
    }

    static void disconnectionCallback(uint8_t conn_id, uint8_t reason) {
        Serial.printf("Device %d disconnected, reason: %d\n", conn_id, reason);
        getInstance().handleDisconnection(conn_id);
        
        // Resume scanning if we have room for more connections
        if (getInstance().active_connections < MAX_CONNECTIONS) {
            getInstance().startScanning();
        }
    }

    void handleError(uint8_t error_code, uint8_t conn_id) {
        Serial.printf("Error %d on connection %d\n", error_code, conn_id);
        
        switch (error_code) {
            case BLE_ERROR_CONNECTION_TIMEOUT:
                // Connection attempt timed out
                handleConnectionTimeout(conn_id);
                break;
                
            case BLE_ERROR_SECURITY_FAILED:
                // Security negotiation failed
                handleSecurityFailure(conn_id);
                break;
                
            case BLE_ERROR_GATT_WRITE_NOT_PERMITTED:
                // Write permission error
                handleWritePermissionError(conn_id);
                break;
                
            default:
                // Generic error handling
                handleGenericError(error_code, conn_id);
                break;
        }
    }

    void handleConnectionTimeout(uint8_t conn_id) {
        // Clean up any partial connection state
        removeDevice(conn_id);
        
        // Resume scanning
        if (active_connections < MAX_CONNECTIONS) {
            startScanning();
        }
    }

    void handleSecurityFailure(uint8_t conn_id) {
        // Log security failure
        Serial.printf("Security negotiation failed for device %d\n", conn_id);
        
        // Optionally retry security negotiation or disconnect
        if (isDeviceAuthenticated(conn_id)) {
            // If previously authenticated, try to re-establish security
            initiateSecurityRequest(conn_id);
        } else {
            // Otherwise disconnect
            BLE.disconnect(conn_id);
        }
    }

    void handleWritePermissionError(uint8_t conn_id) {
        // Check if device needs authentication
        if (!isDeviceAuthenticated(conn_id)) {
            initiateSecurityRequest(conn_id);
        }
    }

    void handleGenericError(uint8_t error_code, uint8_t conn_id) {
        // Log error
        Serial.printf("Generic error %d on connection %d\n", error_code, conn_id);
        
        // Attempt recovery based on error severity
        if (error_code >= BLE_ERROR_CRITICAL_BASE) {
            // Critical error - disconnect and cleanup
            BLE.disconnect(conn_id);
        }
    }

public:
    BLECentralHub() : active_connections(0), 
                      audio_source_conn_id(0xFF), 
                      audio_sink_conn_id(0xFF) {
        memset(devices, 0, sizeof(devices));
    }

    bool begin() {
        // Initialize BLE stack
        if (!BLE.init()) {
            Serial.println("BLE init failed");
            return false;
        }

        // Set device name
        BLE.setDeviceName("BLE Audio Hub");

        // Configure callbacks
        BLE.setEventHandler(BLEConnected, connectionCallback);
        BLE.setEventHandler(BLEDisconnected, disconnectionCallback);
        BLE.setScanCallback(scanCallback);

        // Configure security settings
        configureSecurity();

        // Initialize services
        initializeAudioProfile();
        initializeUARTService();

        // Start scanning
        startScanning();

        return true;
    }

    bool initiateSecurityRequest(uint8_t conn_id) {
        return le_bond_pair(conn_id) == GAP_SUCCESS;
    }

    bool isDeviceAuthenticated(uint8_t conn_id) {
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (devices[i].conn_id == conn_id) {
                return devices[i].authenticated;
            }
        }
        return false;
    }

    bool isDeviceBonded(uint8_t conn_id) {
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (devices[i].conn_id == conn_id) {
                return devices[i].bonded;
            }
        }
        return false;
    }

    void setAudioSource(uint8_t conn_id) {
        if (isDeviceAuthenticated(conn_id) && 
            devices[getDeviceIndex(conn_id)].is_audio_source) {
            audio_source_conn_id = conn_id;
        }
    }

    void setAudioSink(uint8_t conn_id) {
        if (isDeviceAuthenticated(conn_id)) {
            audio_sink_conn_id = conn_id;
        }
    }

    bool pairUARTDevices(uint8_t conn_id1, uint8_t conn_id2) {
        int idx1 = getDeviceIndex(conn_id1);
        int idx2 = getDeviceIndex(conn_id2);
        
        if (idx1 >= 0 && idx2 >= 0 && 
            devices[idx1].has_uart && devices[idx2].has_uart) {
            // Set up UART routing between devices
            return true;
        }
        return false;
    }

    static BLECentralHub& getInstance() {
        static BLECentralHub instance;
        return instance;
    }
};

BLECentralHub centralHub;

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("BLE Central Audio Hub");
    
    if (!centralHub.begin()) {
        Serial.println("Failed to initialize BLE Central!");
        while (1) delay(100);
    }
}

void loop() {
    // Main processing loop
    delay(10);
}

