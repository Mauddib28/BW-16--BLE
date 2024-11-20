#include "BLEDevice.h"
#include "gap.h"
#include "gap_le.h"
#include <algorithm>    // For std::min
#include <cstdlib>     // For std::rand

// Standard Bluetooth SIG UUIDs
#define AUDIO_SERVICE_UUID        0x1859  // LE Audio Service
#define AUDIO_CONTROL_UUID       0x2BC3  // Audio Control Point
#define AUDIO_STATE_UUID         0x2BC4  // Audio State

// Nordic UART Service UUIDs (16-bit portions)
#define NUS_SERVICE_UUID_MSB     0x6E40  // Most significant 16 bits
#define NUS_SERVICE_UUID_LSB     0x0001  // Least significant 16 bits
#define NUS_RX_UUID             0x6E42  // Nordic UART RX Characteristic
#define NUS_TX_UUID             0x6E41  // Nordic UART TX Characteristic

// Full 128-bit UUIDs for Nordic UART Service
const uint8_t NUS_BASE_UUID[] = {0x9E, 0xCA, 0xDC, 0x24, 0x0E, 0xE5, 0xA9, 0xE0, 
                                0x93, 0xF3, 0xA3, 0xB5, 0x00, 0x00, 0x40, 0x6E};

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
public:
    BLECentralHub() : active_connections(0), 
                      audio_source_conn_id(0xFF), 
                      audio_sink_conn_id(0xFF) {
        memset(devices, 0, sizeof(devices));
    }

    ~BLECentralHub() {
        if (uartRxChar) delete uartRxChar;
        if (uartTxChar) delete uartTxChar;
    }

    bool begin() {
        // Initialize BLE stack
        if (!BLE.init()) {
            Serial.println("BLE init failed");
            return false;
        }

        // Set device name
        BLE.setDeviceName("BLE Audio Hub");

        // Configure callbacks using GAP layer
        gap_le_register_cb(GAP_LE_MSG_EVENT_CONN_STATE_CHANGE, 
                          [](uint8_t conn_id, uint8_t new_state, uint16_t disc_cause) {
                              if (new_state == GAP_CONN_STATE_CONNECTED) {
                                  connectionCallback(conn_id);
                              } else if (new_state == GAP_CONN_STATE_DISCONNECTED) {
                                  disconnectionCallback(conn_id, disc_cause);
                              }
                          });

        // Configure security settings
        configureSecurity();

        // Initialize services
        initializeAudioProfile();
        initializeUARTService();

        // Start scanning
        startScanning();

        // Register discovery callback
        gap_le_register_cb(GAP_LE_MSG_EVENT_DISCOVERY_RESULT, 
                          [](uint8_t conn_id, T_LE_DISCOVERY_RESULT* result) {
                              getInstance().handleServiceDiscovery(conn_id, result);
                          });

        return true;
    }

    bool initiateSecurityRequest(uint8_t conn_id) {
        // Use the GAP security parameter setting
        T_GAP_CAUSE cause = gap_set_security_param(GAP_PARAM_SECURITY_REQUEST, sizeof(uint8_t), &conn_id);
        return cause == GAP_CAUSE_SUCCESS;
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

private:
    ConnectedDevice devices[MAX_CONNECTIONS];
    uint8_t active_connections;
    uint8_t audio_source_conn_id;
    uint8_t audio_sink_conn_id;
    BLECharacteristic* uartTxChar;
    BLECharacteristic* uartRxChar;

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
                    default:
                        devices[i].authenticated = false;
                        devices[i].bonded = false;
                        break;
                }
                devices[i].security_level = static_cast<uint8_t>(new_state);
                break;
            }
        }
    }

    void configureSecurity() {
        // Enhanced security configuration
        T_GAP_LE_SECURITY_PARAM security_param;
        
        // Require MITM protection and Secure Connections
        security_param.auth_req = GAP_AUTHEN_BIT_BONDING_FLAG |
                                GAP_AUTHEN_BIT_MITM_FLAG |
                                GAP_AUTHEN_BIT_SC_FLAG;
                                
        // Support both display and keyboard for flexible pairing
        security_param.io_cap = GAP_IO_CAP_KEYBOARD_DISPLAY;
        
        // Configure OOB and key sizes
        security_param.oob_enable = false;
        security_param.key_size = 16;  // Maximum key size

        // Key distribution for both initiator and responder
        // Using the correct AmebaBLE API constants
        security_param.initiator_key_dist = GAP_KDIST_BITS_ENCKEY | GAP_KDIST_BITS_IDKEY;
        security_param.responder_key_dist = GAP_KDIST_BITS_ENCKEY | GAP_KDIST_BITS_IDKEY;
        
        gap_le_set_param(GAP_PARAM_BOND_SECURITY_PARAMS, sizeof(security_param), &security_param);
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
        audioControlChar.setProperties(GATT_CHAR_PROP_READ | 
                                     GATT_CHAR_PROP_WRITE | 
                                     GATT_CHAR_PROP_NOTIFY);
        audioControlChar.setPermissions(GATT_PERM_READ | GATT_PERM_WRITE);
        audioControlChar.setWriteCallback(handleAudioControl);
        
        BLECharacteristic audioStateChar(AUDIO_STATE_UUID);
        audioStateChar.setProperties(GATT_CHAR_PROP_READ | 
                                   GATT_CHAR_PROP_NOTIFY);
        audioStateChar.setPermissions(GATT_PERM_READ);
    }

    static void handleAudioControl(BLECharacteristic* characteristic) {
        uint8_t* data = characteristic->getDataBuffer();
        uint16_t length = characteristic->getDataLength();
        
        if (length < 1) return;
        
        switch (data[0]) {
            case 0x01: // Start streaming
                getInstance().startAudioStream();
                break;
            case 0x02: // Stop streaming
                getInstance().stopAudioStream();
                break;
            case 0x03: // Configure codec
                getInstance().configureCodec(data, length);
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
        uartRxChar = new BLECharacteristic(NUS_RX_UUID);
        uartRxChar->setProperties(GATT_CHAR_PROP_WRITE | 
                                 GATT_CHAR_PROP_WRITE_NO_RSP);
        uartRxChar->setPermissions(GATT_PERM_WRITE);
        uartRxChar->setWriteCallback(handleUARTRx);
        
        uartTxChar = new BLECharacteristic(NUS_TX_UUID);
        uartTxChar->setProperties(GATT_CHAR_PROP_READ | 
                                 GATT_CHAR_PROP_NOTIFY);
        uartTxChar->setPermissions(GATT_PERM_READ);
        uartTxChar->setCCCDCallback(handleUARTTx);
    }

    BLECharacteristic* getUARTTxCharacteristic() {
        return uartTxChar;
    }

    static void handleUARTRx(BLECharacteristic* characteristic, uint8_t conn_id) {
        uint8_t* data = characteristic->getDataBuff();
        uint16_t length = characteristic->getDataLen();

        // Forward data to paired UART device
        getInstance().forwardUARTData(conn_id, data, length);
    }

    static void handleUARTTx([[maybe_unused]] BLECharacteristic* characteristic, 
                           uint8_t conn_id, 
                           uint16_t device_cccd) {
        // Handle notification subscription changes
        if (device_cccd & GATT_CLIENT_CHAR_CONFIG_NOTIFY) {
            Serial.print("UART notifications enabled for connection ");
            Serial.println(conn_id);
        } else {
            Serial.print("UART notifications disabled for connection ");
            Serial.println(conn_id);
        }
    }

    void forwardUARTData(uint8_t source_conn_id, uint8_t* data, uint8_t length) {
        // Find paired UART device and forward data
        for (int i = 0; i < MAX_CONNECTIONS; i++) {
            if (devices[i].conn_id != source_conn_id && devices[i].has_uart) {
                // Send data in chunks if necessary
                uint8_t offset = 0;
                while (offset < length) {
                    uint8_t chunk_size = std::min(UART_PACKET_SIZE, length - offset);
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
        if (txChar) {
            // Check if notifications are enabled using the BLE API
            uint16_t cccd_value;
            if (txChar->getCCCD(conn_id, &cccd_value) == 0 && 
                (cccd_value & GATT_CLIENT_CHAR_CONFIG_NOTIFY)) {
                txChar->setData(data, length);
                txChar->notify(conn_id);
            }
        }
    }

    // Connection Management Methods
    void handleConnection(uint8_t conn_id, const char* name, const char* address) {
        if (addDevice(conn_id, name, address)) {
            // Discover services and characteristics
            discoverServices(conn_id);
            
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
        // Start service discovery using direct GATT operations
        T_GAP_DEV_STATE new_state = {
            .gap_init_state = 0,
            .gap_adv_state = 0,
            .gap_scan_state = 0,
            .gap_conn_state = 0
        };
        
        le_get_gap_param(GAP_PARAM_DEV_STATE, &new_state);
        
        if (new_state.gap_init_state == GAP_INIT_STATE_STACK_READY) {
            // Register for GATT discovery events
            gatt_discovery_reg_cb(handleGattDiscovery);
            
            // Start GATT discovery
            gatt_discovery_services(conn_id, 0x0001, 0xFFFF);
        }
    }

    static void handleGattDiscovery(T_GATT_DISCOVERY_EVT evt_type, uint8_t conn_id, uint16_t att_handle, uint16_t uuid16) {
        if (evt_type == GATT_DISCOVERY_SERVICES) {
            // Check for audio and UART services
            if (uuid16 == AUDIO_SERVICE_UUID) {
                getInstance().devices[getInstance().getDeviceIndex(conn_id)].is_audio_source = true;
                Serial.println("Found Audio Service");
            } else if ((uuid16 & 0xFFFF) == NUS_SERVICE_UUID_MSB) {
                getInstance().devices[getInstance().getDeviceIndex(conn_id)].has_uart = true;
                Serial.println("Found UART Service");
            }
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
        // Configure scan parameters
        le_scan_param_set(GAP_PARAM_SCAN_MODE, sizeof(uint8_t), (void *)&GAP_SCAN_MODE_ACTIVE);
        
        uint16_t scan_interval = 0x20;  // 20ms
        le_scan_param_set(GAP_PARAM_SCAN_INTERVAL, sizeof(uint16_t), &scan_interval);
        
        uint16_t scan_window = 0x10;    // 10ms
        le_scan_param_set(GAP_PARAM_SCAN_WINDOW, sizeof(uint16_t), &scan_window);
        
        uint8_t scan_filter_policy = GAP_SCAN_FILTER_ANY;
        le_scan_param_set(GAP_PARAM_SCAN_FILTER_POLICY, sizeof(uint8_t), &scan_filter_policy);
        
        uint8_t scan_filter_duplicate = GAP_SCAN_FILTER_DUPLICATE_ENABLE;
        le_scan_param_set(GAP_PARAM_SCAN_FILTER_DUPLICATE, sizeof(uint8_t), &scan_filter_duplicate);
        
        // Start scanning
        le_scan_start();
    }

    void handleScanResult(T_LE_CB_DATA *p_data) {
        if (p_data->cb_type != GAP_MSG_LE_SCAN_INFO) {
            return;
        }

        T_LE_SCAN_INFO *scan_info = p_data->p_le_scan_info;
        
        // Process advertising data
        uint8_t *adv_data = scan_info->data;
        uint8_t adv_len = scan_info->data_len;
        
        bool hasTargetService = false;
        
        // Parse advertising data for service UUIDs
        for (int i = 0; i < adv_len; ) {
            uint8_t field_len = adv_data[i++];
            uint8_t field_type = adv_data[i++];
            
            if (field_type == GAP_ADTYPE_16BIT_MORE || 
                field_type == GAP_ADTYPE_16BIT_COMPLETE) {
                
                for (int j = 0; j < field_len - 1; j += 2) {
                    uint16_t uuid = adv_data[i + j] | (adv_data[i + j + 1] << 8);
                    if (uuid == AUDIO_SERVICE_UUID || uuid == NUS_SERVICE_UUID_MSB) {
                        hasTargetService = true;
                        break;
                    }
                }
            }
            
            i += field_len - 1;
        }
        
        if (hasTargetService && active_connections < MAX_CONNECTIONS) {
            // Stop scanning before connecting
            le_scan_stop();
            
            // Connect to device with correct parameter order
            le_connect(GAP_PHYS_CONN_INIT_1M_BIT,  // Initial PHY parameter
                      scan_info->bd_addr,           // Remote device address
                      scan_info->remote_addr_type,  // Remote address type
                      GAP_LOCAL_ADDR_LE_PUBLIC,     // Local address type
                      1000);                        // Connection timeout 1000ms
        }
    }

    static void connectionCallback(uint8_t conn_id) {
        Serial.print("Connected to device ");
        Serial.println(conn_id);
        
        // For now, we'll just use the connection ID since we can't get the address directly
        // We can get more device information during service discovery
        getInstance().handleConnection(conn_id, "Unknown", "Unknown");
    }

    static void disconnectionCallback(uint8_t conn_id, uint16_t disc_cause) {
        Serial.print("Device ");
        Serial.print(conn_id);
        Serial.print(" disconnected, reason: ");
        Serial.println(disc_cause);
        
        getInstance().handleDisconnection(conn_id);
        
        // Resume scanning if we have room for more connections
        if (getInstance().active_connections < MAX_CONNECTIONS) {
            getInstance().startScanning();
        }
    }

    void handleError(uint8_t error_code, uint8_t conn_id) {
        Serial.print("Error ");
        Serial.print(error_code);
        Serial.print(" on connection ");
        Serial.println(conn_id);
        
        // For now, let's handle all errors generically until we can confirm
        // the exact error codes from the AmebaBLE API
        handleGenericError(error_code, conn_id);
    }

    void handleGenericError(uint8_t error_code, uint8_t conn_id) {
        Serial.print("Generic error ");
        Serial.print(error_code);
        Serial.print(" on connection ");
        Serial.println(conn_id);
        
        // Disconnect on any error for now
        le_disconnect(conn_id);
    }
};

void setup() {
    Serial.begin(115200);
    while (!Serial) delay(10);

    Serial.println("BLE Central Audio Hub");
    
    if (!BLECentralHub::getInstance().begin()) {
        Serial.println("Failed to initialize BLE Central!");
        while (1) delay(100);
    }
}

void loop() {
    // Main processing loop
    delay(10);
}

