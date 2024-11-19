#include <AmebaBLE.h>
#include <BLEDevice.h>
#include <BLEAdvertising.h>
#include <BLEClient.h>
#include <map>
#include <vector>

// Maximum number of simultaneous connections
#define MAX_CONNECTIONS 10
// Service UUIDs
#define UART_SERVICE_UUID        "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define AUDIO_SERVICE_UUID       "1234A000-B5A3-F393-E0A9-E50E24DCCA9E" // Custom UUID

// Standard BLE Audio UUIDs and constants based on Bluetooth SIG specifications
#define BAP_UNICAST_CLIENT_UUID        "184E" // Basic Audio Profile Unicast Client
#define BAP_BROADCAST_SOURCE_UUID      "184F" // Basic Audio Profile Broadcast Source
#define BAP_BROADCAST_SINK_UUID        "1850" // Basic Audio Profile Broadcast Sink
#define VOLUME_CONTROL_SERVICE_UUID     "1844"  // Volume Control Service
#define VOLUME_STATE_CHAR_UUID          "2B7D"
#define VOLUME_CONTROL_POINT_CHAR_UUID  "2B7C"
#define AUDIO_STREAM_ENDPOINT_CHAR_UUID "2BC6"

// Security-related definitions
#define MIN_KEY_SIZE 16
#define MAX_KEY_SIZE 16

// Device tracking structures
struct ConnectedDevice {
    String name;
    String address;
    bool isAuthenticated;
    bool isBonded;
    bool isAudioSource;
    bool isAudioSink;
    uint8_t connectionId;
    BLEClient* client;
};

class BLECentralManager {
private:
    std::map<String, ConnectedDevice> connectedDevices;
    ConnectedDevice* activeAudioSource = nullptr;
    ConnectedDevice* activeAudioSink = nullptr;
    
    // Device lists by category
    std::vector<ConnectedDevice*> publicDevices;
    std::vector<ConnectedDevice*> authenticatedDevices;
    std::vector<ConnectedDevice*> bondedDevices;

public:
    BLECentralManager() {
        BLE.init("BLE_Audio_Hub");
        // Note: RTL8720DN automatically negotiates the optimal MTU
    }

    bool startScanning() {
        BLE.configScan()->setInterval(1349);
        BLE.configScan()->setWindow(449);
        BLE.configScan()->setActiveScan(true);
        
        // Set scan callback
        BLE.configScan()->setScanCallback(new AdvertisedDeviceCallbacks());
        
        return BLE.startScan(5000); // Scan for 5 seconds
    }

    void handleNewConnection(BLEAdvertisedDevice* device) {
        ConnectedDevice newDevice;
        newDevice.name = device->getName().c_str();
        newDevice.address = device->getAddress().toString().c_str();
        newDevice.connectionId = connectedDevices.size() + 1;
        
        // Attempt connection
        BLEClient* client = BLE.central();
        if (client->connect(device->getAddress())) {
            newDevice.client = client;
            
            // Determine device capabilities and security level
            determineDeviceCapabilities(&newDevice);
            categorizeDevice(&newDevice);
            
            connectedDevices[newDevice.address] = newDevice;
        }
    }

private:
    void determineDeviceCapabilities(ConnectedDevice* device) {
        // Check for audio capabilities
        BLERemoteService* audioService = device->client->getService(BLEUUID(AUDIO_SERVICE_UUID));
        if (audioService != nullptr) {
            // Determine if device is source/sink based on characteristics
            device->isAudioSource = true; // or false based on characteristics
            device->isAudioSink = !device->isAudioSource;
        }
    }

    void categorizeDevice(ConnectedDevice* device) {
        if (device->isBonded) {
            bondedDevices.push_back(device);
        } else if (device->isAuthenticated) {
            authenticatedDevices.push_back(device);
        } else {
            publicDevices.push_back(device);
        }
    }
};

class AudioProfileManager {
private:
    // Audio Quality of Service parameters
    struct QoSConfiguration {
        uint8_t retransmissionCount;    // Number of retransmission attempts
        uint16_t maxTransportLatency;   // Maximum latency in milliseconds
        uint8_t minPresentationDelay;   // Minimum presentation delay
        uint8_t maxPresentationDelay;   // Maximum presentation delay
        uint8_t preferredQoSProfile;    // Preferred QoS profile (balanced/reliability/latency)
    };

    // Extended audio stream configuration
    struct AudioStreamConfig {
        uint8_t codecId;                // Codec identifier (LC3 = 0x06)
        uint16_t samplingFreq;          // Sampling frequency in Hz
        uint8_t framesDuration;         // Frame duration in ms
        uint8_t audioChannelAllocation; // Channel allocation bitmap
        uint16_t octetsPerFrame;        // Octets per audio frame
        uint8_t blocksPerSDU;           // Blocks per SDU
        uint8_t framingMode;            // Framing mode (0=unframed, 1=framed)
        uint16_t maxSDUSize;            // Maximum SDU size
        uint8_t retransmissionNumber;   // Number of retransmissions
        uint16_t maxTransportLatency;   // Maximum transport latency
        uint16_t presentationDelay;     // Presentation delay
        uint16_t contextType;           // Audio context type bitmap
    };

    enum class StreamState {
        IDLE,
        CONFIGURING,
        ENABLING,
        STREAMING,
        DISABLING,
        ERROR
    };

    StreamState currentState;
    QoSConfiguration qosConfig;
    AudioStreamConfig activeConfig;
    BLERemoteCharacteristic* streamEndpoint;
    BLERemoteCharacteristic* volumeControl;

public:
    AudioProfileManager() {
        initializeDefaultConfigs();
    }

    bool configureAudioStream(BLEClient* client, bool isSource) {
        if (!client->connected()) {
            return false;
        }

        // Discover audio services
        BLERemoteService* audioService = client->getService(BLEUUID(AUDIO_SERVICE_UUID));
        if (!audioService) {
            Serial.println("Audio service not found");
            return false;
        }

        // Get stream endpoint characteristic
        streamEndpoint = audioService->getCharacteristic(BLEUUID(AUDIO_STREAM_ENDPOINT_CHAR_UUID));
        if (!streamEndpoint) {
            Serial.println("Stream endpoint not found");
            return false;
        }

        // Configure stream based on role (source/sink)
        if (!configureStreamEndpoint(isSource)) {
            return false;
        }

        // Set up volume control if available
        setupVolumeControl(client);

        currentState = StreamState::CONFIGURED;
        return true;
    }

    bool startStreaming() {
        if (currentState != StreamState::CONFIGURED) {
            return false;
        }

        // Send stream start command
        uint8_t startCmd[] = {0x01}; // Example start command
        if (!streamEndpoint->writeValue(startCmd, sizeof(startCmd))) {
            return false;
        }

        currentState = StreamState::STREAMING;
        return true;
    }

    bool stopStreaming() {
        if (currentState != StreamState::STREAMING) {
            return false;
        }

        // Send stream stop command
        uint8_t stopCmd[] = {0x00}; // Example stop command
        if (!streamEndpoint->writeValue(stopCmd, sizeof(stopCmd))) {
            return false;
        }

        currentState = StreamState::CONFIGURED;
        return true;
    }

private:
    void initializeDefaultConfigs() {
        qosConfig = {
            .retransmissionCount = 2,
            .maxTransportLatency = 20,
            .minPresentationDelay = 10,
            .maxPresentationDelay = 40,
            .preferredQoSProfile = 1
        };

        activeConfig = {
            .codecId = 0x06, // LC3 codec
            .samplingFreq = 48000,
            .framesDuration = 10,
            .audioChannelAllocation = 0x03, // Stereo
            .octetsPerFrame = 120,
            .blocksPerSDU = 1,
            .framingMode = 1,
            .maxSDUSize = 512,
            .retransmissionNumber = qosConfig.retransmissionCount,
            .maxTransportLatency = qosConfig.maxTransportLatency,
            .presentationDelay = 15,
            .contextType = 0x0002
        };

        currentState = StreamState::IDLE;
    }

    bool configureStreamEndpoint(bool isSource) {
        // Pack configuration into byte array
        uint8_t config[20];
        memcpy(config, &activeConfig, sizeof(activeConfig));
        
        // Write configuration to stream endpoint
        return streamEndpoint->writeValue(config, sizeof(config));
    }

    void setupVolumeControl(BLEClient* client) {
        BLERemoteService* volService = client->getService(BLEUUID(VOLUME_CONTROL_SERVICE_UUID));
        if (volService) {
            volumeControl = volService->getCharacteristic(BLEUUID(VOLUME_CONTROL_POINT_CHAR_UUID));
            if (volumeControl) {
                // Set up volume control notifications
                volumeControl->setNotifyCallback(new VolumeControlCallback());
                volumeControl->enableNotify();
            }
        }
    }
};

// Volume control notification callback
class VolumeControlCallback : public BLECharacteristicCallbacks {
    void onNotify(BLECharacteristic* characteristic) {
        uint8_t* value = characteristic->getValue();
        uint8_t length = characteristic->getLength();
        
        if (length > 0) {
            // Handle volume control updates
            uint8_t volume = value[0];
            // Process volume change
        }
    }
};

class SecurityManager {
private:
    enum class SecurityLevel {
        NONE = 0,
        UNAUTHENTICATED, // Just Works pairing
        AUTHENTICATED,   // MITM protection
        AUTHENTICATED_SC // MITM + Secure Connections
    };

    struct SecurityState {
        bool encrypted;
        bool authenticated;
        bool bonded;
        SecurityLevel level;
        uint8_t keySize;
    };

    std::map<String, SecurityState> deviceSecurityStates;

public:
    SecurityManager() {
        initializeSecurity();
    }

    bool setupDeviceSecurity(BLEClient* client, const String& address) {
        // Set security level to require encryption and MITM protection
        client->setEncryption(address, BLE_GAP_LE_SEC_ENCRYPT_MITM);
        
        // Register for security callbacks
        client->setSecurityCallbacks(new SecurityCallbacks(this));
        
        return true;
    }

    void updateSecurityState(const String& address, const SecurityState& state) {
        deviceSecurityStates[address] = state;
    }

    SecurityState getSecurityState(const String& address) {
        if (deviceSecurityStates.find(address) != deviceSecurityStates.end()) {
            return deviceSecurityStates[address];
        }
        return SecurityState{false, false, false, SecurityLevel::NONE, 0};
    }

private:
    void initializeSecurity() {
        // Configure BLE security settings for RTL8720DN
        BLE.setEncryption(GAP_SECURITY_LEVEL_2);
        
        // Enable MITM protection
        BLE.setAuthenticationParameter(
            GAP_AUTHEN_SECURITY_REQUIRED,
            GAP_AUTHEN_SECURITY_MITM | GAP_AUTHEN_SECURITY_BONDING
        );
        
        // Set IO capabilities
        BLE.setAuthenticationParameter(
            GAP_AUTHEN_IO_CAPABILITIES,
            GAP_IO_CAP_DISPLAY_YES_NO
        );
    }
};

class SecurityCallbacks : public BLESecurityCallbacks {
private:
    SecurityManager* securityManager;
    
public:
    SecurityCallbacks(SecurityManager* manager) : securityManager(manager) {}

    void onAuthenticationComplete(const String& address, uint8_t status) override {
        if (status == GAP_SUCCESS) {
            SecurityManager::SecurityState state;
            state.encrypted = true;
            state.authenticated = true;
            state.level = SecurityManager::SecurityLevel::AUTHENTICATED_SC;
            state.keySize = MAX_KEY_SIZE;
            
            securityManager->updateSecurityState(address, state);
            
            Serial.println("Authentication successful for device: " + address);
        } else {
            Serial.println("Authentication failed for device: " + address);
        }
    }

    void onPasskeyDisplay(const String& address, uint32_t passkey) override {
        char passkeyString[7];
        snprintf(passkeyString, sizeof(passkeyString), "%06d", passkey);
        Serial.println("Please enter this passkey on peer device: " + String(passkeyString));
    }

    bool onPasskeyConfirm(const String& address, uint32_t passkey) override {
        char passkeyString[7];
        snprintf(passkeyString, sizeof(passkeyString), "%06d", passkey);
        Serial.println("Please confirm passkey matches on peer device: " + String(passkeyString));
        
        // In a real implementation, you would wait for user confirmation
        // For this example, we auto-confirm
        return true;
    }

    void onPasskeyInput(const String& address) override {
        // In a real implementation, you would get input from the user
        // For this example, we use a default passkey
        uint32_t passkey = 123456;
        BLE.inputPasskey(address, passkey);
    }

    void onBondingComplete(const String& address, bool success) override {
        if (success) {
            Serial.println("Bonding successful for device: " + address);
            
            // Update security state to include bonding
            SecurityManager::SecurityState state = securityManager->getSecurityState(address);
            state.bonded = true;
            securityManager->updateSecurityState(address, state);
        } else {
            Serial.println("Bonding failed for device: " + address);
        }
    }
};

// Helper class for managing bonded devices
class BondManager {
private:
    static const size_t MAX_BONDED_DEVICES = 10;
    struct BondInfo {
        String address;
        uint8_t keys[MAX_KEY_SIZE];
        bool valid;
    };
    
    std::vector<BondInfo> bondedDevices;

public:
    bool addBondedDevice(const String& address, const uint8_t* keys) {
        if (bondedDevices.size() >= MAX_BONDED_DEVICES) {
            return false;
        }

        BondInfo info;
        info.address = address;
        memcpy(info.keys, keys, MAX_KEY_SIZE);
        info.valid = true;
        
        bondedDevices.push_back(info);
        return true;
    }

    bool removeBondedDevice(const String& address) {
        for (auto it = bondedDevices.begin(); it != bondedDevices.end(); ++it) {
            if (it->address == address) {
                bondedDevices.erase(it);
                return true;
            }
        }
        return false;
    }

    bool isBonded(const String& address) {
        for (const auto& device : bondedDevices) {
            if (device.address == address && device.valid) {
                return true;
            }
        }
        return false;
    }
};

// UART Service implementation
class UARTManager {
private:
    static constexpr size_t UART_BUFFER_SIZE = 512;
    BLERemoteCharacteristic* txChar;
    BLERemoteCharacteristic* rxChar;
    uint8_t txBuffer[UART_BUFFER_SIZE];
    uint8_t rxBuffer[UART_BUFFER_SIZE];
    size_t rxIndex = 0;

public:
    bool setupUARTService(BLEClient* client) {
        BLERemoteService* uartService = client->getService(BLEUUID(UART_SERVICE_UUID));
        if (!uartService) {
            Serial.println("UART service not found");
            return false;
        }

        // Get TX and RX characteristics
        txChar = uartService->getCharacteristic(BLEUUID(UART_TX_CHAR_UUID));
        rxChar = uartService->getCharacteristic(BLEUUID(UART_RX_CHAR_UUID));

        if (!txChar || !rxChar) {
            Serial.println("UART characteristics not found");
            return false;
        }

        // Set up notifications for received data
        rxChar->setNotifyCallback(new UARTCallback(this));
        rxChar->enableNotify();

        return true;
    }

    bool sendData(const uint8_t* data, size_t length) {
        if (!txChar || length > UART_BUFFER_SIZE) {
            return false;
        }
        return txChar->writeValue(data, length);
    }

    void handleReceivedData(const uint8_t* data, size_t length) {
        if (rxIndex + length <= UART_BUFFER_SIZE) {
            memcpy(rxBuffer + rxIndex, data, length);
            rxIndex += length;
            processReceivedData();
        }
    }

private:
    void processReceivedData() {
        // Process received data in rxBuffer
        // This is where you would implement your UART data handling logic
        rxIndex = 0; // Reset buffer after processing
    }
};

// UART notification callback
class UARTCallback : public BLECharacteristicCallbacks {
private:
    UARTManager* uartManager;

public:
    UARTCallback(UARTManager* manager) : uartManager(manager) {}

    void onNotify(BLECharacteristic* characteristic) {
        uint8_t* value = characteristic->getValue();
        uint8_t length = characteristic->getLength();
        
        if (length > 0) {
            uartManager->handleReceivedData(value, length);
        }
    }
};

// Connection management
class ConnectionManager {
private:
    static constexpr uint8_t MAX_RETRY_COUNT = 3;
    static constexpr uint16_t CONNECTION_INTERVAL = 100; // ms
    static constexpr uint16_t SUPERVISION_TIMEOUT = 1000; // ms

    struct ConnectionParams {
        uint16_t minInterval;
        uint16_t maxInterval;
        uint16_t latency;
        uint16_t timeout;
    };

    struct ConnectionStats {
        uint32_t packetsReceived;
        uint32_t packetsSent;
        uint32_t errorCount;
        int8_t lastRssi;
        uint32_t connectionTime;
        uint8_t retryCount;
    };

    std::map<String, ConnectionStats> connectionStats;
    std::map<String, ConnectionParams> connectionParams;

public:
    bool updateConnectionParameters(BLEClient* client, const String& address) {
        ConnectionParams params = {
            .minInterval = 8,  // 10ms (8 * 1.25ms)
            .maxInterval = 16, // 20ms (16 * 1.25ms)
            .latency = 0,
            .timeout = 100     // 1s (100 * 10ms)
        };

        return client->updateConnParams(
            params.minInterval,
            params.maxInterval,
            params.latency,
            params.timeout
        );
    }

    void updateConnectionStats(const String& address, int8_t rssi) {
        auto& stats = connectionStats[address];
        stats.lastRssi = rssi;
        stats.connectionTime = millis();
    }

    bool isConnectionHealthy(const String& address) {
        auto it = connectionStats.find(address);
        if (it == connectionStats.end()) {
            return false;
        }

        // Check RSSI
        if (it->second.lastRssi < -90) {
            return false;
        }

        // Check error rate
        if (it->second.errorCount > 10) {
            return false;
        }

        return true;
    }

    void handleConnectionFailure(const String& address) {
        auto& stats = connectionStats[address];
        stats.errorCount++;
        
        if (stats.errorCount > MAX_RETRY_COUNT) {
            // Remove device from active connections
            connectionStats.erase(address);
            connectionParams.erase(address);
        }
    }
};

// Scan callback implementation
class ScanCallback : public BLEAdvertisedDeviceCallbacks {
private:
    BLECentralManager* centralManager;

public:
    ScanCallback(BLECentralManager* manager) : centralManager(manager) {}

    void onResult(BLEAdvertisedDevice* advertisedDevice) override {
        // Check if device has required services
        if (advertisedDevice->haveServiceUUID()) {
            BLEUUID serviceUUID = advertisedDevice->getServiceUUID();
            if (serviceUUID.equals(BLEUUID(UART_SERVICE_UUID)) ||
                serviceUUID.equals(BLEUUID(AUDIO_SERVICE_UUID))) {
                centralManager->handleNewConnection(advertisedDevice);
            }
        }
    }
};

// Device discovery and management
class DeviceManager {
private:
    std::vector<BLEAdvertisedDevice*> discoveredDevices;
    std::map<String, uint32_t> lastSeenDevices;
    static constexpr uint32_t DEVICE_TIMEOUT = 60000; // 60 seconds

public:
    void addDiscoveredDevice(BLEAdvertisedDevice* device) {
        String address = device->getAddress().toString().c_str();
        lastSeenDevices[address] = millis();
        
        // Update or add device
        bool found = false;
        for (auto& dev : discoveredDevices) {
            if (dev->getAddress().equals(device->getAddress())) {
                // Update existing device
                delete dev;
                dev = new BLEAdvertisedDevice(*device);
                found = true;
                break;
            }
        }
        
        if (!found) {
            discoveredDevices.push_back(new BLEAdvertisedDevice(*device));
        }
    }

    void cleanupOldDevices() {
        uint32_t currentTime = millis();
        auto it = discoveredDevices.begin();
        
        while (it != discoveredDevices.end()) {
            String address = (*it)->getAddress().toString().c_str();
            if (currentTime - lastSeenDevices[address] > DEVICE_TIMEOUT) {
                delete *it;
                it = discoveredDevices.erase(it);
                lastSeenDevices.erase(address);
            } else {
                ++it;
            }
        }
    }

    std::vector<BLEAdvertisedDevice*>& getDiscoveredDevices() {
        return discoveredDevices;
    }

    ~DeviceManager() {
        for (auto device : discoveredDevices) {
            delete device;
        }
    }
};

// Global manager instance
BLECentralManager* centralManager = nullptr;

void setup() {
    Serial.begin(115200);
    while(!Serial) delay(10);
    
    if (!BLE.begin()) {
        Serial.println("Failed to initialize BLE!");
        while (1);
    }
    
    centralManager = new BLECentralManager();
    centralManager->startScanning();
}

void loop() {
    BLE.poll(); // Required for RTL8720DN BLE operations
    delay(100);
}
