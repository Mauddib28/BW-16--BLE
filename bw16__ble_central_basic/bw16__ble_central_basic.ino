#include <BLEDevice.h>

// Callback for when a device is discovered
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {
    void onResult(BLEAdvertisedDevice advertisedDevice) {
        Serial.print("Found device: ");
        Serial.println(advertisedDevice.toString().c_str());
        
        // Connect to the device
        BLEDevice::getScan()->stop();
        BLEDevice::createClient()->connect(advertisedDevice);
        
        // Discover services and characteristics
        discoverServices();
    }
};

// Function to discover services and characteristics
void discoverServices() {
    BLEClient* pClient = BLEDevice::createClient();
    Serial.println("Connected to device, discovering services...");

    // Get the services
    pClient->discoverServices();

    // Print services and characteristics
    std::map<std::string, BLERemoteService*> services = pClient->getServices();
    for (auto& service : services) {
        Serial.print("Service UUID: ");
        Serial.println(service.first.c_str());

        // Get characteristics for the service
        std::map<std::string, BLERemoteCharacteristic*> characteristics = service.second->getCharacteristics();
        for (auto& characteristic : characteristics) {
            Serial.print("  Characteristic UUID: ");
            Serial.println(characteristic.first.c_str());

            // Get descriptors for the characteristic
            std::map<std::string, BLERemoteDescriptor*> descriptors = characteristic.second->getDescriptors();
            for (auto& descriptor : descriptors) {
                Serial.print("    Descriptor UUID: ");
                Serial.println(descriptor.first.c_str());
            }
        }
    }
}

// Setup function
void setup() {
    Serial.begin(115200);
    BLEDevice::init("BLE Central Device");
    
    // Start scanning for devices
    BLEScan* pBLEScan = BLEDevice::getScan();
    pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
    pBLEScan->setActiveScan(true);  // Note: This capability does NOT exist
    pBLEScan->startScan(30); // Scan for 30 seconds
}

void loop() {
    // Nothing to do here, scanning is handled in the callback
}
