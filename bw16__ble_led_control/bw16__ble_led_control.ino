/*

 Example guide:
 https://www.amebaiot.com/en/amebad-arduino-ble-pwm/

 Expanded upon the above example to create a BLE device that controls a set of LEDs (e.g. single diodes, spectrum, strip)
 	- Input:	[ R, G, B ] values formatted in that triplet which provide the Pulse-Width Modulation (PWM) for each LED
	- Output:	Pulse-Width Modulation (PWM) control of the LEDs via voltage controls (e.g. direct, voltage stepper)
  - Note: Commands to Write LED brightness (PWM) is "!C<R><G><B>" with R, G, B as a byte value (i.e. 0x00 - 0xFF)

 Last Edit:		2024/12/05
 Done By:		Paul A. Wortman
 */

#include "BLEDevice.h"

#define UART_SERVICE_UUID      "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_RX "6E400002-B5A3-F393-E0A9-E50E24DCCA9E"
#define CHARACTERISTIC_UUID_TX "6E400003-B5A3-F393-E0A9-E50E24DCCA9E"

#define STRING_BUF_SIZE 100

BLEService UartService(UART_SERVICE_UUID);
BLECharacteristic Rx(CHARACTERISTIC_UUID_RX);
BLECharacteristic Tx(CHARACTERISTIC_UUID_TX);
BLEAdvertData advdata;
BLEAdvertData scndata;
bool notify = false;

void readCB (BLECharacteristic* chr, uint8_t connID) {
    Serial.print("Characteristic ");
    //Serial.print(chr->getUUID().str());   // Failing when read occurs????
    Serial.print(" read by connection ");
    Serial.println(connID);
}

void writeCB (BLECharacteristic* chr, uint8_t connID) {
    Serial.print("Characteristic ");
    //Serial.print(chr->getUUID().str());   // Failing here <---- Causes BLE device to freeze
    Serial.print(" write by connection ");
    Serial.println(connID);
    uint16_t datalen = chr->getDataLen();
    if (datalen > 0) {
        if (chr->readData8() == '!') {
            uint8_t command[datalen];
            chr->getData(command, datalen);
            if (command[1] == 'C') {    // Note: The parsing will take the hex value of the ASCii character (e.g. 0 = 0x48)
                Serial.print("Color command R = ");
                Serial.print(command[2]);
                Serial.print(" G = ");
                Serial.print(command[3]);
                Serial.print(" B = ");
                Serial.println(command[4]);
                // print hex
                //printf("Color command R = %x G = %x B = %x \n", command[2], command[3], command[4]);
                // print decimal
                //printf("Color command R = %d G = %d B = %d \n", command[2], command[3], command[4]);
                analogWrite(LED_R, command[2]);
                analogWrite(PA30, command[3]);
                analogWrite(LED_B, command[4]);                
            }
        } else {
            Serial.print("Received string: ");
            Serial.print(chr->readString());
            Serial.println();
        }
    }
}

void notifCB(BLECharacteristic* chr, uint8_t connID, uint16_t cccd) {
    if (cccd & GATT_CLIENT_CHAR_CONFIG_NOTIFY) {
        //printf("Notifications enabled on Characteristic %s for connection %d \n", chr->getUUID().str(), connID);
        Serial.print("Notifications enabled on Characteristic");
        notify = true;
    } else {
        //printf("Notifications disabled on Characteristic %s for connection %d \n", chr->getUUID().str(), connID);
        Serial.print("Notifications disabled on Characteristic");
        notify = false;
    }
    Serial.print(chr->getUUID().str());
    Serial.print(" for connection");
    Serial.println(connID);
}

// Note: The Serial Prints have been commented out because they were preventing the Device/BLe from initializing?
void setup() {
    //Serial.println("[*] Setting-up BLE LED Driver");
    Serial.begin(115200);
    //Serial.println("[*] Starting Main Loop Function");

    pinMode(LED_R, OUTPUT);
    pinMode(PA30, OUTPUT);    // PA30
    pinMode(LED_B, OUTPUT);

    analogWrite(LED_R, 255);
    analogWrite(PA30, 255);
    analogWrite(LED_B, 255);

    advdata.addFlags(GAP_ADTYPE_FLAGS_LIMITED | GAP_ADTYPE_FLAGS_BREDR_NOT_SUPPORTED);
    advdata.addCompleteName("AMEBA_BLE_DEV");
    scndata.addCompleteServices(BLEUUID(UART_SERVICE_UUID));

    Rx.setWriteProperty(true);
    Rx.setWritePermissions(GATT_PERM_WRITE);
    Rx.setWriteCallback(writeCB);
    Rx.setBufferLen(STRING_BUF_SIZE);
    Tx.setReadProperty(true);
    Tx.setReadPermissions(GATT_PERM_READ);
    Tx.setReadCallback(readCB);
    Tx.setNotifyProperty(true);
    Tx.setCCCDCallback(notifCB);
    Tx.setBufferLen(STRING_BUF_SIZE);

    UartService.addCharacteristic(Rx);
    UartService.addCharacteristic(Tx);

    BLE.init();
    BLE.configAdvert()->setAdvData(advdata);
    BLE.configAdvert()->setScanRspData(scndata);
    BLE.configServer(1);
    BLE.addService(UartService);

    BLE.beginPeripheral();
}

void loop() {
    //Serial.print(".");
    if (Serial.available()) {
        Tx.writeString(Serial.readString());
        if (BLE.connected(0) && notify) {
            Tx.notify(0);
        }
    }
    delay(100);		// Delay Time
}
