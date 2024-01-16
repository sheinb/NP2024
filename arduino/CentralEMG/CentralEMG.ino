/*
  MyoWare Receiver BLE Central SINGLE SENSOR Example
  SparkFun Electronics
  Pete Lewis
  3/17/2022

  This example sets up a SparkFun RedBoard Artemis as a BLE central device,
  Then, it connects to a second Artemis peripheral device that is reading a single MyoWare
  Muscle sensor. It then streams the data on the Serial Terminal.

  Note, in BLE, you have services, characteristics and values.
  Read more about it here:

  https://www.arduino.cc/reference/en/libraries/arduinoble/

  Note, before it begins checking the data and printing it,
  It first sets up some BLE stuff:
    1. sets up as a central
    2. scans for any peripherals
    3. Connects to the device named "MYOWARE1"
    4. Subscribes MYOWARE1's data characteristic

  In order for this example to work, you will need a second Artemis, and it will
  need to be programmed with the provided code specific to being a peripheral device, 
  and advertising as MYOWARE1 with the specific characteristic UUID.

  Note, both the service and the characteristic get unique UUIDs 
  (even though they are extremely close to being the same thing in this example)

  This Artemis, aka the "BLE Central," will subscribe to the peripheral board's 
  characteristic, and check to see if the value has been updated. When it has been 
  updated, it will print the value to the serial terminal.

  Hardware:
  SparkFun RedBoard Artemis
  USB from Artemis to Computer.

  ** For consistent BT connection follow these steps:
  ** 1. Reset Peripheral
  ** 2. Wait 5 seconds
  ** 3. Reset Central
  ** 4. Enjoy BT connection
  **
  ** ArduinoBLE does not support RE-connecting two devices.
  ** If you loose connection, you must follow this hardware reset sequence again.

  This example code is in the public domain.
*/

#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

#include <UMS3.h>
#include <Wire.h>
#include <Bounce2.h>

// UM ESP32-S3 helpers (for Light Sensor, Battery, LED)
UMS3 ums3;

#define I2C_DEV_ADDR 0x08

// use this to decide which peripheral to search for
const int selectorSwitchPin = 5;
Bounce2::Button selectorSwitch = Bounce2::Button();

int sensorLevel = 0;
void onSensorRequest() {
  uint16_t sensor_value = sensorLevel;
  //  uint16_t sensor_value = analogRead(A6);
  Wire.write((const uint8_t*)&sensor_value, 2);
}

#define SETUP2

#if defined(SETUP1)
static BLEUUID sensorServiceUUID("2e2250a0-eeca-43d7-bed9-68aaa0e70383");
static BLEUUID sensorCharacteristicUUID("19b10001-e8f2-537e-4f6c-d104768a1214");

static BLEUUID openMVServiceUUID("3a9e17c6-1639-11ee-be56-0242ac120002");
static BLEUUID openMVCharacteristicUUID("3a9e1af0-1639-11ee-be56-0242ac120002");
#elif defined(SETUP2)
static BLEUUID sensorServiceUUID("a622f798-1c6d-11ee-be56-0242ac120002");
static BLEUUID sensorCharacteristicUUID("a622faea-1c6d-11ee-be56-0242ac120002");

static BLEUUID openMVServiceUUID("a622fc66-1c6d-11ee-be56-0242ac120002");
static BLEUUID openMVCharacteristicUUID("a622fdd8-1c6d-11ee-be56-0242ac120002");
#elif defined(SETUP3)
static BLEUUID sensorServiceUUID("a623001c-1c6d-11ee-be56-0242ac120002");
static BLEUUID sensorCharacteristicUUID("a6230170-1c6d-11ee-be56-0242ac120002");

static BLEUUID openMVServiceUUID("a6230314-1c6d-11ee-be56-0242ac120002");
static BLEUUID openMVCharacteristicUUID("a62306fc-1c6d-11ee-be56-0242ac120002");
#elif defined(SETUP4)
static BLEUUID sensorServiceUUID("a62308a0-1c6d-11ee-be56-0242ac120002");
static BLEUUID sensorCharacteristicUUID("a6230a08-1c6d-11ee-be56-0242ac120002");

static BLEUUID openMVServiceUUID("a6230ba2-1c6d-11ee-be56-0242ac120002");
static BLEUUID openMVCharacteristicUUID("a6230d1e-1c6d-11ee-be56-0242ac120002");
#endif
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* myDevice;

/* Keep track of which peripheral we are listening to (based on switch) */
typedef enum { PERIPHERAL_NOT_CONNECTED,
               PERIPHERAL_SENSOR,
               PERIPHERAL_OPENMV } peripheral_t;
peripheral_t currentPeripheral = PERIPHERAL_SENSOR;
peripheral_t connectedPeripheral = PERIPHERAL_NOT_CONNECTED;
void initiateScan(peripheral_t peripheral);

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
  static int count = 0;
  if (length == 2) {
    sensorLevel = *((uint16_t*)pData);
    ums3.setPixelColor(UMS3::colorWheel(count++ * 2));
  }
  Serial.println(sensorLevel);
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    initiateScan(currentPeripheral);
    Serial.println("onDisconnect, attempting reconnect");
  }
};

bool connectToServer(BLEUUID serviceUUID, BLEUUID charUUID) {
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient* pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  // Connect to the remove BLE Server.
  pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
  Serial.println(" - Connected to server");
  //pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)

  // Obtain a reference to the service we are after in the remote BLE server.
  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");


  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  // Read the value of the characteristic.
  if (pRemoteCharacteristic->canRead()) {
    uint16_t value = pRemoteCharacteristic->readUInt16();
    Serial.print("The characteristic value was: ");
    Serial.println(value);
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  connected = true;
  return true;
}

/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks {

  BLEUUID serviceUUID;

public:
  MyAdvertisedDeviceCallbacks(BLEUUID id) {
    serviceUUID = id;
  }

private:
  /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    //    Serial.print("BLE Advertised Device found: ");
    //    Serial.println(advertisedDevice.toString().c_str());
    // We have found a device, let us now see if it contains the service we are looking for.
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    }  // Found our server
  }    // onResult
};     // MyAdvertisedDeviceCallbacks

void initiateScan(peripheral_t peripheral) 
{
  BLEUUID currentServiceUUID;
  BLEScan* pBLEScan;

  currentPeripheral = peripheral;
  switch (peripheral) {
    case PERIPHERAL_SENSOR:
      currentServiceUUID = sensorServiceUUID;
      break;
    case PERIPHERAL_OPENMV:
      currentServiceUUID = openMVServiceUUID;
      break;
  }
  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  pBLEScan = BLEDevice::getScan();
  pBLEScan->stop();

  if (connected) {
    Serial.println("closing connection to peripheral");
    BLEDevice::deinit();
    connected = false;
  }

  BLEDevice::init("");
  pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks(currentServiceUUID));
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}

void setPeripheral(void)
{
    Serial.println("Searching for Sensor Peripheral");
    currentPeripheral = PERIPHERAL_SENSOR;
 //   Serial.println("Searching for OpenMV Peripheral");
 //   currentPeripheral = PERIPHERAL_OPENMV;
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("BLE Central Control");

  ums3.begin();
  //ums3.toggleBlueLED();
  ums3.setPixelBrightness(20);
  ums3.setPixelColor(UMS3::colorWheel(10));

  BLEUUID currentServiceUUID;

  setPeripheral();
  initiateScan(currentPeripheral);
}


void loop() {
  BLEUUID serviceUUID, characteristicUUID;

  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    switch (currentPeripheral) {
      case PERIPHERAL_SENSOR:
        serviceUUID = sensorServiceUUID;
        characteristicUUID = sensorCharacteristicUUID;
        break;
      case PERIPHERAL_OPENMV:
        serviceUUID = openMVServiceUUID;
        characteristicUUID = openMVCharacteristicUUID;
        break;
      default:
        return;
    }

    if (connectToServer(serviceUUID, characteristicUUID)) {
      Serial.println("Connected...");
      connectedPeripheral = currentPeripheral;
    } else {
      Serial.println("Failed to connect...");
      connectedPeripheral = PERIPHERAL_NOT_CONNECTED;
    }
    doConnect = false;
  }

  // If we are connected to a peer BLE Server, update the characteristic each time we are reached
  // with the current time since boot.
  if (connected) {

  } else if (doScan) {
    
  }

  delay(1000);  // Delay a second between loops.
}  // End of loop
