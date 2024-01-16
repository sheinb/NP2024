/*
  Bluetooth modeled on:
    MyoWare Sensor BLE Peripheral SINGLE SENSOR Example
  SparkFun Electronics
  Pete Lewis
  3/17/2022

  This example reads a single MyoWare Muscle Sensor, and then gets that data from this RedBoard Artemis 
  (the peripheral) to a second RedBoard Artemis (the central) over BLE.

  Note, in BLE, you have services, characteristics and values.
  Read more about it here:

  https://www.arduino.cc/reference/en/libraries/arduinoble/

  Note, before it begins reading the ADC and updating the data,
  It first sets up some BLE stuff:
    1. sets up as a peripheral
    2. sets up a service and characteristic (the data)
        -Note, Services and characteristics have custom 128-bit UUID,
        -These must match the UUIDs in the code on the central device.
    3. advertises itself

  In order for this example to work, you will need a second Artemis, and it will
  need to be programmed with the provided code specific to being a central device, 
  looking for this specific peripheral/service/characteristic.

  Note, both the service and the characteristic get unique UUIDs 
  (even though they are extremely close to being the same thing in this example)

  The second Artemis, aka the "BLE Central," will subscribe to the first board's 
  characteristic, and check to see if the value has been updated. When it has been 
  updated, it will print the value to the serial terminal.

  Hardware:
  MyoWare Sensor with Link Shield snapped on top.
  TRS cable from Link Shield to A0 port of Arduino Shield.
  Arduino Shield pressed into RedBoard Artemis.
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
#include <BLEServer.h>
#include <UMS3.h>
#include <Bounce2.h>
#include <BLE2902.h>

#define SENSOR4
#if defined(SENSOR1)
#define SERVICE_NAME "SENSOR1"
#define SERVICE_UUID "2e2250a0-eeca-43d7-bed9-68aaa0e70383"
#define CHARACTERISTIC_UUID "19b10001-e8f2-537e-4f6c-d104768a1214"
#elif defined(SENSOR2)
#define SERVICE_NAME "SENSOR2"
#define SERVICE_UUID "a622f798-1c6d-11ee-be56-0242ac120002"
#define CHARACTERISTIC_UUID "a622faea-1c6d-11ee-be56-0242ac120002"
#elif defined(SENSOR3)
#define SERVICE_NAME "SENSOR3"
#define SERVICE_UUID "a623001c-1c6d-11ee-be56-0242ac120002"
#define CHARACTERISTIC_UUID "a6230170-1c6d-11ee-be56-0242ac120002"
#elif defined(SENSOR4)
#define SERVICE_NAME "SENSOR4"
#define SERVICE_UUID "a62308a0-1c6d-11ee-be56-0242ac120002"
#define CHARACTERISTIC_UUID "a6230a08-1c6d-11ee-be56-0242ac120002"
#endif

BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool oldDeviceConnected = false;
bool deviceConnected = false;

// Mode selector
Bounce2::Button modeButton = Bounce2::Button();
const int modeButtonPin = 0;

typedef enum { ANALOG_MODE, SINEWAVE_MODE, TOUCH_MODE, N_SENSOR_MODES } sensor_mode_t;
sensor_mode_t sensorMode = ANALOG_MODE;

// UM ESP32-S3 helpers (for Light Sensor, Battery, LED)
UMS3 ums3;
const int ledPin = LED_BUILTIN; // pin to use for the LED

const int analogInput = A9;  // GPIO10
const int touchInput = 1;

float sinefreq = 10;

// set this to baseline touch upon reboot
int touchBaseline;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};



void setup() 
{
  Serial.begin(115200);
  delay(500);
  Serial.println("Sensor BLE Peripheral");

  ums3.begin();
  //ums3.toggleBlueLED();
  ums3.setPixelBrightness(10);
  ums3.setPixelColor(UMS3::colorWheel(sensorMode*60));

  modeButton.attach(modeButtonPin, INPUT_PULLUP);
  modeButton.interval(50);
  modeButton.setPressedState(LOW);

  touchBaseline = touchRead(touchInput);

  BLEDevice::init(SERVICE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID,
                                         BLECharacteristic::PROPERTY_READ |
                                         BLECharacteristic::PROPERTY_WRITE |
                                         BLECharacteristic::PROPERTY_NOTIFY |
                                         BLECharacteristic::PROPERTY_INDICATE
                                       );

  // https://www.bluetooth.com/specifications/gatt/viewer?attributeXmlFile=org.bluetooth.descriptor.gatt.client_characteristic_configuration.xml
  // Create a BLE Descriptor
  pCharacteristic->addDescriptor(new BLE2902());

  pService->start();
  // BLEAdvertising *pAdvertising = pServer->getAdvertising();  // this still is working for backward compatibility
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(true);
  BLEDevice::startAdvertising();
  Serial.println("Waiting a client connection to notify...");
}

void loop() 
{
  static uint16_t val = 0;
  modeButton.update();
  if ( modeButton.pressed() ) {  // Call code if button transitions from HIGH to LOW
    sensorMode = (sensor_mode_t) ((sensorMode+1) % N_SENSOR_MODES); 
  }

  // update neopixel for status
  ums3.setPixelColor(UMS3::colorWheel(sensorMode*60));
  uint8_t brightness = 255*(((float) val/25000.0)+.01);
  ums3.setPixelBrightness(brightness);

  // notify changed value
  if (deviceConnected) {
    switch (sensorMode) {
      case ANALOG_MODE:
      {
        val = analogRead(analogInput); // Read the sensor attached to Analog Pin A0
      }
      break;
      case SINEWAVE_MODE:
      {
        val = (sin(sinefreq*(millis()/1000.)*M_PI*2)+1)*2048;
      }
      break;
      case TOUCH_MODE:
      {
        touch_value_t raw_val = touchRead(touchInput);
        val = constrain(map(raw_val, touchBaseline, 150000, 0, 4096), 0, 4096);
      }
      break;
    }
    pCharacteristic->setValue((uint8_t *) &val, 2);
    pCharacteristic->notify();

    //Serial.println(val);
    delay(2);
  }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
  }

  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      // do stuff here on connecting
      oldDeviceConnected = deviceConnected;
  }
}
