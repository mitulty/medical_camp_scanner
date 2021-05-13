#include <BLE2902.h>
#include <Arduino.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLEDevice.h>
#include <ArduinoJson.h>


#define RXD2 16
#define TXD2 17
#define LED 2
#define SERVICE_UUID "78cf3218-1d2c-4a62-85e4-c72633490e0f"

#define POSITION_CHARACTERISTIC_UUID "c3e660d4-c395-4a73-93fc-5392bc42092c"
#define REQUEST_CHARACTERISTIC_UUID "b7172833-07fd-4c42-9199-7b2636108949"

BLECharacteristic *positionCharacteristic;
BLECharacteristic *requestCharacteristic;

bool deviceConnected = false;
char jsonStr[50];
StaticJsonDocument<50> doc_main;


class ServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      Serial.println("Central connected XD");
      deviceConnected = true;
    };
 
    void onDisconnect(BLEServer* pServer) {
      Serial.println("Central dis-connected :(");
      deviceConnected = false;
    }
};

class Request: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* pCharacteristic) {
      std::string value = pCharacteristic->getValue();
      Serial.printf("Received '%s'\n", value.c_str());
      Serial2.write(value.c_str());
      digitalWrite(LED, HIGH);
      delay(1000);
      digitalWrite(LED, LOW);
    }
};

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting BLE work!");
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
  pinMode(LED, OUTPUT);

  BLEDevice::init("esp32-device");
  BLEServer *pServer = BLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  BLEService *positionService = pServer->createService(SERVICE_UUID);

  positionCharacteristic = positionService->createCharacteristic(
                              POSITION_CHARACTERISTIC_UUID,
                              BLECharacteristic::PROPERTY_NOTIFY
                            );
  positionCharacteristic -> addDescriptor(new BLE2902());

  requestCharacteristic = positionService->createCharacteristic(
                              REQUEST_CHARACTERISTIC_UUID,
                              BLECharacteristic::PROPERTY_WRITE
                            );
  BLEDescriptor* descriptor = new BLEDescriptor(BLEUUID((uint16_t) 0x2903));
  descriptor->setValue("Takes Server Request");
  requestCharacteristic -> addDescriptor(descriptor);
  requestCharacteristic -> setCallbacks(new Request());
  requestCharacteristic -> addDescriptor(new BLE2902());
 
  positionService ->start();
 
  positionCharacteristic->setValue("0");
  requestCharacteristic ->setValue("0");
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x12);
  BLEDevice::startAdvertising();
  Serial.println("Characteristic defined! Now you can read it in your phone!");
}


void loop() 
{
  delay(300);
  if (Serial2.available())
  {
    doc_main = Serial2.readString();
    serializeJson(doc_main, jsonStr);
    Serial.printf("Serialized JSON: %s\n", jsonStr);
    positionCharacteristic->setValue(jsonStr);
    positionCharacteristic->notify();
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
    delay(1000);
    digitalWrite(LED, HIGH);
    delay(1000);
    digitalWrite(LED, LOW);
  }
}
