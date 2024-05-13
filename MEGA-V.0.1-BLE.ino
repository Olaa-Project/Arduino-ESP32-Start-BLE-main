#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "SIM800L.h"
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
/****************************************SIM***************************************/
#define SIM800_RX_PIN 18
#define SIM800_TX_PIN 19
#define SIM800_RST_PIN 5
const char APN[] = "unitel3g";
SIM800L* sim800l;
/****************************************END SIM***************************************/
/****************************************GPS***************************************/
#define RXPin 17
#define TXPin 25
HardwareSerial gpsSerial(2);
TinyGPSPlus gps;
/****************************************END GPS***************************************/
/****************************************BATTERY***************************************/
const float arduinoVCC = 5.01;//Your Arduino voltage
unsigned long ValueR1 = 5075;
unsigned long ValueR2 = 100000;
double Voltage_Source = 12;
const int analogPin = 2;//the pin connecting the voltage.
const int inputResolution =1023;//works with most Arduino boards. 1047
const float average_of = 500;//Average of 500 readings.
float voltage;


/****************************************END BATTERY***************************************/
/****************************************Vehicle***************************************/
const char VehicleSerialNo[] = "vh001";
String deviceToken = "adasdhsdfkj";
/****************************************END Vehicle***************************************/
#define START_PIN 27
#define LOCK_PIN 14
const char URL[] = "mega.olaa.la/api/gps/add";
const char CONTENT_TYPE[] = "application/json";
unsigned long previousMillis = 0;
const long period = 50000;
bool lockStatus = false;
static double CURRENT_LAT, CURRENT_LON, LAST_LAT, LAST_LON, DEGREE, CURRENTBAT;
int countError;

/****************************************BLE***************************************/
BLEServer* pServer = NULL;
BLECharacteristic* pCharacteristic = NULL;
bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;

#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID "beb5483e-36e1-4688-b7f5-ea07361b26a8"

class MyServerCallbacks: public BLEServerCallbacks {
  void onConnect(BLEServer* pServer) {
    deviceConnected = true;
    BLEDevice::startAdvertising();
  };
  void onDisconnect(BLEServer* pServer) {
    deviceConnected = false;
  }
};

class MyCallbacks: public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic *pCharacteristic) {
    std::string rxValue = pCharacteristic->getValue();
    String valueFromUi;
    if (rxValue.length() > 0) {
      for (int i = 0; i < rxValue.length(); i++){
        valueFromUi += rxValue[i];
      }
      Serial.print("Received Value: ");
      Serial.println(valueFromUi);
      if(valueFromUi == (deviceToken + 0)){
        digitalWrite(START_PIN, LOW);
        pCharacteristic->setValue("1");
      }else if(valueFromUi == (deviceToken + 1)){
        digitalWrite(START_PIN, HIGH);
        pCharacteristic->setValue("0");
      }else{
        pCharacteristic->setValue("2");
      }
    }
  }
};
/****************************************END BLE***************************************/
void setup() {
  Serial.begin(115200);
  BLEDevice::init("MEGA-MOTOR");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID,
                      BLECharacteristic::PROPERTY_READ   |
                      BLECharacteristic::PROPERTY_WRITE  |
                      BLECharacteristic::PROPERTY_NOTIFY |
                      BLECharacteristic::PROPERTY_INDICATE
                    );
  pCharacteristic->setCallbacks(new MyCallbacks());
  pCharacteristic->addDescriptor(new BLE2902());
  pService->start();
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->setScanResponse(false);
  pAdvertising->setMinPreferred(0x0);  // set value to 0x00 to not advertise this parameter
  BLEDevice::startAdvertising();

  pinMode(START_PIN, OUTPUT);
  pinMode(LOCK_PIN, OUTPUT);
  digitalWrite(START_PIN, HIGH);
  digitalWrite(LOCK_PIN, HIGH);
  gpsSerial.begin(9600, SERIAL_8N1, RXPin, TXPin);
  while(!Serial);
  SoftwareSerial* serial = new SoftwareSerial(SIM800_RX_PIN, SIM800_TX_PIN);
  serial->begin(9600);
  delay(1000);
  sim800l = new SIM800L((Stream *)serial, SIM800_RST_PIN, 200, 512);
  setupModule();
}
 
void loop() {
  bool newData = false;
  while (gpsSerial.available()){
    if (gps.encode(gpsSerial.read())){
     newData = checkLatLon();
     break;
    }
  }
  readVoltage();
  CURRENTBAT = getVoltageAverage();

  if(newData){
    sendDataToServer(true);
  }
  if(!newData && millis() - previousMillis >= period){
    sendDataToServer(false);
  }
}

bool checkLatLon(){
  CURRENT_LAT = gps.location.lat();
  CURRENT_LON = gps.location.lng();
  double distanceKm = TinyGPSPlus::distanceBetween(CURRENT_LAT, CURRENT_LON, LAST_LAT, LAST_LON);
  if(distanceKm >= 40 && CURRENT_LAT > 10 && CURRENT_LON > 90){
    DEGREE = TinyGPSPlus::courseTo(CURRENT_LAT, CURRENT_LON, LAST_LAT, LAST_LON);
    return true;
  }
  return false;
}

void sendDataToServer(bool type){
  bool connected = false;
  for(uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    connected = sim800l->connectGPRS();
  }
  if(!connected) {
    sim800l->reset();
    setupModule();
    return;
  }

  String DataPost = "";
  DataPost += "{\"sn\": \"";
  DataPost += VehicleSerialNo;
  DataPost += "\", \"lat\": \"";
  DataPost += String(CURRENT_LAT, 6);
  DataPost += "\", \"lng\": \"";
  DataPost += String(CURRENT_LON, 6);
  DataPost += "\", \"degree\": \"";
  DataPost += DEGREE;
  DataPost += "\", \"currentVoltage\": \"";
  DataPost += CURRENTBAT;
  DataPost += "\", \"gps\": \"";
  DataPost += type;
  DataPost += "\", \"lock\": \"";
  DataPost += lockStatus;
  DataPost += "\"}";
  int n = DataPost.length();
  char PAYLOAD[n + 1];
  strcpy(PAYLOAD, DataPost.c_str());
  Serial.println(PAYLOAD);
  uint16_t rc;
  rc = sim800l->doPost(URL, CONTENT_TYPE, PAYLOAD, 10000, 10000);
  previousMillis = millis();
  if(rc == 200) {
    if(type){
      LAST_LAT = CURRENT_LAT;
      LAST_LON = CURRENT_LON;
    }
    countError = 0;
    // String res;
    // res = sim800l->getDataReceived();
    // if(res == "1" && LOCK_PIN == HIGH){
    //   digitalWrite(LOCK_PIN, LOW);
    //   lockStatus = true;
    // }
    // if(res == "0" && LOCK_PIN == LOW){
    //   digitalWrite(LOCK_PIN, HIGH);
    //   lockStatus = false;
    // }

    String res;
    res = sim800l->getDataReceived();
    if(res == "1"){
        digitalWrite(LOCK_PIN, LOW);
        lockStatus = true;
    }

    if(res == "0"){
        digitalWrite(LOCK_PIN, HIGH);
        lockStatus = false;
    }
    
  } else {
    countError++;
    if(countError > 5){
      sim800l->reset();
      setupModule();
      return;       
    }
  }
  bool disconnected = sim800l->disconnectGPRS();
  for(uint8_t i = 0; i < 5 && !connected; i++) {
    delay(1000);
    disconnected = sim800l->disconnectGPRS();
  }
  delay(1000);
}

void setupModule() {
  while(!sim800l->isReady()) {
    delay(1000);
  }
  uint8_t signal = sim800l->getSignal();
  while(signal <= 0) {
    delay(1000);
    signal = sim800l->getSignal();
  }
  delay(1000);
  NetworkRegistration network = sim800l->getRegistrationStatus();
  while(network != REGISTERED_HOME && network != REGISTERED_ROAMING) {
    delay(1000);
    network = sim800l->getRegistrationStatus();
  }
  delay(1000);
  bool success = sim800l->setupGPRS(APN);
  while(!success) {
    success = sim800l->setupGPRS(APN);
    delay(3000);
  }
}

/*************************BATTERY*************************/
void readVoltage(){
  int A0Value = analogRead(analogPin);
  float voltage_sensed = A0Value * (arduinoVCC / (float)inputResolution);    
  voltage = voltage_sensed * ( 1 + ( (float) ValueR2 /  (float) ValueR1) );
}




float getVoltageAverage(){
  float voltage_temp_average=0;
  for(int i=0; i < average_of; i++){
    readVoltage();
    voltage_temp_average +=voltage;
  }
  return voltage_temp_average / average_of;
}

