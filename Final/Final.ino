
#include <BLEDevice.h>
#include <BLEUtils.h>
#include <BLEServer.h>
#include <BLE2902.h>
#include "MPU6050_tockn.h"
#include <Wire.h>

BLECharacteristic *pCharacteristic;
bool deviceConnected = false;
uint8_t txValue = 0;
std::string rxValue;
#define SERVICE_UUID        "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHARACTERISTIC_UUID_RX "582adc94-2eab-4735-b821-293883e26762"
#define CHARACTERISTIC_UUID_TX "0da69903-6d69-4810-bccc-4c56d2880188"

MPU6050 mpu6050(Wire);

const int endstop1_Pin = 27; 
const int endstop2_Pin = 15;
const int motor_Pin1 = 18;
const int motor_Pin2 = 19;
const int Led1_4_Pin = 17; 
const int Led2_5_Pin = 4;
const int Led3_6_Pin = 16;
bool button1_state = false;
bool button2_state = false;
bool button3_state = false;
bool led_state = true;
unsigned long timePressButton3 = 0;
unsigned long timePressButton2 = 0;
unsigned long timePressButton1 = 0;
#define STOP 0
#define RUN_FORWARD 1
#define RUN_BACKWARD 2
int motorState = STOP;
bool accelCheck = false;
long timeAccel = 0;
float accelAverage;

/* BLE */
class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};

class MyCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
      rxValue = pCharacteristic->getValue();
      if (rxValue.length() == 1) {

        byte data0 = (uint8_t)rxValue[0];
        Serial.println(data0);
      }

    }

};
class MySecurity : public BLESecurityCallbacks {

    bool onConfirmPIN(uint32_t pin) {
      return false;
    }

    uint32_t onPassKeyRequest() {
      ESP_LOGI(LOG_TAG, "PassKeyRequest");
      return 199720;
    }

    void onPassKeyNotify(uint32_t pass_key) {
      ESP_LOGI(LOG_TAG, "On passkey Notify number:%d", pass_key);
    }

    bool onSecurityRequest() {
      ESP_LOGI(LOG_TAG, "On Security Request");
      return true;
    }

    void onAuthenticationComplete(esp_ble_auth_cmpl_t cmpl) {
      ESP_LOGI(LOG_TAG, "Starting BLE work!");
      if (cmpl.success) {
        uint16_t length;
        esp_ble_gap_get_whitelist_size(&length);
        ESP_LOGD(LOG_TAG, "size: %d", length);
      }
    }
};
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting BLE work!");

  BLEDevice::init("ESP32");
  BLEDevice::setEncryptionLevel(ESP_BLE_SEC_ENCRYPT);
  BLEDevice::setSecurityCallbacks(new MySecurity());
  BLEServer *pServer = BLEDevice::createServer();
  BLEService *pService = pServer->createService(SERVICE_UUID);
  pCharacteristic = pService->createCharacteristic(
                      CHARACTERISTIC_UUID_TX,
                      BLECharacteristic::PROPERTY_NOTIFY
                    );

  pCharacteristic->addDescriptor(new BLE2902());

  BLECharacteristic *pCharacteristic = pService->createCharacteristic(
                                         CHARACTERISTIC_UUID_RX,
                                         BLECharacteristic::PROPERTY_WRITE
                                         | BLECharacteristic::PROPERTY_WRITE_NR
                                       );
  pCharacteristic->setCallbacks(new MyCallbacks());

  pCharacteristic->setValue("Hello World says Neil");
  pService->start();
  BLEAdvertising *pAdvertising = pServer->getAdvertising();
  pAdvertising->start();
  BLESecurity *pSecurity = new BLESecurity();
  pSecurity->setAuthenticationMode(ESP_LE_AUTH_REQ_SC_ONLY);
  pSecurity->setCapability(ESP_IO_CAP_OUT);
  pSecurity->setInitEncryptionKey(ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK);

  esp_ble_auth_req_t auth_req = ESP_LE_AUTH_REQ_SC_MITM_BOND;     //bonding with peer device after authentication
  esp_ble_io_cap_t iocap = ESP_IO_CAP_OUT;           //set the IO capability to No output No input
  uint8_t key_size = 16;      //the key size should be 7~16 bytes
  uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
  //set static passkey
  uint32_t passkey = 199720;
  uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;
  uint8_t oob_support = ESP_BLE_OOB_DISABLE;
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_STATIC_PASSKEY, &passkey, sizeof(uint32_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
  esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

  pinMode(endstop1_Pin, INPUT);
  pinMode(endstop2_Pin, INPUT);
  pinMode(motor_Pin1, OUTPUT);
  pinMode(motor_Pin2, OUTPUT);
  pinMode(Led1_4_Pin, OUTPUT);
  pinMode(Led2_5_Pin, OUTPUT);
  pinMode(Led3_6_Pin, OUTPUT);
  Wire.begin();
  mpu6050.begin();
  mpu6050.calcGyroOffsets(true);
  Serial.println("START" );
}
void loop()
{
  // Serial.print("button1_state: ");
  // Serial.println(button1_state);
  // Serial.print("button2_state: ");
  // Serial.println(button2_state);
  // Serial.print("button3_state: ");
  // Serial.println(button3_state);
  accelAverage = calAccelerator(50);
  Serial.println(accelAverage);
  if (accelAverage > 1.5 && !accelCheck) {
    accelCheck = true;
  }
  if (accelCheck)
  {
    if (accelAverage < 1.5) {
      turnOnLed(Led1_4_Pin);
      turnOnLed(Led2_5_Pin);
      turnOnLed(Led3_6_Pin);
      delay(2000);
      turnOffLed(Led1_4_Pin);
      turnOffLed(Led2_5_Pin);
      turnOffLed(Led3_6_Pin);
      accelCheck = false;
    }
  }

  readBLE();
  if (button2_state)
  {
    if (motorState == STOP)
    {
      Serial.println("RUN FORWARD" );
      motor_FORWARD();
      pCharacteristic->setValue("2");
      pCharacteristic->notify();
    }
    else if (motorState == RUN_FORWARD)
    {
      if (readStatusEndstop(endstop1_Pin) == HIGH)
      {
        Serial.println("RUN BACKWARD" );
        motor_BACKWARD();
        pCharacteristic->setValue("3");
        pCharacteristic->notify();
      }
    }
    else if (motorState == RUN_BACKWARD)
    {
      if (readStatusEndstop(endstop2_Pin) == HIGH)
      {
        Serial.println("Stop" );
        motor_Stop();
        pCharacteristic->setValue("4");
        pCharacteristic->notify();
        button2_state = false;
      }
    }
  }
  if (button3_state)
  {
    if ((unsigned long)(millis() - timePressButton3) < 200)
    {
      turnOnLed(Led1_4_Pin);
      turnOnLed(Led2_5_Pin);
      turnOnLed(Led3_6_Pin);
    }
    else if ((unsigned long)(millis() - timePressButton3) < 400)
    {
      turnOffLed(Led1_4_Pin);
      turnOffLed(Led2_5_Pin);
      turnOffLed(Led3_6_Pin);
    }
    else
    {
      timePressButton3 = millis();
    }
  }
  if (button1_state)
  {
    if ((unsigned long)(millis() - timePressButton1) < (150 * 1))
    {
      turnOnLed(Led1_4_Pin);
    }
    else if ((unsigned long)(millis() - timePressButton1) < (150 * 2))
    {
      turnOffLed(Led1_4_Pin);
    }
    else if ((unsigned long)(millis() - timePressButton1) < (150 * 3))
    {
      turnOnLed(Led2_5_Pin);
    }
    else if ((unsigned long)(millis() - timePressButton1) < (150 * 4))
    {
      turnOffLed(Led2_5_Pin);
    }
    else if ((unsigned long)(millis() - timePressButton1) < (150 * 5))
    {
      turnOnLed(Led3_6_Pin);
    }
    else if ((unsigned long)(millis() - timePressButton1) < (150 * 6))
    {
      turnOffLed(Led3_6_Pin);
    }
    else
    {
      timePressButton1 = millis();
    }
  }
  if ((button1_state || button3_state) && led_state )
  {
    pCharacteristic->setValue("1");
    pCharacteristic->notify();
    led_state = false;
  }
  else if (!(button1_state || button3_state)) {
    pCharacteristic->setValue("0");
    pCharacteristic->notify();
    led_state = true;
  }
}
void motor_FORWARD()
{
  motorState = RUN_FORWARD;
  digitalWrite(motor_Pin1, HIGH);
  digitalWrite(motor_Pin2, LOW);
}
void motor_BACKWARD()
{
  motorState = RUN_BACKWARD;
  digitalWrite(motor_Pin1, LOW);
  digitalWrite(motor_Pin2, HIGH);
}
void motor_Stop()
{
  motorState = STOP;
  digitalWrite(motor_Pin1, LOW);
  digitalWrite(motor_Pin2, LOW);
}
int readStatusEndstop(int pin)
{
  delay(50);
  return digitalRead(pin);
}
void turnOnLed(int pin)
{
  digitalWrite(pin, HIGH);
}
void turnOffLed(int pin)
{
  digitalWrite(pin, LOW);
}
int readBLE() {
  if ((uint8_t)rxValue[0] == 49) // ON/OFF ROW
  {
    //while(digitalRead(RF_D0) == HIGH){};
    button1_state = !(button1_state);
    timePressButton1 = millis();
    {
      turnOffLed(Led1_4_Pin);
      turnOffLed(Led2_5_Pin);
      turnOffLed(Led3_6_Pin);
    }
    button3_state = false;
    rxValue = -1;
    return 0;
  }
  if ((uint8_t)rxValue[0] == 51) // Motor
  {
    button2_state = !(button2_state);
    timePressButton2 = millis();
    rxValue = -1;
    return 2;
  }
  if ((uint8_t)rxValue[0] == 50) // ON/OFF ALL
  {
    Serial.println("////////////");
    button3_state = !(button3_state);
    {
      turnOffLed(Led1_4_Pin);
      turnOffLed(Led2_5_Pin);
      turnOffLed(Led3_6_Pin);
    }
    timePressButton3 = millis();
    button1_state = false;
    rxValue = -1;
    return 3;
  }
}

float calAccelerator(int N) {
  float accelTotal[100], vAverage = 0;
  unsigned long timeCheck = millis();

  for (int i = 0; i < N; i++) {
    mpu6050.update();
    vAverage += sqrt(pow(mpu6050.getAccX(), 2) + pow(mpu6050.getAccY(), 2) + pow(mpu6050.getAccZ(), 2));
  }
  vAverage =  vAverage / N;

  Serial.print("Times: ");
  Serial.println(millis() - timeCheck);
  return vAverage;
}
























