#include <Arduino.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"
#include "heartRate.h"
#include <DHT.h>

// WiFi 和 MQTT 配置
const char* ssid = "sulit";
const char* password = "12345678";
const char* mqttHostUrl = "iot-06z00i9gvlpckb9.mqtt.iothub.aliyuncs.com";
const int mqttPort = 1883;
const char* mqttUser = "esp32s3&k2acmy6ouFy";
const char* mqttPassword = "e0eb00d66e48d33e31c1d1936d09f3eb039e808031fb8080e44b84939ff83de5";
const char* clientId = "k2acmy6ouFy.esp32s3|securemode=2,signmethod=hmacsha256,timestamp=1756826695375|";
// 注 阿里云在连接成功后密钥可与对应时间戳id保持不变 无需调用库计算 详见阿里云提供的动态连接一机一密的那个说明手册  
//若是用tls sha256加密可用bearssl 计算 注意13位时间戳精度

#define PRODUCT_KEY     "k2acmy6ouFy"
#define DEVICE_NAME     "esp32s3"
#define ALINK_TOPIC_PROP_POST "/sys/" PRODUCT_KEY "/" DEVICE_NAME "/thing/event/property/post"

// 传感器对象
MAX30105 particleSensor;

// DHT11 配置
#define DHTPIN 4       
#define DHTTYPE DHT11   
DHT dht(DHTPIN, DHTTYPE);

// 全局变量
float temperature = 0.0;
float humidity = 0.0;
float heartRate = 0.0;
float spo2 = 0.0;

WiFiClient espClient;
PubSubClient client(espClient);

// 算法用缓冲区
#define FreqS 25
#define BUFFER_SIZE (FreqS * 4)

uint32_t irBuffer[BUFFER_SIZE];
uint32_t redBuffer[BUFFER_SIZE];
int bufferIndex = 0;

// 手动添加宏定义
#define MAX30105_SAMPLERATE_400 0x03
#define MAX30105_PULSEWIDTH_411 0x03

// 函数声明
void wifiInit();
void mqttCheckConnect();
void mqttIntervalPost();

void setup() {
  Serial.begin(115200);
  while (!Serial);

  // 初始化 I2C
  Wire.begin();  // ESP32-WROOM 默认使用 GPIO21(SDA), GPIO22(SCL)

  // 初始化 MAX30105
  Serial.println("Initializing MAX30105...");
  if (!particleSensor.begin(Wire)) {
    Serial.println("Failed to initialize MAX30105!");
    while (1);
  }
  Serial.println("MAX30105 initialized");

  // 配置传感器
  particleSensor.setup();
  particleSensor.setPulseAmplitudeRed(0x1F);
  particleSensor.setPulseAmplitudeIR(0x1F);
  particleSensor.setSampleRate(MAX30105_SAMPLERATE_400);
  particleSensor.setPulseWidth(MAX30105_PULSEWIDTH_411);

  // 初始化 DHT11
  Serial.println("Initializing DHT11...");
  dht.begin();
  Serial.println("DHT11 initialized");

  // 连接 WiFi
  wifiInit();

  // 设置 MQTT
  client.setServer(mqttHostUrl, mqttPort);
  client.setKeepAlive(60);
}

void loop() {
  // 读取一个 IR 和 Red 值
  long irValue = particleSensor.getIR();
  long redValue = particleSensor.getRed();

  // 只有当 IR 值有效时才记录
  if (irValue > 50000) {
    if (bufferIndex < BUFFER_SIZE) {
      irBuffer[bufferIndex] = irValue;
      redBuffer[bufferIndex] = redValue;
      bufferIndex++;
    }
  }

  // 当缓冲区满时，调用 Maxim 算法计算
  if (bufferIndex >= BUFFER_SIZE) {
    int32_t spo2Temp;
    int8_t spo2Valid;
    int32_t heartRateTemp;
    int8_t hrValid;

    // 调用 Maxim 官方算法
    // 注意：ESP32-WROOM 使用 uint32_t 缓冲区
    maxim_heart_rate_and_oxygen_saturation(
      irBuffer,
      BUFFER_SIZE,
      redBuffer,
      &spo2Temp,
      &spo2Valid,
      &heartRateTemp,
      &hrValid
    );

    if (hrValid) {
      heartRate = (float)heartRateTemp;
      Serial.print("Heart Rate: ");
      Serial.println(heartRate);
    }
    if (spo2Valid) {
      spo2 = (float)spo2Temp;
      Serial.print("SpO2: ");
      Serial.println(spo2);
    }

    bufferIndex = 0;
    memset(irBuffer, 0, sizeof(irBuffer));
    memset(redBuffer, 0, sizeof(redBuffer));
  }
  if (!client.connected()) {
    mqttCheckConnect();
  }
  client.loop();

  static unsigned long lastMqttPost = 0;
  if (millis() - lastMqttPost >= 5000) {
    lastMqttPost = millis();
    float newTemperature = dht.readTemperature();
    float newHumidity = dht.readHumidity();    
    if (!isnan(newTemperature)) {
      temperature = newTemperature;
    }
    if (!isnan(newHumidity)) {
      humidity = newHumidity;
    }
    
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.print("°C, Humidity: ");
    Serial.print(humidity);
    Serial.println("%");
    mqttIntervalPost();
  }
}

// WiFi 连接
void wifiInit() {
  Serial.print("Connecting to WiFi...");
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("WiFi Connected, IP: ");
  Serial.println(WiFi.localIP());
}

// MQTT 连接
void mqttCheckConnect() {
  if (!client.connected()) {
    Serial.print("MQTT Connecting...");
    if (client.connect(clientId, mqttUser, mqttPassword)) {
      Serial.println(" MQTT Connected!");
    } else {
      Serial.print(" Failed, state=");
      Serial.print(client.state());
      Serial.println(", retrying in 2 seconds");
      delay(2000);
    }
  }
}

// MQTT 上报
void mqttIntervalPost() {
  char param[80];
  char jsonBuf[160];

  int heartRateInt = (int)heartRate;
  int spo2Int = (int)spo2;
  int temperatureInt = (int)(temperature * 10); 
  int humidityInt = (int)humidity;

  sprintf(param, "{\"heartrate\":%d,\"spo2\":%d,\"temperature\":%.1f,\"humidity\":%d}", 
          heartRateInt, spo2Int, temperature, humidityInt);
  sprintf(jsonBuf, "{\"id\":\"123\",\"version\":\"1.0\",\"method\":\"thing.event.property.post\",\"params\":%s}", param);

  Serial.println(jsonBuf);
  if (client.publish(ALINK_TOPIC_PROP_POST, jsonBuf)) {
    Serial.println("MQTT Published");
  } else {
    Serial.println("MQTT Publish Failed!");
  }
}