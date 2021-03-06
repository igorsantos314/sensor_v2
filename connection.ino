#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

unsigned long myChannelNumber = 1110526;
const char * myWriteAPIKey = "2JT0XT1NLS0GOJ4W";

#define WIFISSID01 "extensao_iot" // Put your WifiSSID here
#define PASSWORD01 "aluno123" // Put your wifi password here

//#define WIFISSID01 "POCO X3" // Put your WifiSSID here
//#define PASSWORD01 "12345678"

//#define WIFISSID02 "IFPE-PUBLICA" // Put your WifiSSID here
//#define PASSWORD02 "aluno123" // Put your wifi password here

//#define WIFISSID01 "GVT-F051" // Put your WifiSSID here
//#define PASSWORD01 "1316309065" // Put your wifi password here

const char* servidorMqtt = "eltontorres.asuscomm.com";
uint16_t portaServidorMqtt = 1883;

const char* tokenMqttDisp = "ESP32_UMIDADE_SOLO";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

#define ATTRIBUTES_TOPIC "v1/devices/me/attributes"
#define ATTRIBUTES_REQUEST_TOPIC "v1/devices/me/attributes/request/1"
#define ATTRIBUTES_RESPONSE_TOPIC "v1/devices/me/attributes/response/1"

//CLASSE PARA CONEXAO WIFI E MQTT
class connection{
  void setupWifiClient() {
    Serial.println("Starting wifi...");
    WiFi.mode(WIFI_STA);
    WiFi.begin(WIFISSID01, PASSWORD01);
    connectWifi();  
    client.setServer(servidorMqtt , portaServidorMqtt);
    client.setCallback(on_message);
    imprimirInformacoesWifiEMQTT();
  }
  
  boolean isWifiConnected() {
    return WiFi.status() == WL_CONNECTED;
  }
  
  bool connectWifi() {
    Serial.println("Connecting Wifi...");  
    while (!isWifiConnected()) {
      delay(500);
      Serial.print(".");
    }
    Serial.println("");
    Serial.println("WiFi connected");
    return true;
  }
  
  bool reconnectMQTT(PubSubClient *clientMQTT) {
    bool isMQTTConnected = clientMQTT->connected();
    if (!isMQTTConnected) {
      Serial.println("Connecting to ThingsBoard node ...");
      // Attempt to connect (clientId, username, password)
      // Create a random client ID
      String clientId = "ESP32Client-";
      clientId += String(random(0xffff), HEX);
      isMQTTConnected = clientMQTT->connect(clientId.c_str(), tokenMqttDisp, NULL);
      if (isMQTTConnected) {
        Serial.println("Subscribing to ThingsBoard node ...");
        clientMQTT->subscribe(ATTRIBUTES_TOPIC);
        clientMQTT->publish(ATTRIBUTES_REQUEST_TOPIC, "{'sharedKeys': 'measureTimeInterval'}");
        clientMQTT->publish(ATTRIBUTES_REQUEST_TOPIC, "{'sharedKeys': 'qttOfMeasures'}");
        Serial.println("End Subscribing to ThingsBoard node ...");
      }
    }
    return isMQTTConnected;
  }
  
  void printMessage(const char* topic, JsonDocument& doc){
    char buffer[256];  
    serializeJsonPretty(doc, buffer);
    Serial.print("Topic: ");
    Serial.println(topic);
    Serial.print("Message: ");
    Serial.println(buffer);
    Serial.println("//--------------------------------------------------------------//");
  }
  
  void on_message(const char* topic, byte* payload, unsigned int length) {
    StaticJsonDocument<256> doc;
    deserializeJson(doc, payload, length);
    //printMessage(topic, doc);
  
    if (strcmp(topic, ATTRIBUTES_TOPIC) == 0) {
      if (doc.containsKey("measureTimeInterval")) {
        measureTimeInterval = doc["measureTimeInterval"];
      }
      if (doc.containsKey("qttOfMeasures")) {
        qttOfMeasures = doc["qttOfMeasures"];
      }
      if (doc.containsKey("turn_on")) {
        pinRele = doc["turn_on"];
        digitalWrite(5, pinRele);
      }
    } else if (strcmp(topic, ATTRIBUTES_RESPONSE_TOPIC) == 0) {
      if (doc.containsKey("shared") && doc["shared"].containsKey("measureTimeInterval")) {
        measureTimeInterval = doc["shared"]["measureTimeInterval"];
      }
      if (doc.containsKey("shared") && doc["shared"].containsKey("qttOfMeasures")) {
        qttOfMeasures = doc["shared"]["qttOfMeasures"];
      }
      if (doc.containsKey("shared") && doc["shared"].containsKey("turn_on")) {
        pinRele = doc["shared"]["turn_on"];
        digitalWrite(5, pinRele);
      }
       
    } else {
      Serial.println("T??pico n??o tratado!");
    }
  }
  
  void sendSensorInfoToMQTTServer(Sensor sensor, PubSubClient *clientMQTT) {
    bool isConnected = reconnectMQTT(clientMQTT);
    if (isConnected) {
      values measure = sensor.lastMeasure;
      String supplyVoltageRawATT = String("supplyVoltageRaw");
      String supplyVoltageATT = String("supplyVoltage");
      String sensorVoltageRawATT = String("sensorVoltageRaw");
      String sensorVoltageATT = String("sensorVoltage");
      String resistanceATT = String("resistance");
      String weightATT = String("weight");
  
      supplyVoltageRawATT.concat(sensor.id);
      supplyVoltageATT.concat(sensor.id);
      sensorVoltageRawATT.concat(sensor.id);
      sensorVoltageATT.concat(sensor.id);
      resistanceATT.concat(sensor.id);
      weightATT.concat(sensor.id);
      char weightString[20];
      sprintf(weightString, "%.5f", sensor.weight);
  
      enviarInfoParaServidorMQTT(clientMQTT, supplyVoltageRawATT, String(measure.supplyVoltageRaw));
      enviarInfoParaServidorMQTT(clientMQTT, supplyVoltageATT, String(measure.supplyVoltage));
      enviarInfoParaServidorMQTT(clientMQTT, sensorVoltageRawATT, String(measure.sensorVoltageRaw));
      enviarInfoParaServidorMQTT(clientMQTT, sensorVoltageATT, String(measure.sensorVoltage));
      enviarInfoParaServidorMQTT(clientMQTT, resistanceATT, String(measure.resistance));
      enviarInfoParaServidorMQTT(clientMQTT, weightATT, weightString);
    } else {
      Serial.println("N??o foi possivel conectar com o servidor MQTT!");
    }
  }
  
  void enviarInfoParaServidorMQTT(PubSubClient *clientMQTT, String atributo, String valor) {
    // Prepare a JSON payload string
    char buffer[256];
    StaticJsonDocument<256> doc;
    doc[atributo] = valor;
    size_t n = serializeJson(doc, buffer);
    boolean sent = clientMQTT->publish("v1/devices/me/telemetry", buffer, n);
    //if (sent) {
    //  Serial.println("Mensagem enviada com sucesso!");
    //} else {
    //  Serial.println("Mensagem n??o enviada!");
    //}
  }
  
  void imprimirInformacoesWifiEMQTT() {
    // Imprimir os valores-padr??o das vari??veis referentes ao WiFi e servidor MQTT
    Serial.println(tokenMqttDisp);
    Serial.println(servidorMqtt);
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
    Serial.print("MAC: ");
    Serial.println(WiFi.macAddress());
    Serial.print("Wifi status: ");
    Serial.println(WiFi.status());
    Serial.print("Wifi SSID: ");
    Serial.println(WiFi.SSID());
    Serial.print("GW: ");
    Serial.println(WiFi.gatewayIP());
    Serial.print("Mask: ");
    Serial.println(WiFi.subnetMask());
  }
  
  void printArray(String msg, float* myArray, int size) {
    Serial.println(msg);
    Serial.print("[");
    for (int i = 0; i < size - 1; i++) {
      Serial.print(myArray[i] , 5);
      Serial.print(" , ");
    }
    Serial.print(myArray[size - 1] , 5);
    Serial.println("]");
  }
};
