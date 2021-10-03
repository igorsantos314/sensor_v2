#include "classes.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define ATTRIBUTES_TOPIC "v1/devices/me/attributes"
#define ATTRIBUTES_REQUEST_TOPIC "v1/devices/me/attributes/request/1"
#define ATTRIBUTES_RESPONSE_TOPIC "v1/devices/me/attributes/response/1"

unsigned long myChannelNumber = 1110526;
const char* myWriteAPIKey = "2JT0XT1NLS0GOJ4W";

#define WIFISSID01 "extensao_iot"  // Put your WifiSSID here
#define PASSWORD01 "aluno123"      // Put your wifi password here

//#define WIFISSID01 "POCO X3" // Put your WifiSSID here
//#define PASSWORD01 "12345678"

//#define WIFISSID02 "IFPE-PUBLICA" // Put your WifiSSID here
//#define PASSWORD02 "aluno123" // Put your wifi password here

//#define WIFISSID01 "GVT-F051" // Put your WifiSSID here
//#define PASSWORD01 "1316309065" // Put your wifi password here

const char* servidorMqtt = "eltontorres.asuscomm.com";
uint16_t portaServidorMqtt = 1883;

const char* tokenMqttDisp = "ESP32_UMIDADE_SOLO";

class connection{
  public:
    //ASSINATURA DOS METODOS
    void setupWifiClient();
    boolean isWifiConnected();
    bool connectWifi();
    bool reconnectMQTT(PubSubClient *clientMQTT);
    void printMessage(const char* topic, JsonDocument& doc);
    void on_message(const char* topic, byte* payload, unsigned int length);
    void sendSensorInfoToMQTTServer(Sensor sensor, PubSubClient *clientMQTT);
    void enviarInfoParaServidorMQTT(PubSubClient *clientMQTT, String atributo, String valor);
    void imprimirInformacoesWifiEMQTT();
    void printArray(String msg, float* myArray, int size);
};
