#include <math.h>       // Conversion equation from resistance to %
#include "ArduinoSort.h"

#include <esp_adc_cal.h>
#include "adc.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

unsigned long myChannelNumber = 1110526;
const char * myWriteAPIKey = "2JT0XT1NLS0GOJ4W";

int qttOfMeasures        = 10;                    /* Time ESP32 will go to sleep (in seconds) */
int measureTimeInterval  = 1;                    /* Time ESP32 will go to sleep (in seconds) */
int currQttOfMeasures    = 0;

const long knownResistor = 4700;  // Constant value of known resistor in Ohms

#define DEFAULT_VREF    1086

// Setting up format for reading 3 soil sensors
#define NUM_READS           10                    /* Number of sensor reads for filtering  */
#define uS_TO_S_FACTOR      1000000               /* Conversion factor from micro seconds to seconds */
#define S_TO_MIN_FACTOR     60 * uS_TO_S_FACTOR   /* Conversion factor from seconds to minutes */
#define SLEEP_PERIOD_VALUE  30                    /* Time ESP32 will go to sleep (in seconds) */

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
esp_adc_cal_characteristics_t adc_cal;

struct values {        // Structure to be used in percentage and resistance values matrix to be filtered (have to be in pairs)
  int supplyVoltageRaw;
  float supplyVoltage;
  int sensorVoltageRaw;
  float sensorVoltage;
  int moisture;
  long resistance;
};

struct Sensor {
  int id;
  int moisture;
  int phase_a;
  int phase_b;
  int analog_input;
  int analog_input2;
  adc1_channel_t channel;
  adc1_channel_t channel2;
  long knownResistor;
  float weight;
  values lastMeasure;
};

//  ----------------------------------------------------------------------------------- //
Sensor sensor1, sensor2;
Sensor *sensor1_ptr, *sensor2_ptr;

RTC_DATA_ATTR int bootCount = 0;

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

unsigned long lastSend = 0;

int pinRele = 0;

void setup() {
  Serial.begin(9600);
  setupChannelsAndPins();
  setupWifiClient();
  pinMode(5, OUTPUT);
  imprimirInformacoesWifiEMQTT();
  sensor1_ptr = &sensor1;
  sensor2_ptr = &sensor2;
  declareSensor(sensor1_ptr, 5, 23, 22, 34, 34, ADC1_CHANNEL_6, ADC1_CHANNEL_6, knownResistor);
  declareSensor(sensor2_ptr, 6, 21, 19, 35, 35, ADC1_CHANNEL_7, ADC1_CHANNEL_7, knownResistor);
  //declareSensor(sensor2_ptr, 6, 23, 22, 34, 34, ADC1_CHANNEL_6, ADC1_CHANNEL_6, knownResistor);
  setupDeepSleep();
  Serial.println("//--------------------------------------------------------------//");
}

void loop() {
  //CASO SEJA TEMPO DE MENSURAR
  if (isTimeToMeasure()) {
    
    //VERIFICAR SE A QUANTIDADE DE AFERIÇOES JA CHEGOU NO LIMITE
    if (currQttOfMeasures < qttOfMeasures) {     
      
      Serial.println("Aferição dos sensores...");

      //AFERE O SENSOR 1
      measureSensor(sensor1_ptr);

      //AFERE O SENSOR 2
      measureSensor(sensor2_ptr);

      //CONECTAR COM O WIFI
      boolean isWifiConnected = connectWifi();
      
      //VERIFICA SE HA CONEXAO
      if (isWifiConnected) {
        
        //CRIA CONEXAO MQTT
        boolean isMQTTConnected = reconnectMQTT(&client);

        //VERIFICA SE HA CONEXAO COM O SERVIDOR MQTT
        if (isMQTTConnected) {
          Serial.println("Envio das informações para os servidores...");

          //ENVIAR OS DADOS DO SENSOR 1
          sendSensorInfoToMQTTServer(sensor1, &client);

          //ENVIAR OS DADOS DO SENSOR 2
          sendSensorInfoToMQTTServer(sensor2, &client);
        }
      }
      Serial.println("//--------------------------------------------------------------//");

      //AUMENTA QUANTIDADE DE AFERICOES
      currQttOfMeasures++;
      lastSend = micros();
    } 
    //ENTRA NO DEEP SLEEP QUANDO A QUANTIDADE DE AFERICOES CHEGA NO LIMITE
    else {
      Serial.println("Going to sleep now");
      Serial.flush();
      esp_deep_sleep_start();
    }
  }
  client.loop();
}

boolean isTimeToMeasure() {
  return lastSend == 0 || ((micros() - lastSend) > (measureTimeInterval * S_TO_MIN_FACTOR));
}

//Questionamento ??
void excitateSensor(Sensor *sensor) {
  excitateSensorIntern(sensor->phase_a, sensor->phase_b);
}

void excitateSensorIntern (int phase_a, int phase_b) {
  // read sensor, filter, and calculate resistance value
  // Noise filter: median filter
  for (int i = 0; i < 100; i++) {
    pinMode(phase_a, OUTPUT);
    pinMode(phase_b, INPUT);
    digitalWrite(phase_a, HIGH);                 // set the voltage supply on
    delayMicroseconds(100);
    digitalWrite(phase_a, LOW);                  // set the voltage supply off
    
    delayMicroseconds(100);

    pinMode(phase_a, INPUT);
    pinMode(phase_b, OUTPUT);
    digitalWrite(phase_b, HIGH);                 // set the voltage supply on
    delayMicroseconds(100);
    digitalWrite(phase_b, LOW);                  // set the voltage supply off
  }
}

void putSensorToImpedance(Sensor *sensor) {
  pinMode(sensor->phase_a, INPUT);
  pinMode(sensor->phase_b, INPUT);
}

void setupDeepSleep() {
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(SLEEP_PERIOD_VALUE * S_TO_MIN_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(SLEEP_PERIOD_VALUE) +
                 " Minutes");
}

void setupChannelsAndPins() {
  adc1_config_width(ADC_WIDTH_BIT_12);//Configura a resolucao
  adc1_config_channel_atten(ADC1_CHANNEL_4, atten);//Configura a atenuacao
  adc1_config_channel_atten(ADC1_CHANNEL_5, atten);//Configura a atenuacao

  adc1_config_channel_atten(ADC1_CHANNEL_6, atten);//Configura a atenuacao
  adc1_config_channel_atten(ADC1_CHANNEL_7, atten);//Configura a atenuacao

  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &adc_cal);

  pinMode(33, INPUT);
  pinMode(32, INPUT);
  pinMode(35, INPUT);
  pinMode(34, INPUT);
}

void print_wakeup_reason() {
  esp_sleep_wakeup_cause_t wakeup_reason;
  wakeup_reason = esp_sleep_get_wakeup_cause();
  switch (wakeup_reason) {
    case ESP_SLEEP_WAKEUP_EXT0 : Serial.println("Wakeup caused by external signal using RTC_IO"); break;
    case ESP_SLEEP_WAKEUP_EXT1 : Serial.println("Wakeup caused by external signal using RTC_CNTL"); break;
    case ESP_SLEEP_WAKEUP_TIMER : Serial.println("Wakeup caused by timer"); break;
    case ESP_SLEEP_WAKEUP_TOUCHPAD : Serial.println("Wakeup caused by touchpad"); break;
    case ESP_SLEEP_WAKEUP_ULP : Serial.println("Wakeup caused by ULP program"); break;
    default : Serial.printf("Wakeup was not caused by deep sleep: %d\n", wakeup_reason); break;
  }
}

void declareSensor(Sensor *sensor, int id, int phase_a, int phase_b, int analog_input, int analog_input2, adc1_channel_t channel, adc1_channel_t channel2, long knownResistor) {
  sensor->id = id;
  //  sensor.moisture;
  sensor->phase_a = phase_a;
  sensor->phase_b = phase_b;
  sensor->analog_input = analog_input;
  sensor->analog_input2 = analog_input2;
  sensor->channel = channel;
  sensor->channel2 = channel2;
  sensor->knownResistor = knownResistor;
}

void measureSensor(Sensor *sensor) {
  values results[NUM_READS];
  excitateSensor(sensor);
  putSensorToImpedance(sensor);
  //measure(results, sensor->phase_a, sensor->phase_b, sensor->analog_input, sensor->channel, sensor->knownResistor);
  //values firstReading = average(results);
  measure(results, sensor->phase_b, sensor->phase_a, sensor->analog_input2, sensor->channel2, sensor->knownResistor);
  values secondReading = average(results);
  values firstReading = secondReading;
  sensor->lastMeasure = average(firstReading, secondReading);
  printOutputSensor(*sensor, firstReading.resistance, secondReading.resistance);
  printValues(firstReading);
  printValues(secondReading);
  printValues(sensor->lastMeasure);
}

void printOutputSensor(Sensor sensor, long firstReading, long secondReading) {
  long bias = abs(firstReading - secondReading);
  long readingAvg = (firstReading + secondReading) / 2;
  Serial.print ("Sensor ");
  Serial.print (sensor.id);
  Serial.print ("\t" );
  Serial.print ("read 1\t" );
  Serial.print (firstReading);
  Serial.print ("\t" );
  Serial.print ("read 2\t" );
  Serial.print (secondReading);
  Serial.print ("\t" );
  Serial.print ("bias\t" );
  Serial.print (bias);
  Serial.print ("\t" );
  Serial.print ("value\t");
  Serial.println (readingAvg);
}

void measure (values *results, int phase_a, int phase_b, int analog_input, adc1_channel_t channel, long knownResistor) {
  // read sensor, filter, and calculate resistance value
  // Noise filter: median filter
  values curr;
  for (int i = 0; i < NUM_READS; i++) {
    pinMode(phase_a, OUTPUT);
    pinMode(phase_b, INPUT);

    digitalWrite(phase_a, HIGH);                 // set the voltage supply on
    delay(10);
    curr.supplyVoltageRaw = analogReadByTable(analog_input);   // read the supply voltage
    delayMicroseconds(100);
    digitalWrite(phase_a, LOW);                  // set the voltage supply off
    delay(1);

    pinMode(phase_a, INPUT);
    pinMode(phase_b, OUTPUT);

    digitalWrite(phase_b, HIGH);                 // set the voltage supply on
    delay(10);
    curr.sensorVoltageRaw = analogReadByAPI(channel);
    delayMicroseconds(100);
    digitalWrite(phase_b, LOW);                  // set the voltage supply off

    // Calculate resistance
    curr.supplyVoltage = rawToVoltageByTable(curr.supplyVoltageRaw);
    curr.sensorVoltage = rawToVoltageByAPI(curr.sensorVoltageRaw);
    //if (curr.sensorVoltage <= 0.17) {//Esse valor foi definido abrindo o circuito (resistor) e aferindo o valor de sensorVoltageFinal
    //  curr.sensorVoltage = 0.1; // Evitar exceção e elevar o valor resistência para próximo do máximo possível aferido.
    //}
    curr.resistance = (knownResistor * (curr.supplyVoltage - curr.sensorVoltage ) / curr.sensorVoltage) ;
    delay(1);
    results[i] = curr;
    //    printValues(curr);
  }
}

void printValues(values value) {
  Serial.print("supplyVoltage: ");
  Serial.print(value.supplyVoltageRaw);
  Serial.print("\t");
  Serial.print(value.supplyVoltage);
  Serial.print("\t");
  Serial.print("sensorVoltage: ");
  Serial.print(value.sensorVoltageRaw);
  Serial.print("\t");
  Serial.print(value.sensorVoltage);
  Serial.print("\t");
  Serial.print("resistance: ");
  Serial.print("\t");
  Serial.println(value.resistance);
}

int analogReadByTable(int analog_input) {
  int voltage = analogRead(analog_input);
  return ADC_LUT[voltage];
}

float rawToVoltageByTable(int value) {
  return value * 3.3 / 4095;
}

int analogReadByAPI(adc1_channel_t channel) {
  return adc1_get_raw(channel);//Obtem o valor RAW do ADC
}

float rawToVoltageByAPI(int value) {
  return esp_adc_cal_raw_to_voltage(value, &adc_cal) / 1000.0;
}

values average(values results[]) {
  values result;
  result.supplyVoltageRaw = 0;
  result.supplyVoltage = 0.0;
  result.sensorVoltageRaw = 0;
  result.sensorVoltage = 0.0;
  result.resistance = 0;
  long sum = 0;
  for (int i = 0; i < NUM_READS; i++) {
    result.supplyVoltageRaw += results[i].supplyVoltageRaw;
    result.supplyVoltage += results[i].supplyVoltage;
    result.sensorVoltageRaw += results[i].sensorVoltageRaw;
    result.sensorVoltage += results[i].sensorVoltage;
    result.resistance += results[i].resistance;
  }
  result.supplyVoltageRaw = result.supplyVoltageRaw / NUM_READS;
  result.supplyVoltage = result.supplyVoltage / NUM_READS;
  result.sensorVoltageRaw = result.sensorVoltageRaw / NUM_READS;
  result.sensorVoltage = result.sensorVoltage / NUM_READS;
  result.resistance = result.resistance / NUM_READS;
  return result;
}

values average(values readingA, values readingB) {
  values result;
  result.supplyVoltageRaw = (readingA.supplyVoltageRaw + readingB.supplyVoltageRaw) / 2;
  result.supplyVoltage = (readingA.supplyVoltage + readingB.supplyVoltage) / 2;
  result.sensorVoltageRaw = (readingA.sensorVoltageRaw + readingB.sensorVoltageRaw) / 2;
  result.sensorVoltage = (readingA.sensorVoltage + readingB.sensorVoltage) / 2;
  result.resistance = (readingA.resistance + readingB.resistance) / 2;
  return result;
}

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
    Serial.println("Tópico não tratado!");
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
    Serial.println("Não foi possivel conectar com o servidor MQTT!");
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
  //  Serial.println("Mensagem não enviada!");
  //}
}

void imprimirInformacoesWifiEMQTT() {
  // Imprimir os valores-padrão das variáveis referentes ao WiFi e servidor MQTT
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
