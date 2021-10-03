// ----------------------------------- INCLUDES -----------------------------------
#include <math.h>       // Conversion equation from resistance to %
#include "ArduinoSort.h"

// --- INCLUINDO O HEADER ---
#include "classes.h"
#include "connection.h"

#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
// --------------------------------------------------------------------------------

// ----------------------------------- DEFINES ------------------------------------
//TENSÃO DE REFERENCIA DO ADC
#define DEFAULT_VREF    1086

// Setting up format for reading 3 soil sensors
#define NUM_READS           10                    /* Number of sensor reads for filtering  */
#define uS_TO_S_FACTOR      1000000               /* Conversion factor from micro seconds to seconds */
#define S_TO_MIN_FACTOR     60 * uS_TO_S_FACTOR   /* Conversion factor from seconds to minutes */
#define SLEEP_PERIOD_VALUE  30                    /* Time ESP32 will go to sleep (in seconds) */
// --------------------------------------------------------------------------------

//VERIFICAR COM HITALO
RTC_DATA_ATTR int bootCount = 0;

//VERIFICAR COM HITALO
const long knownResistor = 4700;  // Constant value of known resistor in Ohms

static const adc_bits_width_t width = ADC_WIDTH_BIT_12;
static const adc_atten_t atten = ADC_ATTEN_DB_11;
static const adc_unit_t unit = ADC_UNIT_1;
esp_adc_cal_characteristics_t adc_cal;

unsigned long lastSend = 0;
int pinRele = 0;

void setup() {
  
  adc1_config_width(ADC_WIDTH_BIT_12);//Configura a resolucao
  adc1_config_channel_atten(ADC1_CHANNEL_4, atten);//Configura a atenuacao
  adc1_config_channel_atten(ADC1_CHANNEL_5, atten);//Configura a atenuacao
  
  adc1_config_channel_atten(ADC1_CHANNEL_6, atten);//Configura a atenuacao
  adc1_config_channel_atten(ADC1_CHANNEL_7, atten);//Configura a atenuacao

  esp_adc_cal_value_t adc_type = esp_adc_cal_characterize(unit, atten, width, DEFAULT_VREF, &adc_cal);

  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(33, INPUT);
  pinMode(32, INPUT);
  pinMode(35, INPUT);
  pinMode(34, INPUT);
  
  beginClient();
  imprimirInformacoesWifiEMQTT();
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

void setupDeepSleep() {
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));
  print_wakeup_reason();
  esp_sleep_enable_timer_wakeup(SLEEP_PERIOD_VALUE * S_TO_MIN_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(SLEEP_PERIOD_VALUE) + " Minutes");
}

boolean isTimeToMeasure() {
  return lastSend == 0 || ((micros() - lastSend) > (measureTimeInterval * S_TO_MIN_FACTOR));
}
