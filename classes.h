#include <esp_adc_cal.h>
#include "adc.h"

class SensorMeasure{

  public:
    //ATRIBUTOS DA CLASSE
    int supplyVoltageRaw;
    float supplyVoltage;
    int sensorVoltageRaw;
    float sensorVoltage;
    int moisture;
    long resistance;

    //ASSINATURA DOS METODOS
    void printAttr();
};

class SensorUmidade{
  
  public:
    //ATRIBUTOS DA CLASSE
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
    SensorMeasure lastMeasure;

    //CONSTRUTOR
    SensorUmidade(int sensor_id, int sensor_moisture, int sensor_phase_a, int sensor_phase_b, int sensor_analog_input, int sensor_analog_input2, adc1_channel_t sensor_channel, adc1_channel_t sensor_channel2, long sensor_knownResistor);  

    //ASSINATURA DOS METODOS
    void putSensorToImpedance();
    void setupChannelsAndPins();
    void printAttr();
};

class Measurer{
 
  public:
    //ATRIBUTOS DA CLASSE
    SensorUmidade sensor;
    
    //CONSTRUTOR
    Measurer(SensorUmidade s1);

    //ASSINATURA DOS METODOS
    void excitateSensorIntern();
    void measureSensor(int numReads);
    void measure(SensorMeasure *resultS, int numReads);
    int analogReadByTable(int analog_input);
    float rawToVoltageByTable(int value);
    int analogReadByAPI(adc1_channel_t channel);
    float rawToVoltageByAPI(int value);
    SensorMeasure average(SensorMeasure results[], int numReads);
    SensorMeasure average(SensorMeasure readingA, SensorMeasure readingB);
};


