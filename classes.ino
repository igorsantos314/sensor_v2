class Values{
  //ATRIBUTOS DA CLASSE
  int supplyVoltageRaw;
  float supplyVoltage;
  int sensorVoltageRaw;
  float sensorVoltage;
  int moisture;
  long resistance;
  
};

class SensorUmidade{

  //ATRIBUTOS DA CLASSE
  int sensor_id;
  int sensor_moisture;
  int sensor_phase_a;
  int sensor_phase_b;
  int sensor_analog_input;
  int sensor_analog_input2;
  adc1_channel_t sensor_channel;
  adc1_channel_t sensor_channel2;
  long sensor_knownResistor;
  float sensor_weight;
  Values sensor_lastMeasure;
  
  //CONSTRUTOR
  public:SensorUmidade(int id, int moisture, int phase_a, int phase_b, int analog_input, int analog_input2, adc1_channel_t channel, adc1_channel_t channel2, long knownResistor){
    sensor_id = id;
    sensor_moisture = moisture;
    sensor_phase_a = phase_a;
    sensor_phase_b = phase_b;
    sensor_analog_input = analog_input; 
    sensor_analog_input2 = analog_input2;
    sensor_channel = channel;
    sensor_channel2 = channel2;
    sensor_knownResistor = knownResistor;
  }

  void setupDeepSleep() {
    ++bootCount;
    Serial.println("Boot number: " + String(bootCount));
    print_wakeup_reason();
    esp_sleep_enable_timer_wakeup(SLEEP_PERIOD_VALUE * S_TO_MIN_FACTOR);
    Serial.println("Setup ESP32 to sleep for every " + String(SLEEP_PERIOD_VALUE) + " Minutes");
  }
};
