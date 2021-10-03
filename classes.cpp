//CLASSE COM ATRIBUTOS E METODOS REFENRENTES AO SENSOR

#include "classes.h"

SensorUmidade::SensorUmidade(int sensor_id, int sensor_moisture, int sensor_phase_a, int sensor_phase_b, int sensor_analog_input, int sensor_analog_input2, adc1_channel_t sensor_channel, adc1_channel_t sensor_channel2, long sensor_knownResistor){
  id = sensor_id;
  moisture = sensor_moisture;
  phase_a = sensor_phase_a;
  phase_b = sensor_phase_b;
  analog_input = sensor_analog_input; 
  analog_input2 = sensor_analog_input2;
  channel = sensor_channel;
  channel2 = sensor_channel2;
  knownResistor = sensor_knownResistor;
}

void SensorUmidade::putSensorToImpedance() {
  pinMode(phase_a, INPUT);
  pinMode(phase_b, INPUT);
} 

void SensorUmidade::setupChannelsAndPins() {
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

void SensorUmidade::printAttr() {
  firstReading = lastMeasure;
  secondReading = lastMeasure;

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

// ---------------------------------------------- MEASURE ----------------------------------------------

public Measurer::Measurer(SensorUmidade s1){
  sensor = s1;
}

//phase_a e b, sao atributos da classe SensorUmidade - CORRIGIDA
void Measurer::excitateSensorIntern () {
  // read sensor, filter, and calculate resistance value
  // Noise filter: median filter
  for (int i = 0; i < 100; i++) {
    pinMode(sensor.phase_a, OUTPUT);
    pinMode(sensor.phase_b, INPUT);
    digitalWrite(sensor.phase_a, HIGH);                 // set the voltage supply on
    delayMicroseconds(100);
    digitalWrite(sensor.phase_a, LOW);                  // set the voltage supply off
    
    delayMicroseconds(100);

    pinMode(sensor.phase_a, INPUT);
    pinMode(sensor.phase_b, OUTPUT);
    digitalWrite(sensor.phase_b, HIGH);                 // set the voltage supply on
    delayMicroseconds(100);
    digitalWrite(sensor.phase_b, LOW);                  // set the voltage supply off
  }
}

// CORRIGIDA
void Measurer::measureSensor(int numReads) {
  SensorMeasure results[numReads];
  
  //INICIALIZA O SENSOR - CORRENTE FASE A E B
  excitateSensorIntern();

  //PINOS DE OUTPUT PARA INPUT
  sensor.putSensorToImpedance();

  //FAZ A AFERIÇÃO
  measure(results, numReads);
  
  //CALCULA A MÉDIA DOS 10 ULTIMOS RESULTADOS
  SensorMeasure averageResults = average(results);

  //SALVA O ULTIMO RESULTADO
  sensor.lastMeasure = averageResults;

  //EXIBE AS INFO DO SENSOR
  sensor.printAttr();

  //EXIBE AS INFORMAÇÕES DA AFERIÇÃO
  averageResults.printAttr();

  //EXIBE AS INFORMAÇÕES DA ULTIMA AFERIÇÃO
  sensor.lastMeasure.printAttr();
}

void Measurer::measure(SensorMeasure *resultS, int numReads) {
  // read sensor, filter, and calculate resistance value
  // Noise filter: median filter

  //MENSURAÇÃO ATUAL
  SensorMeasure curr;

  //REALIZA A AFERIÇÃO numReads VEZES E APPEND EM RESULTS
  for (int i = 0; i < numReads; i++) {
    pinMode(sensor.phase_a, OUTPUT);
    pinMode(sensor.phase_b, INPUT);
    
    digitalWrite(sensor.phase_a, HIGH);                 // set the voltage supply on
    delay(10);

    //CONVERTE DE ANALÓGIOC PARA DIGITAL
    curr.supplyVoltageRaw = analogReadByTable(sensor.analog_input);   // read the supply voltage
    
    delayMicroseconds(100);
    digitalWrite(sensor.phase_a, LOW);                  // set the voltage supply off
    delay(1);
    
    pinMode(sensor.phase_a, INPUT);
    pinMode(sensor.phase_b, OUTPUT);

    digitalWrite(sensor.phase_b, HIGH);                 // set the voltage supply on
    delay(10);

    //SENSOR O SINAL DIGITAL
    curr.sensorVoltageRaw = analogReadByAPI(sensor.channel);
    
    delayMicroseconds(100);
    digitalWrite(sensor.phase_b, LOW);                  // set the voltage supply off
    
    // Calculate resistance
    curr.supplyVoltage = rawToVoltageByTable(curr.supplyVoltageRaw);
    curr.sensorVoltage = rawToVoltageByAPI(curr.sensorVoltageRaw);
    
    //if (curr.sensorVoltage <= 0.17) {//Esse valor foi definido abrindo o circuito (resistor) e aferindo o valor de sensorVoltageFinal
    //  curr.sensorVoltage = 0.1; // Evitar exceção e elevar o valor resistência para próximo do máximo possível aferido.
    //}

    //REALIZA O CALCULO DA RESISTÊNCIA
    curr.resistance = (knownResistor * (curr.supplyVoltage - curr.sensorVoltage ) / curr.sensorVoltage) ;
    
    delay(1);

    //ARMAZENA O VALOR NO POSIÇÃO i CORRESPONDENTE
    results[i] = curr;
  }
}

//CORRIGIDA
int Measurer::analogReadByTable(int analog_input) {
  int voltage = analogRead(analog_input);
  return ADC_LUT[voltage];
}

//OK
float Measurer::rawToVoltageByTable(int value) {
  return value * 3.3 / 4095;
}

//CORRGIDA
int Measurer::analogReadByAPI(adc1_channel_t channel) {
  return adc1_get_raw(channel);//Obtem o valor RAW do ADC
}

//OK
float Measurer::rawToVoltageByAPI(int value) {
  return esp_adc_cal_raw_to_voltage(value, &adc_cal) / 1000.0;
}

SensorMeasure Measurer::average(SensorMeasure results[], int numReads) {
  SensorMeasure result;
  
  result.supplyVoltageRaw = 0;
  result.supplyVoltage = 0.0;
  result.sensorVoltageRaw = 0;
  result.sensorVoltage = 0.0;
  result.resistance = 0;
  
  long sum = 0;
  
  for (int i = 0; i < numReads; i++) {
    result.supplyVoltageRaw += results[i].supplyVoltageRaw;
    result.supplyVoltage += results[i].supplyVoltage;
    result.sensorVoltageRaw += results[i].sensorVoltageRaw;
    result.sensorVoltage += results[i].sensorVoltage;
    result.resistance += results[i].resistance;
  }
  
  result.supplyVoltageRaw = result.supplyVoltageRaw / numReads;
  result.supplyVoltage = result.supplyVoltage / numReads;
  result.sensorVoltageRaw = result.sensorVoltageRaw / numReads;
  result.sensorVoltage = result.sensorVoltage / numReads;
  result.resistance = result.resistance / numReads;
  
  return result;
}

SensorMeasure Measurer::average(SensorMeasure readingA, SensorMeasure readingB) {
  SensorMeasure result;
  
  result.supplyVoltageRaw = (readingA.supplyVoltageRaw + readingB.supplyVoltageRaw) / 2;
  result.supplyVoltage = (readingA.supplyVoltage + readingB.supplyVoltage) / 2;
  result.sensorVoltageRaw = (readingA.sensorVoltageRaw + readingB.sensorVoltageRaw) / 2;
  result.sensorVoltage = (readingA.sensorVoltage + readingB.sensorVoltage) / 2;
  result.resistance = (readingA.resistance + readingB.resistance) / 2;
  
  return result;
}

---------------------------------------------- SENSOR MEASURE ---------------------------------------------- 
void SensorMeasure::printAttr() {
  Serial.print("supplyVoltage: ");
  Serial.print(supplyVoltageRaw);
  Serial.print("\t");
  Serial.print(supplyVoltage);
  Serial.print("\t");
  Serial.print("sensorVoltage: ");
  Serial.print(sensorVoltageRaw);
  Serial.print("\t");
  Serial.print(sensorVoltage);
  Serial.print("\t");
  Serial.print("resistance: ");
  Serial.print("\t");
  Serial.println(resistance);
}
