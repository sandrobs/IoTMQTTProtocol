#include <SFE_BMP180.h>  //Biblioteca para sensor BMP180
#include <Wire.h>        //Biblioteca para comunicação I2C
#include <Arduino_JSON.h>
#include <assert.h>
#include <ArduinoMqttClient.h>
#include <ESP8266WiFi.h>

// Define as informações da sua rede Wi-Fi
const char* ssid = "ssid";
const char* pass = "pass";

WiFiClient wifiClient;
MqttClient mqttClient(wifiClient);

//Define variaveis de conexão com o Broker utilizado para testess com o protocolo MQTT
const char broker[] = "test.mosquitto.org";
int        port     = 1883;
const char topic[]  = "IoTPosCloudSetrem";

const long interval = 1000;
unsigned long previousMillis = 0;

 //Define objeto sensor na classe SFE_BMP180 da biblioteca
SFE_BMP180 sensor; 

#define ALTITUDE 340  // Altitude do apartamento onde foi desenvolvido o protótipo

char status;              //Variável auxiliar para verificação do resultado
double temperatura;       //variável para armazenar o valor da temperatura
double pressao;           //variável para armazenar o valor da pressão absoluta
double pressao_relativa;  //variável para armazenar a pressão relativa

void setup() {

  //delay para contornar problema em que o print na porta Serial não estava funcionando
  delay(10000);

  Serial.begin(9600);  //Inicializa comunicação serial com velocidade de 9600

  while (!Serial) {
    ; //espera conexão com a porta serial
  }

  // Tenta conectar na rede Wi-Fi
  Serial.print("Tentando conectar ao WPA SSID: ");
  Serial.println(ssid);
  while (WiFi.begin(ssid, pass) != WL_CONNECTED) {
    // fallhou, tenta novamente
    Serial.print(".");
    delay(5000);
  }

  Serial.println("Placa NodeMCU conectada à rede Wi-Fi.");
  Serial.println();

  Serial.print("Tentando conectar ao broker MQTT: ");
  Serial.println(broker);

  if (!mqttClient.connect(broker, port )) {
    Serial.print("MQTT connection failed! Error code = ");
    Serial.println(mqttClient.connectError());

    while (1);
  }

  Serial.println("Placa NodeMCU conectada ao broker MQTT.");
  Serial.println();

  //Inicialização do sensor
  if (sensor.begin()) {

    Serial.println("===========================================================");
    Serial.println("Sensor BMP180 inicializado com sucesso");
  } else {
    Serial.println("Falha na inicializacao do sensor BMP180\n\n");
    while (1)
      ;
  }

}

void loop() {
  
  readSensor();

}

void readSensor() {

  //Leitura da temperatura
  status = sensor.startTemperature();  //Inicializa a leitura da temperatura

  if (status != 0)  //se status for diferente de zero (sem erro de leitura)
  {
    delay(status);                                //Realiza uma pequena pausa para que a leitura seja finalizada
    status = sensor.getTemperature(temperatura);  //Armazena o valor da temperatura na variável temperatura
    if (status != 0)                              //se status for diferente de zero (sem erro de leitura)
    {
      Serial.print("Temperatura: ");      //Imprime "Temperatura: " na serial
      Serial.print(temperatura, 1);       //Imprime o valor da variável temperatura com uma casa decimal após a vírgula
      Serial.println(" graus Celsius ");  //Imprime " graus Celsius " na serial
      //Leitura da Pressão Absoluta
      status = sensor.startPressure(3);  //Inicializa a leitura
      if (status != 0)                   //se status for diferente de zero (sem erro de leitura)
      {

        delay(status);                                      //Realiza uma pequena pausa para que a leitura seja finalizada

        status = sensor.getPressure(pressao, temperatura);  //Atribui o valor medido de pressão à variável pressao, em função da variável temperatura

        if (status != 0)                                    //se status for diferente de zero (sem erro de leitura)
        {

          //Imprime na serial o valor da Pressão absoluta
          Serial.print("Pressao absoluta: ");
          Serial.print(pressao, 1);
          Serial.println(" hPa ");

          //Leitura da Pressão Relativa
          pressao_relativa = sensor.sealevel(pressao, ALTITUDE);  //Atribui o valor medido de pressão relativa à variavel pressao_relativa, em função da ALTITUDE
          Serial.print("Pressao relativa ao nivel do mar: ");
          Serial.print(pressao_relativa, 1);  //Imprime na serial o valor da pressão relativa
          Serial.println(" hPa ");

          //Imprimir a Altitude
          Serial.print("Altitude: ");
          Serial.print(ALTITUDE, 0);
          Serial.println(" metros ");

          sendTopicMqqt();

        }
      }
    }
  } else Serial.println("Erro na leitura do sensor\n");

  Serial.println("===========================================================");
  delay(5000);

}

void sendTopicMqqt() {

  //Cria Json com valores obtidos pelo sensor
  JSONVar myObject;

  myObject["temperatura"] = (double) temperatura;
  myObject["pressao"] = (double) pressao;
  myObject["pressao_relativa"] = (double) pressao_relativa;
  myObject["altitude"] = (int) ALTITUDE;

  String jsonString = JSON.stringify(myObject);

  mqttClient.poll();

  unsigned long currentMillis = millis();

  if (currentMillis - previousMillis >= interval) {

    // armazena a ultima vez que o tópico foi enviado
    previousMillis = currentMillis;

    Serial.print("Enviando mensagem para o tópico: ");
    Serial.println(topic);
    Serial.println(jsonString);

    // envia a mensagem
    mqttClient.beginMessage(topic);
    mqttClient.print(jsonString);
    mqttClient.endMessage();

    Serial.println();

    delay(5000);

  }

}