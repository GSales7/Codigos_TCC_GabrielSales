/*************************************************************
* Autor: Gabriel Sales de Jesus
* Trabalho de Conclusão de Curso em Engenharia Eletrônica
*************************************************************/

/************************ Bibliotecas ************************/
// BMP
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

//GPS
// #include <SoftwareSerial.h>
#include <SoftwareSerial.h>
#include <TinyGPS++.h>

// MESH
#include "painlessMesh.h"
#include <Arduino_JSON.h>
/**************************************************************/

/****************** Definições e Constantes *******************/
// Rede MESH
#define MESH_PREFIX "RedeMeshComBMP280" // Nome da Rede
#define MESH_PASSWORD "123.456.789.@" // Senha da Rede
#define MESH_PORT 5555 // Porta Padrão

// Definições para o GPS
#define RX 16
#define TX 17
#define GPS_BAUD 9600

// Identificação do Nó
const int nodeNumber = 3;
/****************************************************************/

/************** Declaraões de  Variáveis e Objetos **************/
// Contador de Mensagens recebidas
int contador = 0;
// Declaração do objeto bmp
Adafruit_BMP280 bmp;

// Declaração do Objeto gps
TinyGPSPlus gps;

// Declaração da SoftwareSerial
SoftwareSerial gpsSerial(RX, TX);

// Declaração da String a ser enviada para outros nós com as leituras do sensor
String readings;

// Declaração do objeto gerenciador de tarefas
Scheduler userScheduler;

// Cria um objeto mesh para manipular a rede
painlessMesh mesh; 

void sendMessage(); // Declaração da função sendMessage
String getReadings(); // Declaração da função getReadings
/****************************************************************/

/********************* Criação das Funções *********************/
// Criação da tarefa responsável por chamar a função sendMessage a cada 5s
Task taskSendMessage(TASK_SECOND * 5, TASK_FOREVER, &sendMessage);

// Criação da função que obtem as leituras de temperatura, pressão e altitude do sensor BMP280
// e as leituras de localização, altitude e velocidade do GPS
// concatena as informações, incluindo o número do nó em uma variável JSON (jsonReadings)
String getReadings() {
  JSONVar jsonReadings;
  jsonReadings["contador"] = contador += 1;
  jsonReadings["node"] = nodeNumber;
  jsonReadings["temp"] = bmp.readTemperature();
  jsonReadings["pres"] = (bmp.readPressure() / 100.0F);
  jsonReadings["alt"] = bmp.readAltitude();
  
  // If que verifica se os dados do GPS são válidos
  if (gps.location.isValid()) {
    jsonReadings["lat"] = gps.location.lat();
    jsonReadings["lng"] = gps.location.lng();
    jsonReadings["speed"] = gps.speed.kmph();
    jsonReadings["gpsAlt"] = gps.altitude.meters();
    jsonReadings["hdop"] = gps.hdop.value() / 100.0;
    jsonReadings["sats"] = gps.satellites.value();
    
    // Ajuste do fuso horário para o de Brasília (UTC-3)
    int timezoneOffset = -3; // UTC-3 para Brasília
    int hour = gps.time.hour() + timezoneOffset;
    if (hour < 0) {
      hour += 24;
    } else if (hour > 23) {
      hour -= 24;
    }
    jsonReadings["time"] = String(hour) + ":" + String(gps.time.minute()) + ":" + String(gps.time.second());
  } else {
    jsonReadings["gps"] = "Não Fixado";
  }
  
  readings = JSON.stringify(jsonReadings); // Converte a variável JSON para string
  return readings;
}

// Criação da função responsável por enviar as mensagens para os outros nós
void sendMessage() {
  String msg = getReadings(); 
  mesh.sendBroadcast(msg);
}

// Criação da função responsável por Inicializar o BMP280
void initBMP() {
  if (!bmp.begin(0x76)) {
    Serial.println("Could not find a valid BMP280 sensor");
  }
}

// Criação da função responsável por receber as mensagens dos outros nós
void received(uint32_t from, String &msg) {

  Serial.println(msg.c_str());
  // Variável JSON que recebe os dados
  JSONVar myObject = JSON.parse(msg.c_str());
  int node = myObject["node"];
  double temp = myObject["temp"];
  double alt = myObject["alt"];
  double pres = myObject["pres"];
  
  // If para verificar se foi recebido algum dado do GPS
  if (myObject.hasOwnProperty("lat")) {
    double lat = myObject["lat"];
    double lng = myObject["lng"];
    double speed = myObject["speed"];
    double gpsAlt = myObject["gpsAlt"];
    double hdop = myObject["hdop"];
    int sats = myObject["sats"];
    String time = myObject["time"];
  } 
}

// Criação da função responsável por avisar que houve uma nova conexão na rede
void newConnection(uint32_t nodeId) {
  Serial.printf("--> Início Aqui: Nova Conexão, nodeId = %u\n", nodeId);
}

// Criação da função responsével por avisar que ocorreram mudanças na rede
void changedConnection() {
  Serial.println("Conexões Mudaram");
}

// Criação da função responsável por ajustar o tempo da rede
void nodeTimeAdjusted(int32_t offset) {
  Serial.printf("Tempo ajustado %u. Offset = %d\n", mesh.getNodeTime(), offset);
}

/****************************************************************/

void setup() {
  // Inicia a porta Serial
  Serial.begin(115200);

  // Chamada da Função que inicia o sensor BMP280
  initBMP();

  // Inicia a serial do GPS
  gpsSerial.begin(GPS_BAUD);
  Serial.println("Software Serial para o GPS iniciada com Baud Rate de 9600");

  // Inicialização da Rede Mesh e Suas funções
  mesh.setDebugMsgTypes(ERROR | STARTUP);
  mesh.init(MESH_PREFIX, MESH_PASSWORD, &userScheduler, MESH_PORT);

  // Chamando função de recebimento de mensagem de outros dispositivos da rede
  mesh.onReceive(&received);

  // Chamando as funções responsáveis por monitorar as mudanças que ocorram na rede
  mesh.onNewConnection(&newConnection);
  mesh.onChangedConnections(&changedConnection);
  mesh.onNodeTimeAdjusted(&nodeTimeAdjusted);

  // Adicionando e ativando tarefa de envio de mensagem
  userScheduler.addTask(taskSendMessage);
  taskSendMessage.enable();
}

void loop() {
  
  // Atualiza a malha da Rede Mesh
  mesh.update();

  // Atualiza os dados do GPS sem bloquear o loop
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }
}
