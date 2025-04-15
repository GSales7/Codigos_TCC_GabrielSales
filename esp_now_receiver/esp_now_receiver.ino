/***********************************************************
* Autor: Gabriel Sales de Jesus
* Trabalho de Conclusão de Curso em Engenharia Eletrônica
***********************************************************/

/********************** Bibliotecas ***********************/
// ESP NOW
#include <esp_now.h>
#include <WiFi.h>

// Sensor BMP280
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

/*********************************************************/

/*********** Definições de Objetos e Variáveis ***********/
// ID do Nó
#define NODE 1 

// I2C bmp
Adafruit_BMP280 bmp;

/*********************************************************/

/***************** Estrutura dos Dados *******************/
// Estrutura para enviar os dados do BMP280 e GPS
typedef struct struct_message {
    int node;
    int contador;
    float temp;
    float pres;
    float alt;
    double gps_lat;
    double gps_long;
    float gps_speed;
    float gps_alt;
    int gps_sats;
    float gps_hdop;
    char gps_time[20]; // Horário formatado
} struct_message;

/*********************************************************/

// Criando uma struct_message chamada meusDados
struct_message meusDados;

// Criando uma estrutura para armazernar as leituras dos outros nós
struct_message leituraNo2;
struct_message leituraNo3;

// Criando um array com todas as estruturas
struct_message boardsStruct[2] = {leituraNo2, leituraNo3};

// Criação da função que exibirá os dados recebidos na serial
void exibirDados(struct_message dados) {
  Serial.print("Nó: ");
  Serial.println(dados.node);
  Serial.print("Contador: ");
  Serial.println(dados.contador);
  Serial.print("Temperatura: ");
  Serial.println(dados.temp);
  Serial.print("Pressão: ");
  Serial.println(dados.pres);
  Serial.print("Altitude: ");
  Serial.println(dados.alt);

  Serial.print("GPS LAT: ");
  Serial.println(dados.gps_lat, 6);
  Serial.print("GPS LONG: ");
  Serial.println(dados.gps_long, 6);
  Serial.print("GPS Velocidade (km/h): ");
  Serial.println(dados.gps_speed);
  Serial.print("GPS Altitude (m): ");
  Serial.println(dados.gps_alt);
  Serial.print("Satélites GPS: ");
  Serial.println(dados.gps_sats);
  Serial.print("HDOP GPS: ");
  Serial.println(dados.gps_hdop);
  Serial.print("Horário GPS: ");
  Serial.println(dados.gps_time);
};

// Criando a função Callback que irá executar quando receber os dados
void OnDataRecv(const uint8_t *mac_addr, const uint8_t *receivedData, int len) {
  // Criando variável para armazenar o endereço MAC
  char macStr[18];
  Serial.print("Pacote recebido de: ");
  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", 
            mac_addr[0],  mac_addr[1], mac_addr[2],  mac_addr[3], mac_addr[4], mac_addr[5]);
  Serial.println(macStr);

  // Converte os dados recebidos para a estrutura `struct_message`
  memcpy(&meusDados, receivedData, sizeof(meusDados));
  Serial.printf("Board ID %u: %u bytes\n", meusDados.node, len);
  
  // Atualiza as estruturas com os novos dados recebidos
  boardsStruct[meusDados.node-2].node = meusDados.node;
  boardsStruct[meusDados.node-2].contador = meusDados.contador;
  boardsStruct[meusDados.node-2].temp = meusDados.temp;
  boardsStruct[meusDados.node-2].pres = meusDados.pres;
  boardsStruct[meusDados.node-2].alt = meusDados.alt;
  boardsStruct[meusDados.node-2].gps_lat = meusDados.gps_lat;
  boardsStruct[meusDados.node-2].gps_long = meusDados.gps_long;
  boardsStruct[meusDados.node-2].gps_speed = meusDados.gps_speed;
  boardsStruct[meusDados.node-2].gps_alt = meusDados.gps_alt;
  boardsStruct[meusDados.node-2].gps_sats = meusDados.gps_sats;
  boardsStruct[meusDados.node-2].gps_hdop = meusDados.gps_hdop;
 
  // Corrige a atribuição da string gps_time
  strncpy(boardsStruct[meusDados.node-2].gps_time, meusDados.gps_time, sizeof(meusDados.gps_time));

  exibirDados(boardsStruct[meusDados.node-2]);
  Serial.println();
}

void setup() {
  // Iniciando Serial
  Serial.begin(115200);

  // Define o dispositivo como Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Inicia a rede ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW");
    return;
  }

  // Registro de status de transmissão, para pegar informações dos dados recebidos
  esp_now_register_recv_cb(esp_now_recv_cb_t(OnDataRecv));
}

void loop() {

}
