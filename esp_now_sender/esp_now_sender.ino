/*********************************************************
* Autor: Gabriel Sales de Jesus
* Trabalho de Conclusão de Curso em Engenharia Eletrônica
*********************************************************/

/********************** Bibliotecas *********************/
// ESP NOW
#include <esp_now.h>
#include <WiFi.h>

// Sensor BMP280
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// GPS
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
/********************************************************/

/*********** Definições de Objetos e Variáveis **********/
// ID do Nó
#define NODE 10

// I2C bmp
Adafruit_BMP280 bmp;

// GPS
#define RX 16
#define TX 17
#define GPS_BAUD 9600
#define TIMEZONEOFFSET -3
TinyGPSPlus gps;
SoftwareSerial gpsSerial(RX, TX);

int contador = 0;
int intervaloEnvio = 5000; // Intervalo de 5s
long ultimoEnvio = 0;
/*********************************************************/

// Endereço MAC do receptor
uint8_t broadcastAddress[] = {0x3C, 0x61, 0x05, 0x14, 0x57, 0x64}; //ESP1

/******************* Estrutura dos Dados *****************/
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

/************** Instanciando as Estruturas ***************/
struct_message BMP280GPSReadings;
/*********************************************************/

esp_now_peer_info_t peerInfo;

// Callback quando os dados são enviados
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nStatus do último envio: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Entregue com Sucesso" : "Falha na Entrega");
  Serial.println();
}

void setup() {
  // Iniciando Serial
  Serial.begin(115200);

  // Iniciando o GPS
  gpsSerial.begin(GPS_BAUD);

  // Iniciando o BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("Falha ao inicializar o sensor BMP280!");
  }

  // Define o dispositivo como Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Inicia a rede ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao inicializar ESP-NOW");
    return;
  }

  // Registro de status de transmissão do pacote
  esp_now_register_send_cb(OnDataSent);

  // Registrando o par
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Adicionando o par
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Falha ao adicionar peer");
    return;
  }
}

// Criação da função que exibe os dados enviados
void exibirDadosEnviados(struct_message dados){
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
}

// Criação da Função que realiza as leituras dos dados
void getReadings(){
  // Adicionando o ID do Nó na mensagem da estrutura
  BMP280GPSReadings.node = NODE;
  
  // Atualiza o contador
  BMP280GPSReadings.contador = contador;

  // Coletando os dados do BMP280
  BMP280GPSReadings.temp = bmp.readTemperature();
  BMP280GPSReadings.pres = bmp.readPressure() / 100.0F;
  BMP280GPSReadings.alt = bmp.readAltitude();

  // Verificando se os dados do GPS são válidos e salvando na mensagem a ser enviada
  if (gps.location.isValid()) {

    BMP280GPSReadings.gps_lat = gps.location.lat();
    BMP280GPSReadings.gps_long = gps.location.lng();
    BMP280GPSReadings.gps_speed = gps.speed.kmph();
    BMP280GPSReadings.gps_alt = gps.altitude.meters();
    BMP280GPSReadings.gps_sats = gps.satellites.value();
    BMP280GPSReadings.gps_hdop = gps.hdop.hdop();

    // Ajuste de horário para UTC-3 (Brasília)
    int hour = gps.time.hour() + TIMEZONEOFFSET;
    if (hour < 0) {
      hour += 24;
    } else if (hour > 23){
      hour -= 24;
    }
    snprintf(BMP280GPSReadings.gps_time, sizeof(BMP280GPSReadings.gps_time), "%02d:%02d:%02d", hour, gps.time.minute(), gps.time.second());
  
  } else {
    Serial.println("GPS Não disponível");
  }

  exibirDadosEnviados(BMP280GPSReadings);
}

void loop() {

  // Atualiza os dados do GPS sem bloquear o loop
  while (gpsSerial.available() > 0) {
    gps.encode(gpsSerial.read());
  }

  unsigned long tempoAtual = millis();

  if (tempoAtual - ultimoEnvio >= intervaloEnvio) {
    ultimoEnvio = tempoAtual;

    // Chama a função de leitura dos dados
    getReadings();

    // Envia os dados via ESP-NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &BMP280GPSReadings, sizeof(BMP280GPSReadings));

    if(result == ESP_OK){
      Serial.printf("Enviado com sucesso! => Contador: %d\n", contador);
      contador += 1;
    }
  }
}
