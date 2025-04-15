/*******************************************************
* Autor: Gabriel Sales de Jesus
* Trabalho de Conclusão de Curso em Engenharia Eletrônica
********************************************************/


// Bibliotecas
#include <WiFi.h>
#include <esp_wifi.h>

// Declaração de Funções
// Função para ler o Endereço MAC
void readMacAddress(){ 
  uint8_t baseMac[6];
  esp_err_t resposta = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (resposta == ESP_OK){
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
    baseMac[0], baseMac[1], baseMac[2],
    baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Falha ao ler o Endereço MAC");
  }
}


void setup() {
  // Iniciando porta serial
  Serial.begin(115200);

  // Definindo e Inicializando o ESP como modo estação
  WiFi.mode(WIFI_STA);
  WiFi.begin();

  // Printando e chamando função de leitura do MAC
  Serial.print("Endereço MAC do ESP32: ");
  readMacAddress();
}

void loop() {

}
