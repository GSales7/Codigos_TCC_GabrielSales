/*********************************************************
* Autor: Gabriel Sales de Jesus
* Trabalho de Conclusão de Curso em Engenharia Eletrônica
**********************************************************/

/******************** Bibliotecas ********************/
#include <esp_now.h>
#include <WiFi.h>

/***************** Definições e Variáveis ***************/
const char ID = '2';  // ID do nó
const int canal = 0;
volatile bool flag_rec = false; // Volatile para evitar problemas na interrupção

// Endereço MAC do Nó Sink (destino)
uint8_t broadcastAddress1[] = {0x3C, 0x61, 0x05, 0x14, 0x57, 0x64};

/***************** Estruturas de Dados *****************/
// Estrutura para enviar e receber mensagens
typedef struct struct_message {
    char id_node;
} struct_message;

struct_message sendData;  // Dados a serem enviados
struct_message receiveData;  // Dados recebidos

/*********** Funções Auxiliares ************/

// Callback quando a mensagem é enviada
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.print("Status da transmissão: ");
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso" : "Falha");
}

// Callback quando a mensagem é recebida
void onDataRecv(const uint8_t *mac_addr, const uint8_t *incomingData, int len) {
    Serial.println("Mensagem recebida!");
    memcpy(&receiveData, incomingData, sizeof(receiveData));
    flag_rec = true;  // Marca que recebeu algo
}

// Função para configurar ESP-NOW
void setupESPNow() {
    WiFi.mode(WIFI_STA); // Define como Station
    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar ESP-NOW");
        return;
    }

    // Configura o peer (dispositivo de destino)
    esp_now_peer_info_t peerInfo = {};
    memcpy(peerInfo.peer_addr, broadcastAddress1, 6);
    peerInfo.channel = canal;
    peerInfo.encrypt = false;
    peerInfo.ifidx = WIFI_IF_STA;  // Corrige possível erro de interface

    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Erro ao adicionar peer!");
        return;
    }

    // Registra os callbacks
    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);
}

// Função para enviar mensagem
void sendMessage() {
    esp_err_t result = esp_now_send(broadcastAddress1, (uint8_t*)&sendData, sizeof(sendData));
    if (result == ESP_OK) {
        Serial.println("Mensagem enviada com sucesso!");
    } else {
        Serial.println("Erro ao enviar mensagem!");
    }
}

/******************** Setup e Loop ********************/

void setup() {
    Serial.begin(115200);
    sendData.id_node = ID; // Define o ID do nó para envio
    setupESPNow(); // Configura ESP-NOW
}

void loop() {
    if (flag_rec) {
        flag_rec = false;  // Reseta flag
        Serial.print("Enviando ID de resposta: ");
        Serial.println(sendData.id_node);
        sendMessage();
    }
    delay(10); // Pequeno delay para evitar loop excessivo
}
