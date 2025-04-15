/*********************************************************
* Autor: Gabriel Sales de Jesus
* Trabalho de Conclusão de Curso em Engenharia Eletrônica
**********************************************************/

/******************** Bibliotecas ********************/
#include <esp_now.h>
#include <WiFi.h>

/********* Definições de Objetos e Variáveis ********/
#define TOTAL_NODES 9  // Número total de nós sensores
// #define TEMPO_ESPERA 2000 // Espera 2s pela resposta
#define TEMPO_ESPERA 500 // Espera 500ms pela resposta

// Lista de endereços MAC dos nós sensores
uint8_t sensorMACs[TOTAL_NODES][6] = {
    {0x10, 0x52, 0x1C, 0x69, 0x6F, 0xC8}, // Nó 2
    {0x10, 0x52, 0x1C, 0x69, 0x7D, 0x74}, // Nó 3 
    {0x7C, 0x9E, 0xBD, 0xE4, 0x25, 0x50}, // Nó 4
    {0xB8, 0xF0, 0x09, 0xCD, 0xFA, 0x0C}, // Nó 5
    {0x3C, 0x61, 0x05, 0x14, 0x46, 0x0C}, // Nó 6
    {0x7C, 0x9E, 0xBD, 0xE3, 0xAB, 0x78}, // Nó 7
    {0x64, 0xB7, 0x08, 0xCA, 0x5C, 0x84}, // Nó 8
    {0xB8, 0xF0, 0x09, 0xCD, 0xE4, 0x20}, // Nó 9
    {0x7C, 0x9E, 0xBD, 0xE3, 0x34, 0xA0}  // Nó 10
};

int cont_msg_rec[TOTAL_NODES] = {0, 0, 0, 0, 0, 0, 0, 0, 0};

// Estrutura dos dados enviados e recebidos
typedef struct struct_message {
    char id_node;
} struct_message;

struct_message send;
struct_message receive;

esp_now_peer_info_t peerInfo;

// Índice do nó atual
int currentNode = 0;
bool received = false;
/*****************************************************/

/*********** Funções de Callback ************/
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Mensagem enviada!" : "Falha no envio!");
}

void onDataRecv(const uint8_t *mac_addr, const uint8_t *receivedData, int len) {
    memcpy(&receive, receivedData, sizeof(receive));
    Serial.print("Recebido do nó: ");
    Serial.println(receive.id_node);
    received = true;
}
/*****************************************************/

void setup() {
    Serial.begin(115200);
    WiFi.mode(WIFI_STA);
    
    if (esp_now_init() != ESP_OK) {
        Serial.println("Erro ao inicializar ESP-NOW");
        return;
    }

    esp_now_register_recv_cb(onDataRecv);
    esp_now_register_send_cb(onDataSent);
}

void loop() {
    for (currentNode = 0; currentNode < TOTAL_NODES; currentNode++) {
        Serial.print("Enviando solicitação para nó: ");
        Serial.println(currentNode + 2);
        
        // Adiciona o nó atual como peer
        memcpy(peerInfo.peer_addr, sensorMACs[currentNode], 6);
        peerInfo.channel = 0;
        peerInfo.encrypt = false;

        if (esp_now_add_peer(&peerInfo) != ESP_OK) {
            Serial.println("Falha ao adicionar peer!");
            continue;
        }
        
        // Envia a solicitação
        send.id_node = 'S'; // Indica que é uma solicitação
        received = false;
        esp_now_send(sensorMACs[currentNode], (uint8_t *)&send, sizeof(send));
        
        // Aguarda resposta por até TEMPO_ESPERA segundos
        unsigned long startTime = millis();
        while (!received && millis() - startTime < TEMPO_ESPERA) {
            // delay(10);
            yield(); // Permite que o ESP execute tarefas de fundo sem ficar totalmente preso no loop e nem travar o código como no uso do delay
        }
        
        if (!received) {
            Serial.println("Nenhuma resposta recebida!");
            Serial.println(cont_msg_rec[currentNode]);
        } else{
          Serial.println(cont_msg_rec[currentNode]+= 1);
        }
        
        // Remove o peer para evitar problemas de conexão
        esp_now_del_peer(sensorMACs[currentNode]);
        
        // Aguarda um curto período antes de continuar
        delay(1000);
    }
}
