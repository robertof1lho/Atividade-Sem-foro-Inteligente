### Atividade Samáforo Inteligente 

Este projeto utiliza dois ESP32 para simular um sistema de semáforo que pode alternar entre o modo normal e o modo noturno, com base na luminosidade captada por um sensor LDR. O sistema é composto por um dispositivo **master**, que mede a luminosidade e decide o estado do semáforo, e um **slave**, que controla os LEDs com base nos comandos recebidos.

## Funcionamento Geral

1. O **master** lê o valor do sensor LDR para determinar a luminosidade ambiente.
2. Caso a luminosidade esteja abaixo de 10%, o **master** envia uma mensagem indicando que o modo noturno deve ser ativado.
3. O **slave** ajusta seu comportamento de acordo com a mensagem recebida:
   - No **modo normal**, os LEDs simulam um ciclo tradicional de semáforo (vermelho, verde, amarelo).
   - No **modo noturno**, o LED amarelo pisca continuamente.

## Vídeo

<div style="text-align: center;">
<img title="Vídeo semámafo" src="assets\ledBuiltInOn.jpg"  style="width: 70%;">
</div>

<br>

## Códigos do Projeto

### Código do Master

Este código é responsável por:
- Ler os dados do sensor LDR.
- Determinar o estado do modo noturno com base na luminosidade.
- Enviar mensagens ao **slave** indicando o estado do modo noturno.

```cpp
#include <Wire.h>
#include <esp_now.h>
#include <WiFi.h>
// Endereço MAC do ESP32 receptor (substitua pelo MAC do seu ESP32 receptor)
uint8_t broadcastAddress[] = { 0xFC, 0xB4, 0x67, 0x4F, 0x56, 0x20 };
// Estrutura de dados a ser enviada
typedef struct struct_message {
  bool modo_noturno;  // Estado do modo noturno
} struct_message;
struct_message mensagem;
// Pines para LEDs e sensor LDR
const int greenpin = 18;
const int yellowpin = 16;
const int redpin = 17;
const int ldrpin = 33;
// Callback para feedback de envio
void OnSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Status do envio: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sucesso" : "Falha");
}
void setup() {
  Serial.begin(115200);
  pinMode(redpin, OUTPUT);
  pinMode(greenpin, OUTPUT);
  pinMode(yellowpin, OUTPUT);
  pinMode(ldrpin, INPUT);
  WiFi.mode(WIFI_STA);
  // Inicializar ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Erro ao iniciar ESP-NOW");
    return;
  }
  esp_now_register_send_cb(OnSent);
  // Registrar o receptor
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Falha ao adicionar o par");
    return;
  }
}
void ligarled(int cor, int tempo) {
  digitalWrite(cor, HIGH);
  delay(tempo * 1000);
  digitalWrite(cor, LOW);
}
void loop() {
  int ldrValue = analogRead(ldrpin);
  // Converte o valor para uma escala de luminosidade (0 a 100%)
  float luminosidade = map(ldrValue, 0, 4095, 0, 100);
  // Exibe os valores no Serial Monitor
  Serial.print("LDR Value: ");
  Serial.print(ldrValue);
  Serial.print(" - Luminosidade (%): ");
  Serial.println(luminosidade);
  // Define o estado do modo noturno
  if (luminosidade < 10) {
    mensagem.modo_noturno = true;  // Ativar modo noturno
    Serial.println("Modo noturno ativado.");
    // Pisca o LED amarelo no master para indicar o estado
    digitalWrite(yellowpin, HIGH);
    delay(500);
    digitalWrite(yellowpin, LOW);
    delay(500);
  } else {
    mensagem.modo_noturno = false;  // Desativar modo noturno
    Serial.println("Modo noturno desativado.");
    // Ciclo normal do semáforo no master
    ligarled(redpin, 6);
    ligarled(greenpin, 4);
    ligarled(yellowpin, 2);
  }
  // Envia o estado do modo noturno para o receptor
  esp_now_send(broadcastAddress, (uint8_t *)&mensagem, sizeof(mensagem));
  delay(1000);  // Delay entre os envios
}
```

### Código do Slave

Este código é responsável por:
- Receber as mensagens do **master**.
- Alternar entre o ciclo de semáforo normal e o modo noturno com base nas mensagens recebidas.

```cpp
  #include <esp_now.h>
  #include <WiFi.h>

  const int greenpin = 17;
  const int yellowpin = 18;
  const int redpin = 19;

  // Estrutura de dados a ser recebida
  typedef struct struct_message {
    bool modo_noturno; // Indica se o modo noturno está ativado
  } struct_message;

  struct_message mensagem;

  // Endereço MAC do master
  uint8_t masterMAC[] = { 0xE0, 0x5A, 0x1B, 0x6C, 0x0F, 0x60 };

  bool modoNoturno = false; // Variável para armazenar o estado do modo noturno

  void OnDataRecv(const esp_now_recv_info *info, const uint8_t *data, int len) {
    char macStr[18];
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X",
            info->src_addr[0], info->src_addr[1], info->src_addr[2],
            info->src_addr[3], info->src_addr[4], info->src_addr[5]);

    Serial.print("Message received from: ");
    Serial.println(macStr);

    // Verifica se a mensagem veio do master
    if (memcmp(info->src_addr, masterMAC, 6) == 0) {
      Serial.println("Message is from the master device.");
      memcpy(&mensagem, data, sizeof(mensagem)); // Copia os dados recebidos para a estrutura

      // Atualiza o estado do modo noturno
      modoNoturno = mensagem.modo_noturno;
      Serial.print("Modo noturno: ");
      Serial.println(modoNoturno ? "Ativado" : "Desativado");
    } else {
      Serial.println("Message is from an unknown device.");
    }
  }

  void setup() {
    pinMode(redpin, OUTPUT);
    pinMode(greenpin, OUTPUT);
    pinMode(yellowpin, OUTPUT);

    Serial.begin(115200);

    // Inicializa WiFi no modo Station
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    // Inicializa ESP-NOW
    if (esp_now_init() != ESP_OK) {
      Serial.println("Error initializing ESP-NOW");
      return;
    }

    // Registra o callback de recebimento
    esp_now_register_recv_cb(OnDataRecv);
    Serial.println("ESP-NOW initialized and ready to receive messages.");
  }

  void ligarled(int cor, int tempo) {
    digitalWrite(cor, HIGH);
    delay(tempo * 1000);
    digitalWrite(cor, LOW);
  }

  void piscarLed(int cor, int tempo) {
    digitalWrite(cor, HIGH);
    delay(tempo * 500); // Liga o LED por metade do tempo
    digitalWrite(cor, LOW);
    delay(tempo * 500); // Desliga o LED pelo restante do tempo
  }

  void loop() {
    if (modoNoturno) {
      // Modo noturno: pisca o LED amarelo continuamente
      piscarLed(yellowpin, 1); // Pisca o LED amarelo a cada 1 segundo
    } else {
      // Ciclo normal do semáforo
      ligarled(redpin, 6);   // LED vermelho por 6 segundos
      ligarled(greenpin, 4); // LED verde por 4 segundos
      ligarled(yellowpin, 2); // LED amarelo por 2 segundos
    }
  }

```

Com esses códigos, o sistema estará funcional e responsivo para alternar entre os modos normal e noturno.