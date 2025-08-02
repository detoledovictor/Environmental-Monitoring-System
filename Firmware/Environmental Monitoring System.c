#include <Arduino.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include <Wire.h>
#include <SPI.h>
#include <TFT_eSPI.h>
#include "RoboCore_MMA8452Q.h"
#include <Adafruit_BMP280.h>
#include <WiFi.h>
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <FastLED.h>

// Configurações do Display TFT
TFT_eSPI tft = TFT_eSPI(240, 240);

// Configurações do LED RGB
#define NUM_LEDS 1
#define DATA_PIN 19
#define LED_UPDATE_INTERVAL 50  // ms
CRGB leds[NUM_LEDS];

// Configurações Wi-Fi
const char *ssid = "[Your_SSID]";
const char *password = "[YOUR_PASSWORD]";

// Configurações NTP
WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", -3 * 3600);  // UTC-3 (Brasília)

// Arrays para dias da semana
String diasSemana[] = { "Dom", "Seg", "Ter", "Qua", "Qui", "Sex", "Sab" };

MMA8452Q accelerometer;
Adafruit_BMP280 bmp;

// Estrutura para dados do display
typedef struct {
  String line1;  // Data e hora
  String line2;  // Temperatura
  float accel_x;
  float accel_y;
  float accel_z;
  String line6;  // Status WiFi
} DisplayData;

// Fila para comunicação entre tarefas
QueueHandle_t displayQueue;
DisplayData currentDisplayData;  // Mantém o estado atual do display

SemaphoreHandle_t displayMutex = xSemaphoreCreateMutex();

// Protótipos das tarefas
void TaskBlink(void *pvParameters);
void TaskReadSensors(void *pvParameters);
void TaskWiFiConnect(void *pvParameters);
void TaskDateTime(void *pvParameters);
void TaskUpdateDisplay(void *pvParameters);
void TaskUpdateLED(void *pvParameters);

// Protótipos das funções de desenho
void desenharHorarioSegmentos(const String &timeStr);
void desenharData(const String &dateStr);
void desenharTempUmidade(const String &tempStr);
void desenharAcelerometro(float x, float y, float z);
void desenharIndicadorAcelerometro(float x, float y);
void desenharDigito7Seg(int digito, int x, int y);

void setup() {
  Serial.begin(115200);
  pinMode(2, OUTPUT);

  // Inicializa o LED RGB
  FastLED.addLeds<NEOPIXEL, DATA_PIN>(leds, NUM_LEDS);
  leds[0] = CRGB::Black;
  FastLED.show();

  // Inicializa I2C
  Wire.begin(21, 22);  // SDA, SCL

  // Inicializa o display TFT
  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  // Inicializa o acelerômetro
  if (!accelerometer.init()) {
    Serial.println("Falha na inicialização do acelerômetro");
    while (1)
      ;
  }

  // Inicializa o BMP280
  if (!bmp.begin(0x76)) {
    Serial.println("Falha na inicialização do BMP280!");
    while (1)
      ;
  }

  // Cria a fila para o display
  displayQueue = xQueueCreate(5, sizeof(DisplayData));

  // Inicializa a estrutura de dados do display
  currentDisplayData = DisplayData();

  // Cria as tarefas
  xTaskCreate(TaskBlink, "Blink", 2048, NULL, 1, NULL);
  xTaskCreate(TaskReadSensors, "Sensors", 4096, NULL, 2, NULL);
  xTaskCreate(TaskWiFiConnect, "WiFiConnect", 8192, NULL, 3, NULL);
  xTaskCreate(TaskDateTime, "DateTime", 4096, NULL, 2, NULL);
  xTaskCreate(TaskUpdateDisplay, "Display", 4096, NULL, 3, NULL);
  xTaskCreate(TaskUpdateLED, "LED", 2048, NULL, 2, NULL);
}

void loop() {
  // Nada aqui - tudo é tratado pelas tarefas
}

// Tarefa: Piscar LED onboard
void TaskBlink(void *pvParameters) {
  for (;;) {
    digitalWrite(2, !digitalRead(2));
    vTaskDelay(500 / portTICK_PERIOD_MS);
  }
}

// Tarefa: Leitura de sensores
void TaskReadSensors(void *pvParameters) {
  DisplayData data;

  for (;;) {
    // Lê acelerômetro
    accelerometer.read();
    data.accel_x = accelerometer.x;
    data.accel_y = accelerometer.y;
    data.accel_z = accelerometer.z;

    // Lê temperatura
    float temperature = bmp.readTemperature();
    data.line2 = String(temperature, 1);

    // Envia dados para a fila
    xQueueSend(displayQueue, &data, portMAX_DELAY);

    vTaskDelay(100 / portTICK_PERIOD_MS);
  }
}

// Tarefa: Conexão Wi-Fi
void TaskWiFiConnect(void *pvParameters) {
  DisplayData data;

  data.line6 = "Conectando WiFi...";
  xQueueSend(displayQueue, &data, portMAX_DELAY);

  WiFi.begin(ssid, password);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    vTaskDelay(500 / portTICK_PERIOD_MS);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    data.line6 = "WiFi OK";
    xQueueSend(displayQueue, &data, portMAX_DELAY);

    // Mantém a conexão ativa
    for (;;) {
      if (WiFi.status() != WL_CONNECTED) {
        WiFi.reconnect();
        data.line6 = "Reconectando...";
        xQueueSend(displayQueue, &data, portMAX_DELAY);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
      }
      vTaskDelay(1000 / portTICK_PERIOD_MS);
    }
  } else {
    data.line6 = "WiFi Falhou!";
    xQueueSend(displayQueue, &data, portMAX_DELAY);
    vTaskDelay(5000 / portTICK_PERIOD_MS);
    ESP.restart();
  }
}

// Tarefa: Data e Hora
void TaskDateTime(void *pvParameters) {
  DisplayData data;

  // Espera conexão Wi-Fi
  while (WiFi.status() != WL_CONNECTED) {
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }

  timeClient.begin();

  for (;;) {
    timeClient.update();

    // Formata data e hora
    String dayOfWeek = diasSemana[timeClient.getDay()];
    time_t epochTime = timeClient.getEpochTime();
    struct tm *ptm = gmtime(&epochTime);

    // Formata a hora (HH:MM)
    int hours = timeClient.getHours();
    int minutes = timeClient.getMinutes();
    String formattedTime = String(hours < 10 ? "0" : "") + String(hours) + ":" + String(minutes < 10 ? "0" : "") + String(minutes);

    // Formata a data (DD/MM/YYYY)
    String formattedDate = String(ptm->tm_mday) + "/" + String(ptm->tm_mon + 1) + "/" + String(ptm->tm_year + 1900);

    // Combina tudo
    data.line1 = formattedTime + "|" + dayOfWeek + "," + formattedDate;

    xQueueSend(displayQueue, &data, portMAX_DELAY);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
  }
}

// Tarefa: Atualização centralizada do display
void TaskUpdateDisplay(void *pvParameters) {
  DisplayData newData;
  unsigned long lastUpdate = 0;
  const unsigned long updateInterval = 10;  // ms

  for (;;) {
    unsigned long now = millis();

    // Verifica se há novos dados na fila
    if (xQueueReceive(displayQueue, &newData, 0) == pdTRUE) {
      // Atualiza apenas os campos que receberam novos dados
      if (!newData.line1.isEmpty()) currentDisplayData.line1 = newData.line1;
      if (!newData.line2.isEmpty()) currentDisplayData.line2 = newData.line2;
      if (newData.accel_x != 0 || newData.accel_y != 0 || newData.accel_z != 0) {
        currentDisplayData.accel_x = newData.accel_x;
        currentDisplayData.accel_y = newData.accel_y;
        currentDisplayData.accel_z = newData.accel_z;
      }
      if (!newData.line6.isEmpty()) currentDisplayData.line6 = newData.line6;
    }

    // Atualiza o display apenas no intervalo definido
    if (now - lastUpdate >= updateInterval) {
      lastUpdate = now;

      if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
        // Limpa apenas a área necessária em vez de toda a tela
        tft.fillRect(0, 0, 240, 240, TFT_BLACK);

        // Extrai partes da linha 1 (formato: HH:MM|Dia,DD/MM/YYYY)
        int separatorPos = currentDisplayData.line1.indexOf('|');
        String timeStr = currentDisplayData.line1.substring(0, separatorPos);
        String dateStr = currentDisplayData.line1.substring(separatorPos + 1);

        // Desenha os componentes
        desenharHorarioSegmentos(timeStr);
        desenharData(dateStr);

        if (!currentDisplayData.line2.isEmpty()) {
          desenharTempUmidade(currentDisplayData.line2);
        }

        desenharAcelerometro(currentDisplayData.accel_x, currentDisplayData.accel_y, currentDisplayData.accel_z);
        desenharIndicadorAcelerometro(currentDisplayData.accel_x, currentDisplayData.accel_y);

        xSemaphoreGive(displayMutex);
      }
    }
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}

// Tarefa: Atualização do LED RGB
void TaskUpdateLED(void *pvParameters) {
  String lastWiFiStatus = "";

  for (;;) {
    // Verifica o status atual do WiFi
    String currentStatus = currentDisplayData.line6;

    // Só atualiza o LED se o status mudou
    if (currentStatus != lastWiFiStatus) {
      lastWiFiStatus = currentStatus;

      if (currentStatus == "WiFi OK") {
        // Verde sólido quando conectado
        leds[0] = CRGB::Green;
        FastLED.show();
      } else if (currentStatus.startsWith("Conectando") || currentStatus.startsWith("Reconectando")) {
        leds[0] = CRGB::Yellow;
        FastLED.show();
      } else {
        leds[0] = CRGB::Red;
        FastLED.show();
      }
    }

    vTaskDelay(LED_UPDATE_INTERVAL / portTICK_PERIOD_MS);
  }
}

// Funções de desenho
void desenharHorarioSegmentos(const String &timeStr) {
  int startX = 120 - 80;
  int startY = 35;

  // Extrai horas e minutos da string
  int horas = timeStr.substring(0, 2).toInt();
  int minutos = timeStr.substring(3, 5).toInt();

  // Desenha cada dígito separadamente
  desenharDigito7Seg(horas / 10, startX, startY);
  desenharDigito7Seg(horas % 10, startX + 40, startY);

  // Dois pontos
  if ((millis() / 500) % 2 == 0) {
    tft.fillRect(startX + 78, startY + 15, 4, 4, TFT_CYAN);
    tft.fillRect(startX + 78, startY + 35, 4, 4, TFT_CYAN);
  }

  desenharDigito7Seg(minutos / 10, startX + 90, startY);
  desenharDigito7Seg(minutos % 10, startX + 130, startY);
}

void desenharData(const String &dateStr) {
  int centroX = 120;
  int y = 90;

  // Configurações de texto
  tft.setTextSize(2);
  tft.setTextColor(TFT_CYAN, TFT_BLACK);

  // Cálculo de largura aproximada
  int larguraTexto = dateStr.length() * 12;

  // Posiciona cursor centralizado e imprime
  tft.setCursor(centroX - larguraTexto / 2, y);
  tft.print(dateStr);
}

void desenharTempUmidade(const String &tempStr) {
  int y = 130;

  // Temperatura
  tft.setTextColor(TFT_WHITE, TFT_BLACK);
  tft.setTextSize(1);
  tft.setCursor(20, y);
  tft.print("Temp: ");
  tft.setTextColor(TFT_RED, TFT_BLACK);
  tft.setTextSize(2);
  tft.print(tempStr);
  tft.print("C");
}

void desenharAcelerometro(float x, float y, float z) {
  int yPos = 165;

  tft.setTextColor(TFT_GREEN, TFT_BLACK);
  tft.setTextSize(1);

  // X
  tft.setCursor(20, yPos);
  tft.print("X: ");
  tft.print(x, 2);

  // Y
  tft.setCursor(90, yPos);
  tft.print("Y: ");
  tft.print(y, 2);

  // Z
  tft.setCursor(160, yPos);
  tft.print("Z: ");
  tft.print(z, 2);
}

void desenharIndicadorAcelerometro(float x, float y) {
  int centroX = 120;
  int centroY = 200;
  int raio = 25;

  // Círculo de referência
  tft.drawCircle(centroX, centroY, raio, TFT_DARKGREY);

  // Ponto representando a inclinação
  int pontoX = centroX + (x * raio * 10);
  int pontoY = centroY + (y * raio * 10);

  // Limita dentro do círculo
  float distancia = sqrt(pow(pontoX - centroX, 2) + pow(pontoY - centroY, 2));
  if (distancia > raio) {
    float fator = raio / distancia;
    pontoX = centroX + (pontoX - centroX) * fator;
    pontoY = centroY + (pontoY - centroY) * fator;
  }

  // Desenha o ponto
  tft.fillCircle(pontoX, pontoY, 3, TFT_GREEN);
  tft.fillCircle(centroX, centroY, 2, TFT_WHITE);
}

void desenharDigito7Seg(int digito, int x, int y) {
  bool segmentos[10][7] = {
    { 1, 1, 1, 1, 1, 1, 0 },  // 0
    { 0, 1, 1, 0, 0, 0, 0 },  // 1
    { 1, 1, 0, 1, 1, 0, 1 },  // 2
    { 1, 1, 1, 1, 0, 0, 1 },  // 3
    { 0, 1, 1, 0, 0, 1, 1 },  // 4
    { 1, 0, 1, 1, 0, 1, 1 },  // 5
    { 1, 0, 1, 1, 1, 1, 1 },  // 6
    { 1, 1, 1, 0, 0, 0, 0 },  // 7
    { 1, 1, 1, 1, 1, 1, 1 },  // 8
    { 1, 1, 1, 1, 0, 1, 1 }   // 9
  };

  uint16_t cor = TFT_CYAN;
  uint16_t corOff = TFT_BLACK;

  // Desenha os segmentos
  tft.fillRect(x + 2, y, 26, 4, segmentos[digito][0] ? cor : corOff);        // a
  tft.fillRect(x + 26, y + 2, 4, 22, segmentos[digito][1] ? cor : corOff);   // b
  tft.fillRect(x + 26, y + 26, 4, 22, segmentos[digito][2] ? cor : corOff);  // c
  tft.fillRect(x + 2, y + 46, 26, 4, segmentos[digito][3] ? cor : corOff);   // d
  tft.fillRect(x, y + 26, 4, 22, segmentos[digito][4] ? cor : corOff);       // e
  tft.fillRect(x, y + 2, 4, 22, segmentos[digito][5] ? cor : corOff);        // f
  tft.fillRect(x + 2, y + 23, 26, 4, segmentos[digito][6] ? cor : corOff);   // g
}