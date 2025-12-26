#include <Arduino.h>
#include <TFT_eSPI.h>
#include <cstdint> // Necessário para reconhecer uint16_t
#include <CAN.h>   // Inclui a biblioteca CAN Bus
#include <PCF8574.h> // Inclui a biblioteca PCF8574
#include "../include/Meu_logo_III.h" // Inclui o arquivo do seu logo

// --- MAPEAMENTO DE PINOS ESP32-2432S028 EXTERNOS ---
// Botões agora gerenciados pelo PCF8574 via I2C (Pinos 21 SDA, 22 SCL)
const int pinTrincos = 17;                   // Sensores de porta em série (GND para fechar)
const int pinSensorCabina = 25;              // Sensor magnético único na cabina
const int motorSobe = 4;                     // Acionamento subida
const int motorDesce = 16;                   // Acionamento descida
const int pinFreioBobina = 23;               // Pino para a bobina do freio

// --- CONFIGURAÇÃO PCF8574 ---
// Substitua 0x20 pelo endereço real do seu chip (verifique no módulo)
// SDA (21), SCL (22)
PCF8574 pcfBotoes(0x20, 21, 22); 

TFT_eSPI tft = TFT_eSPI();

// --- CONFIGURAÇÃO DE BRILHO (PWM) ---
#define TFT_BL_PIN 21
#define LEDC_CHANNEL_0 0
#define LEDC_RESOLUTION 8 // Resolução de 8 bits (0 a 255)
#define LEDC_FREQUENCY 5000

// --- CONFIGURAÇÃO CAN BUS (Pinos seguros no CN1/P3) ---
const int canTxPin = 22; // TX no CN1
const int canRxPin = 35; // RX no CN1/P3 (Pino 35 é exclusivo de entrada)

// --- VARIÁVEIS DE CONTROLE ---
int andarAtual = 1;
int andarDestino = 1;
bool emMovimento = false;
bool subindo = false;
bool detectouPrimeiroIman = false; 
bool sensorUltimoEstado = HIGH;

// --- VARIÁVEIS DE ANIMAÇÃO E ESTADO ---
unsigned long millisPiscar = 0;
unsigned long millisSeta = 0;
bool mostrarNumero = true;
int animSetaY = 0;

// Protótipos de funções
void desenharBackground();
void pararElevador();
void setupCan();
void enviarStatusAndar(int andar);
void controlarMovimento(bool subir, bool descer, bool frear);


void desenharBackground() {
    tft.pushImage(0, 0, 320, 240, (uint16_t*)Meu_logo_III); 
}

// Funcao auxiliar para controlar motores e freio com temporizacao
void controlarMovimento(bool subir, bool descer, bool frear) {
    if (frear) {
        // Lógica de PARADA (Desliga motor -> Atraso -> Liga freio)
        digitalWrite(motorSobe, LOW);
        digitalWrite(motorDesce, LOW);
        delay(50); // Atraso para o motor parar de receber corrente
        digitalWrite(pinFreioBobina, LOW); // Liga o freio (LOW = bobina desligada/freio ativado)
    } else {
        // Lógica de PARTIDA (Desliga freio -> Atraso -> Liga motor)
        digitalWrite(pinFreioBobina, HIGH); // Desliga o freio (HIGH = bobina ligada/freio liberado)
        delay(50); // Atraso para o freio abrir completamente
        digitalWrite(motorSobe, subir ? HIGH : LOW);
        digitalWrite(motorDesce, descer ? HIGH : LOW);
    }
}

void setup() {
    Serial.begin(115200);
    
    tft.init();
    tft.setRotation(1);
    tft.invertDisplay(true); 
    tft.setSwapBytes(true); 
    
    ledcSetup(LEDC_CHANNEL_0, LEDC_FREQUENCY, LEDC_RESOLUTION);
    ledcAttachPin(TFT_BL_PIN, LEDC_CHANNEL_0);
    ledcWrite(LEDC_CHANNEL_0, 150); // Brilho ajustado para 150/255
    
    setupCan(); // Inicializa o CAN Bus

    // Inicializa o PCF8574 para os botoes
    pcfBotoes.begin(); 
    for(int i=0; i < 8; i++) {
      pcfBotoes.pinMode(i, INPUT); 
    }

    desenharBackground(); 

    // Configuracao dos demais pinos do ESP32
    pinMode(pinTrincos, INPUT_PULLUP);
    pinMode(pinSensorCabina, INPUT_PULLUP);
    pinMode(motorSobe, OUTPUT);
    pinMode(motorDesce, OUTPUT);
    pinMode(pinFreioBobina, OUTPUT); // Configura o pino do freio

    // Garante que tudo comece parado e freiado
    controlarMovimento(false, false, true); // Usa a nova funcao para parar e frear com segurança

    tft.setTextColor(TFT_WHITE); 
    tft.drawCentreString("OSIRIS ELETRONICA", 160, 10, 2);
    tft.setTextColor(TFT_WHITE); 
    tft.drawCentreString("MONTA-PRATO 2026", 160, 27, 2);
}

void setupCan() {
  CAN.setPins(canRxPin, canTxPin); // Define os pinos CAN personalizados (22/35)
  if (!CAN.begin(125E3)) { // Inicializa o CAN bus a 125 kbps
    Serial.println("Erro ao iniciar CAN bus!");
    tft.drawCentreString("CAN ERROR", 160, 50, 2);
    while (true);
  }
}

void enviarStatusAndar(int andar) {
  CAN.beginPacket(0x100); // ID da mensagem 0x100
  CAN.write(andar);       // Envia o número do andar
  CAN.endPacket();
  Serial.print("DEBUG: Enviado andar ");
  Serial.println(andar);
}


void pararElevador() {
    controlarMovimento(false, false, true); // Parar e Frear com seguranca
    
    emMovimento = false;
    detectouPrimeiroIman = false;
    desenharBackground(); 
    enviarStatusAndar(andarAtual); // Envia o andar atualizado via CAN
}

void loop() {
    unsigned long agora = millis();

    if (digitalRead(pinTrincos) == HIGH) {
        if (emMovimento) {
            pararElevador();
        }
        tft.setTextColor(TFT_RED, TFT_BLACK); 
        tft.drawCentreString("PORTA ABERTA - BLOQUEADO", 160, 210, 2);
        return; 
    } else {
         tft.fillRect(0, 210, 320, 25, TFT_BLACK); 
    }

    // 2. LÓGICA QUANDO PARADO
    if (!emMovimento) {
        if (agora - millisPiscar >= 800) {
            millisPiscar = agora;
            mostrarNumero = !mostrarNumero;
            if (mostrarNumero) {
                tft.setTextColor(TFT_GREEN); 
                tft.setTextSize(12);
                tft.drawNumber(andarAtual, 125, 60);
            } else {
                tft.pushImage(125, 60, 100, 100, (uint16_t*)Meu_logo_III); 
            }
        }

        // LEITURA DOS BOTÕES DO PCF8574 (CORRIGIDA)
        PCF8574::DigitalInput pinValues = pcfBotoes.digitalReadAll();
        uint8_t leituraPCF = pinValues.p0 | (pinValues.p1 << 1) | (pinValues.p2 << 2) | 
                             (pinValues.p3 << 3) | (pinValues.p4 << 4) | 
                             (pinValues.p5 << 5) | (pinValues.p6 << 6) | (pinValues.p7 << 7);

        for (int i = 0; i < 3; i++) { // Verifica P0, P1 e P2 (Andares 1, 2, 3)
            if (!(leituraPCF & (1 << i))) { // Botão pressionado (LOW)
                int desejado = i + 1;
                if (desejado != andarAtual) {
                    andarDestino = desejado;
                    subindo = (andarDestino > andarAtual);
                    emMovimento = true;
                    detectouPrimeiroIman = false;
                    desenharBackground(); 
                    tft.setTextColor(TFT_WHITE);
                    tft.drawCentreString("EM MOVIMENTO", 160, 10, 2);
                }
            }
        }
    }

    // 3. LÓGICA EM MOVIMENTO (Otimizada)
    if (emMovimento) {
        // Aciona motores e libera freio com a nova funcao
        controlarMovimento(subindo, !subindo, false); 

        tft.setTextColor(TFT_YELLOW);
        tft.setTextSize(12);
        tft.drawNumber(andarAtual, 125, 60);

        // Animação das setas otimizada com fillRect
        if (agora - millisSeta >= 150) {
            millisSeta = agora;
            tft.fillRect(10, 50, 60, 140, TFT_BLACK); // Apaga a area da seta
            tft.setTextColor(TFT_CYAN);
            tft.setTextSize(5);

            animSetaY = (animSetaY + (subindo ? -15 : 15));
            if (abs(animSetaY) > 45) animSetaY = 0;

            tft.drawChar(subindo ? '^' : 'v', 20, 90 + animSetaY);
            tft.drawChar(subindo ? '^' : 'v', 20, 130 + animSetaY);
        }

        // Leitura do Sensor Magnético
        bool leituraSensor = (digitalRead(pinSensorCabina) == LOW);
        if (leituraSensor && sensorUltimoEstado == HIGH) {
            if (!detectouPrimeiroIman) {
                detectouPrimeiroIman = true;
            } else {
                if (subindo) andarAtual++; else andarAtual--;
                detectouPrimeiroIman = false; 

                if (andarAtual == andarDestino) {
                    pararElevador(); // Chama a função que envia o status via CAN
                } else {
                    enviarStatusAndar(andarAtual); // Envia o andar intermediário via CAN
                }
            }
            delay(200); 
        }
        sensorUltimoEstado = leituraSensor;
    }
}
