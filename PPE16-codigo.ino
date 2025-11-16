// CÓDIGO Kit Câmera de Vigilância
// AUTOR: MakerHero

// Define o modelo da câmera ANTES de incluir as bibliotecas
#define CAMERA_MODEL_AI_THINKER

// Bibliotecas utilizadas no código
#include "camera_pins.h"
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"
#include "esp_camera.h"
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>
#include <Wire.h>

//--- DECLARAÇÃO DE VARIÁVEIS E CONSTANTES:

String local_hwaddr; // Endereço WiFi local - hardware 
String local_swaddr; // Endereço WiFi local - software 

//--- Informações da rede WiFi - COLOQUE OS DADOS DE WIFI AQUI!
const char* ssid = "";      // Nome do Wifi
const char* password = "";  //Senha do Wifi

//--- Inicialização do Telegram BOT (Robô Chat Telegram)
//--> Declara as informações de identificação do Robô do Telegram: COLOQUE OS DADOS DO CHAT ROBÔ DO TELEGRAM AQUI!
String chatId = "";   // ID do Chat Telegram
String BOTtoken = ""; //Token de identificação do Robô do Chat Telegram
 
bool sendPhoto = false;      //inicializa a variável de enviar foto para desativá-la
 
bool FlashState = false;     //inicializa a variável do flash para desativá-la
bool MotionDetected = false; //inicializa a variável da detecção de movimento para desativá-la
bool MotionState = false;    //inicializa a variável do estado da detecção de movimento para desativá-la

int buttonState;



int botRequestDelay = 1000; // tempo para escanear mensagens
long lastTimeBotRan;        // declara o tempo entre a última mensagem enviada e a nova mensagem

WiFiClientSecure clientTCP; // declara a função que conecta a placa Esp32Cam com a rede Wifi

UniversalTelegramBot bot(BOTtoken, clientTCP); // declara a função que conecta o robô do Telegram e a placa Esp32Cam

//---Definição dos pinos de conexão entre os componentes e a placa:
// Os pinos da câmera AI-THINKER agora são definidos automaticamente em camera_pins.h
#define SENSOR_LED 12
#define FLASH_LED_PIN 4 

// Definição do pino que será conectado o Sensor de Movimento: 
#define MOTION_SENSOR 13 // MOTION_SENSOR PIN: GPIO 13

// Definição do pino que será conectado o Botão para tirar foto:
#define buttonPin 14 // define o pino que alocará a chave Push-button Mini Switch: GPIO 16


void handleNewMessages(int numNewMessages); //função para receber e enviar novas mensagens no Telegram
String sendPhotoTelegram();                 //função para enviar foto ao Telegram

// Função que indica quando o sensor de movimento detecta um sinal
static void IRAM_ATTR detectsMovement(void * arg) {
MotionDetected = true;

}

void startCameraServer(); //Função que inicializa o Servidor Web da Câmera Streaming

//-------FUNÇÃO PRINCIPAL-----------
void setup() 
{
      Serial.setDebugOutput(true);                //Configuração Serial para inicializar o Servidor Web da Câmera Streaming
      WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
      Serial.begin(115200);                       //Inicialização da comunicação Serial da placa Esp32Cam 
      delay(500);

      
      // pinMode(MOTION_SENSOR, INPUT);    // Declara o pino GPIO 13 ou IO13 referente ao Sensor de Movimento como ENTRADA   
      pinMode(SENSOR_LED, OUTPUT);      // Declara o pino GPIO 12 ou IO12 referente ao LED imbutio na placa esp32cam como SAÍDA
      // pinMode(buttonPin,INPUT_PULLUP);  // Declara o pino GPIO 16 ou IO16 referente a Chave Push-Button Mini Switch como ENTRADA sendo do tipo PULLUP
      pinMode(FLASH_LED_PIN, OUTPUT);   // Declara o pino GPIO 04 OU IO4 referente ao Flash da câmera
      
      digitalWrite(SENSOR_LED, LOW);    // Escreve na variável do LED o estado DESLIGADO (LOW)
      digitalWrite(FLASH_LED_PIN, LOW); // Escreve na variável do FLASH da câmera o estado DESLIGADO (LOW)
       
      
      delay(500);                       // Aguarda 0.5 segundos
      
      Serial.println("\nESP Cam using Telegram Bot"); // Escreve no monitor Serial 

      //Configurações para escrever os dados da rede Wifi no monitor serial
      WiFi.mode(WIFI_STA);
      Serial.println();
      Serial.print("Connecting to ");
      Serial.println(ssid);
      WiFi.begin(ssid, password);
       
      clientTCP.setCACert(TELEGRAM_CERTIFICATE_ROOT); // Informações para conexão com a API Telegram (api.telegram.org)

      //Configuração para mostrar o status da conexão com Wifi
      while (WiFi.status() != WL_CONNECTED) {
      Serial.print(".");
      delay(500);
      }
      Serial.println(" >> CONNECTED");
      startCameraServer(); //função que inicia o servidor Web da câmera Streaming
       
      Serial.print("ESP32-CAM IP Address: ");
      Serial.println(WiFi.localIP());
       
      //Configurações dos pinos da PLACA Esp32Cam:
      camera_config_t config;
      config.ledc_channel = LEDC_CHANNEL_0;
      config.ledc_timer = LEDC_TIMER_0;
      config.pin_d0 = Y2_GPIO_NUM;
      config.pin_d1 = Y3_GPIO_NUM;
      config.pin_d2 = Y4_GPIO_NUM;
      config.pin_d3 = Y5_GPIO_NUM;
      config.pin_d4 = Y6_GPIO_NUM;
      config.pin_d5 = Y7_GPIO_NUM;
      config.pin_d6 = Y8_GPIO_NUM;
      config.pin_d7 = Y9_GPIO_NUM;
      config.pin_xclk = XCLK_GPIO_NUM;
      config.pin_pclk = PCLK_GPIO_NUM;
      config.pin_vsync = VSYNC_GPIO_NUM;
      config.pin_href = HREF_GPIO_NUM;
      config.pin_sscb_sda = SIOD_GPIO_NUM;
      config.pin_sscb_scl = SIOC_GPIO_NUM;
      config.pin_pwdn = PWDN_GPIO_NUM;
      config.pin_reset = RESET_GPIO_NUM;
      config.xclk_freq_hz = 20000000;
      config.pixel_format = PIXFORMAT_JPEG;

       
      //Configuração das especificações de imagem
      if (psramFound()) {
      Serial.println("PSRAM detectada! Usando resolução SVGA");
      config.frame_size = FRAMESIZE_SVGA;
      config.jpeg_quality = 12; //0-63 lower number means higher quality
      config.fb_count = 1;
      } else {
      Serial.println("PSRAM não detectada. Usando resolução SVGA");
      config.frame_size = FRAMESIZE_SVGA;
      config.jpeg_quality = 12; //0-63 lower number means higher quality
      config.fb_count = 1;
      }

      // Diagnóstico de memória antes da inicialização
      Serial.println("\n=== Diagnóstico de Memória ===");
      Serial.printf("PSRAM Total: %d bytes\n", ESP.getPsramSize());
      Serial.printf("PSRAM Livre: %d bytes\n", ESP.getFreePsram());
      Serial.printf("Heap Total: %d bytes\n", ESP.getHeapSize());
      Serial.printf("Heap Livre: %d bytes\n", ESP.getFreeHeap());
      Serial.println("==============================\n");

      // Inicialização da câmera
      Serial.println("Inicializando câmera AI-THINKER...");
      esp_err_t err = esp_camera_init(&config);
      if (err != ESP_OK) {
      Serial.printf("\n*** ERRO DE INICIALIZAÇÃO DA CÂMERA ***\n");
      Serial.printf("Código de erro: 0x%x\n", err);

      // Diagnóstico detalhado baseado no código de erro
      if (err == 0x105) {
        Serial.println("ESP_ERR_NOT_FOUND: Câmera não detectada na I2C");
        Serial.println("Verifique:");
        Serial.println("  - Módulo da câmera está conectado corretamente");
        Serial.println("  - Pinos SIOD (GPIO26) e SIOC (GPIO27)");
      } else if (err == 0x106) {
        Serial.println("ESP_ERR_NOT_SUPPORTED: Sensor da câmera não suportado");
        Serial.println("Verifique:");
        Serial.println("  - Módulo da câmera é OV2640 ou OV3660");
        Serial.println("  - Cabo flat da câmera está bem conectado");
        Serial.println("  - Alimentação de 5V está estável (min 2A)");
        Serial.println("  - Tente desconectar e reconectar o módulo da câmera");
      } else if (err == 0x103) {
        Serial.println("ESP_ERR_INVALID_STATE: Estado inválido");
      } else {
        Serial.println("Erro desconhecido. Consulte documentação ESP-IDF");
      }

      Serial.println("\nReiniciando em 3 segundos...");
      delay(3000);
      ESP.restart();
      }

      // Detectar tipo do sensor
      sensor_t * s = esp_camera_sensor_get();
      if (s != NULL) {
        Serial.printf("Câmera inicializada com sucesso!\n");
        Serial.printf("ID do Sensor: 0x%02X\n", s->id.PID);
        if (s->id.PID == OV2640_PID) {
          Serial.println("Sensor detectado: OV2640");
        } else if (s->id.PID == OV3660_PID) {
          Serial.println("Sensor detectado: OV3660");
        } else {
          Serial.printf("Sensor desconhecido: 0x%02X\n", s->id.PID);
        }

        // Redução do tamanho do quadro da foto para uma taxa de quadros inicial mais alta
        s->set_framesize(s, FRAMESIZE_CIF); // UXGA|SXGA|XGA|SVGA|VGA|CIF|QVGA|HQVGA|QQVGA
      }
      err = gpio_install_isr_service(0);
      err = gpio_isr_handler_add(GPIO_NUM_13, & detectsMovement, (void *) 13);
      if (err != ESP_OK) {
      Serial.printf("handler add failed with error 0x%x \r\n", err);
      }
      err = gpio_set_intr_type(GPIO_NUM_13, GPIO_INTR_POSEDGE);
      if (err != ESP_OK) {
      Serial.printf("set intr type failed with error 0x%x \r\n", err);
      }
      
       
      Serial.printf("\n Digite '/start' para iniciar o Bot no Telegram\r\n");      
}

//------------- FUNÇÃO LOOP ------------
void loop() 
{
        
        
        if (sendPhoto)                      //laço de condição que verifica se foi acionada a foto 
        {         
        Serial.println("Preparing photo"); //escreve no monitor Serial
        sendPhotoTelegram();               //chamada para função de envio de foto
        sendPhoto = false;                 //desativa o envio de fotos para não enviar foto indevidas em seguida
        }
         
        

        //Função Debounce para envio de novas mensagens
        if (millis() > lastTimeBotRan + botRequestDelay) {
        int numNewMessages = bot.getUpdates(bot.last_message_received + 1);
        while (numNewMessages) {
        Serial.print("Message received : ");
        handleNewMessages(numNewMessages);
        numNewMessages = bot.getUpdates(bot.last_message_received + 1);
        }
        lastTimeBotRan = millis();
        }

        

        
}

//Função para receber e enviar dados ao Telegram 
String sendPhotoTelegram() 
                    {
                      const char* myDomain = "api.telegram.org";
                      String getAll = "";
                      String getBody = "";
                       
                      camera_fb_t * fb = NULL;
                       
                      if (FlashState == true) digitalWrite(FLASH_LED_PIN, HIGH); // FLASH ON
                       
                      delay(10);
                       
                      fb = esp_camera_fb_get();
                       
                      digitalWrite(FLASH_LED_PIN, LOW); // FLASH OFF
                       
                      if (!fb) {
                      Serial.println("Camera capture failed");
                      delay(200);
                      ESP.restart();
                      return "Camera capture failed";
                      }
                       
                      Serial.println("Connect to " + String(myDomain));
                       
                      if (clientTCP.connect(myDomain, 443)) {
                      Serial.println("Connection successful");
                       
                      String head = "--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"chat_id\"; \r\n\r\n" + chatId + "\r\n--RandomNerdTutorials\r\nContent-Disposition: form-data; name=\"photo\"; filename=\"esp32-cam.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
                      String tail = "\r\n--RandomNerdTutorials--\r\n";
                       
                      uint16_t imageLen = fb->len;
                      uint16_t extraLen = head.length() + tail.length();
                      uint16_t totalLen = imageLen + extraLen;
                       
                      clientTCP.println("POST /bot" + BOTtoken + "/sendPhoto HTTP/1.1");
                      clientTCP.println("Host: " + String(myDomain));
                      clientTCP.println("Content-Length: " + String(totalLen));
                      clientTCP.println("Content-Type: multipart/form-data; boundary=RandomNerdTutorials");
                      clientTCP.println();
                      clientTCP.print(head);
                       
                      uint8_t *fbBuf = fb->buf;
                      size_t fbLen = fb->len;
                      for (size_t n = 0; n < fbLen; n = n + 1024) {
                      if (n + 1024 < fbLen) {
                      clientTCP.write(fbBuf, 1024);
                      fbBuf += 1024;
                      }
                      else if
                      (fbLen % 1024 > 0) {
                      size_t remainder = fbLen % 1024;
                      clientTCP.write(fbBuf, remainder);
                      }
                      }
                       
                      clientTCP.print(tail);
                       
                      esp_camera_fb_return(fb);
                       
                      int waitTime = 10000; // timeout 10 seconds
                      long startTimer = millis();
                      boolean state = false;
                       
                      while ((startTimer + waitTime) > millis()) {
                      Serial.print(".");
                      delay(100);
                      while (clientTCP.available()) {
                      char c = clientTCP.read();
                      if (c == '\n') {
                      if (getAll.length() == 0) state = true;
                      getAll = "";
                      }
                      else if (c != '\r') {
                      getAll += String(c);
                      }
                      if (state == true) {
                      getBody += String(c);
                      }
                      startTimer = millis();
                      }
                      if (getBody.length() > 0) break;
                      }
                      clientTCP.stop();
                       
                      // Print Information
                      //Serial.println(getBody);
                      Serial.println("Photo Sent");
                      }
                      else {
                      getBody = "Connected to api.telegram.org failed.";
                      Serial.println("Connected to api.telegram.org failed.");
                      }
                      return getBody;
                    }


//Função para enviar e receber novas mensagens
void handleNewMessages(int numNewMessages) 
{
              for (int i = 0; i < numNewMessages; i++) 
              {
                
                  // Laço que verifica se o CHAT ID está correto
                  String chat_id = String(bot.messages[i].chat_id);
                  if (chat_id != chatId) 
                  {
                    bot.sendMessage(chat_id, "Unauthorized user", "");
                    continue;
                  }
                  
                  // Escreve no monitor Serial a mensagem recebida
                  String fromName = bot.messages[i].from_name;
                  String text = bot.messages[i].text;
                  Serial.println(numNewMessages + " From " + fromName + " >" + text + " request");
    
                  //Condição que verifica se comando "Flash" foi escrito pelo usuário no chat do Telegram 
                  if (text == "Flash") 
                  {
                    FlashState = !FlashState;
                    String welcome = "STATUS:\n";
                    Serial.print("FlashState = ");
                    if (FlashState == true) welcome += "Flash : LIGADO\n"; else welcome += "Flash : DESLIGADO\n";
                    if (FlashState == true) Serial.println("ON"); else Serial.println("OFF");
                    bot.sendMessage(chatId, welcome, "Markdown");
                  }
    
                     
                  //Condição que verifica se comando "Foto" foi escrito pelo usuário no chat do Telegram 
                  if(text == "Foto")
                  {
                    sendPhoto = true;  //envio da foto
                  }
                  
                  //Condição que verifica se comando "/start" foi escrito pelo usuário no chat do Telegram 
                  if (text == "/start") 
                  {
                    String welcome = "*Kit Câmera de Vigilância*\n\n"; //texto referente a mensagem de inicialização do Robô no Telegram   
                    welcome += "Digite *Foto*, para tirar uma nova foto\n";
                    welcome += "Digite *Flash*, para ligar ou desligar o flash\n";
                    welcome += "Digite */start*, para ver o status do flash\n";
                    welcome += "\nPara aumentar a qualidade ou mudar o efeito da foto, acesse o endereço de IP\n";
      
                    welcome += "\n>>>*STATUS FLASH<<<* \n";
                    if (FlashState == true) welcome += "LIGADO\n"; 
                    else welcome += "DESLIGADO \n";
                                 
                                      
                    bot.sendMessage(chatId, welcome, "Markdown");
                  }
              }
}
