#include <Adafruit_NeoPixel.h>
#include <ESP32_CAN.h>
#include <WiFi.h>
#include <WiFiAP.h>
#include <WebServer.h>
#include <ESP2SOTA.h>


const char* ssid = "ASSI_LART_T24";
const char* password = "T24E_aut";
//WebServer server(80);

#define PIN 16           // Pino controlo LEDS
#define NUM_LEDS 16     // Número de LEDS
#define QUEUE_LENGTH 5
#define QUEUE_ITEM_SIZE sizeof(int)


int ADC_PIN = 12;

QueueHandle_t xQueue;
TaskHandle_t LER_ADC;


hw_timer_t *timer = NULL;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);
void colorWipe(uint32_t color, int wait);


portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

TWAI_Interface CAN1(1000, 21, 22); // argument 1 - BaudRate,  argument 2 - CAN_TX PIN,  argument 3 - CAN_RX PIN
int BRAKE  = 5; 
volatile int brake = 0;

volatile int flag = 0;

unsigned long timeout = 0;
unsigned long send = 0;





 

void setup() {

  pinMode(BRAKE,INPUT);
  strip.show();
  strip.clear();

  
  Serial.begin(115200);
  timeout = millis();

  /***        --- Initiate access point and configure static ip addresss as 06.14.22.24 ---         ***/
/*
  WiFi.mode(WIFI_AP);  
  WiFi.softAP(ssid, password);
  IPAddress IP = IPAddress (06, 14, 22, 24);
  IPAddress NMask = IPAddress (255, 255, 255, 0);
  IPAddress myIP = WiFi.softAPIP();
  WiFi.softAPConfig(IP, IP, NMask);
  ESP2SOTA.begin(&server);
  server.begin();

  server.on("/display", HTTP_GET, []() {
    String message = "Estado da Missao: " + String(digitalRead(BRAKE));
    server.sendHeader("Connection", "close");
    server.send(200, "text/plain", message);
  });*/

}


void loop() {
 // server.handleClient();
  // put your main code here, to run repeatedly:
  if(digitalRead(BRAKE))
  {
    brake = 0;
    colorWipe(strip.Color(255, 0, 0), 0);

  }
  else
  {
    colorWipe(strip.Color(0, 0, 0), 0);
  }
 




    if(millis() >= send + 200)
    {
      volatile float adc_value = analogRead(ADC_PIN);
      volatile float voltage = 6*adc_value/4095;
      volatile float pressao = voltage *  140 / 3.9998;
      Serial.println(voltage);
      Serial.println(pressao);
      CAN1.TXpacketBegin(0x253,0);
      CAN1.TXpacketLoad(((uint8_t)pressao >> 8) && 0xFF);
      CAN1.TXpacketLoad((uint8_t)pressao && 0xFF);
      send = millis();
    }
    


}



void colorWipe(uint32_t color, int wait) {
  for(int i = 0; i < strip.numPixels(); i++) {
    strip.setPixelColor(i, color);
    strip.show();
    delay(wait);
  }
}



