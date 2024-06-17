#include <Adafruit_NeoPixel.h>
#include <ESP32_CAN.h>
#include <BluetoothSerial.h>




#define PIN 16           // Pino controlo LEDS
#define NUM_LEDS 15     // Número de LEDS
#define QUEUE_LENGTH 5
#define QUEUE_ITEM_SIZE sizeof(int)


int ADC_PIN = 14;

QueueHandle_t xQueue;
TaskHandle_t LER_ADC;

BluetoothSerial bl;
hw_timer_t *timer = NULL;
Adafruit_NeoPixel strip = Adafruit_NeoPixel(NUM_LEDS, PIN, NEO_GRB + NEO_KHZ800);
void colorWipe(uint32_t color, int wait);
void Task1code(void *pvParameters);

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

TWAI_Interface CAN1(1000, 21, 22); // argument 1 - BaudRate,  argument 2 - CAN_TX PIN,  argument 3 - CAN_RX PIN
int BRAKE  = 5; 
volatile int brake = 0;

volatile int flag = 0;

unsigned long timeout = 0;
unsigned long send = 0;

int light_status = 0;

void IRAM_ATTR onTimer() 
{
        if(bl.available())
        {
          bl.println("I'm alive");
        }
        //Serial.println("I'm here");
        CAN1.TXpacketBegin(0x203,0);
        if(digitalRead(BRAKE))
        {
          CAN1.TXpacketLoad(1);
        }
        else
        {
          CAN1.TXpacketLoad(0);
        }
        CAN1.TXpackettransmit();
}

 

void setup() {
  // put your setup code here, to run once:
  bl.begin("Brake light");
  pinMode(BRAKE,INPUT);
  strip.show();
  strip.clear();

  timer = timerBegin(0, 80, true); // Timer 0, prescaler 80 (1us ticks), count up
  timerAttachInterrupt(timer, &onTimer, true); // Attach the interrupt handler
  timerAlarmWrite(timer, 1000000, true); // 1 second interval
  timerAlarmEnable(timer); // Enable the timer interrupt
  Serial.begin(115200);
  timeout = millis();

   xQueue = xQueueCreate(QUEUE_LENGTH, QUEUE_ITEM_SIZE);
    if (xQueue == NULL)
    {
        Serial.println("Failed to create queue");
        while (1)
        {
          Serial.println("ERRO - Falhao ao criar queue");
        }
    }

  xTaskCreatePinnedToCore(
      Task1code, /* Function to implement the task */
      "Task1", /* Name of the task */
      10000,  /* Stack size in words */
      NULL,  /* Task input parameter */
      0,  /* Priority of the task */
      &LER_ADC,  /* Task handle. */
      0); /* Core where the task should run */

}

void loop() {
  // put your main code here, to run repeatedly:
  if(digitalRead(BRAKE))
  {
    brake = 0;
    if(bl.available())
    {
      bl.println("Braking");
    }
    light_status = 1;

  }
  else
  {
    light_status = 0;
    //Serial.println("NOT");
        if(bl.available())
        {
          bl.println("Not Braking");
        }
  }
  if(CAN1.RXpacketBegin() == 0x500)
  {
      timeout = millis();
      brake = CAN1.RXpacketRead(4);
      if(brake > 10)
      {
        light_status = 1;
      }
      else
      {
        light_status = 0;
      }
  }
  if(timeout <= millis() + 500)
  {
    light_status = 0;
  }

  if (xQueueSend(xQueue, &light_status, portMAX_DELAY) != pdPASS)
  {
      Serial.println("Failed to send to the queue");
  }



    if(millis() >= send + 200)
    {
      volatile float adc_value = analogRead(ADC_PIN);
      volatile float voltage = 6*adc_value/4095;
      volatile float pressao = voltage *  140 / 3.9998;

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




void Task1code( void * parameter) 
{
  volatile int estado = 0;
  int receivedValue;
  while(1)
  {
    if (xQueueReceive(xQueue, &receivedValue, 100 / portTICK_PERIOD_MS) == pdPASS)
    {
          estado = receivedValue;
    }
    if(estado == 1)
    {
      colorWipe(strip.Color(255, 0, 0), 0);
      Serial.println("ON");
    }
    else
    {
      colorWipe(strip.Color(0, 0, 0), 0);
      Serial.println("OFF");
    }
  }
}