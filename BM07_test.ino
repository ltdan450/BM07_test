#include <Arduino_LSM6DS3.h>

#if CONFIG_FREERTOS_UNICORE
#define ARDUINO_RUNNING_CORE 0
#else
#define ARDUINO_RUNNING_CORE 1
#endif
//#include <LSM6DS3.h>

//#define CG_DESCENT_IMPLEMENTATION
//#include <cg_descent.h>
#define CG_FLOAT float
#define CG_INT int

#include <Wire.h>

#include <WiFi.h>
//#include <WiFiUdp.h>
#include <WiFiUdp.h>

#include "BM_Object.h"
#include <math.h>
#include <assert.h>

//#include <EEPROM.h>
// Variables supplied by customer
const char * networkName = "jimmy";
const char * networkPswd = "meatball";
const float b_earth_mag = 50.8;

//IP address to send UDP data to:
// either use the ip address of the server or
// a network broadcast address
const char * udpAddress = "192.168.0.148";
//const int udpPort = 3333;

//Are we currently connected?
boolean connected = false;

//Global variables
//QueueHandle_t dataqueue;
//WiFiUdp udp;
//WiFiUdp udp;
WiFiUDP udp;
const int udpPort = 27816;


//The udp library class
//WiFiUDP udp;

#define RESET_MC 27
#define SW1 4
#define SW2 5
#define SW3 21

int run_mode = 0; // 0 = ready for anything, 1 = enter cal mode, 2 = cal mode running

void testSys (void *pvParameters) {
  Wire.begin();
  BMSystem sys = BMSystem(1,BMCS());
  //sys.output_queue = dataqueue;
  sys.test_mx_time();
  while(true) {
    sys.mag_test_3();
    vTaskDelay(1000);
  }



}

void testCom (void *pvParameters) {
  bool connected_wifi = false;
  bool connected_server = false;
  IPAddress ip;// = WiFi.local
  IPAddress serv_ip = IPAddress();
  WiFi.macAddress();

  uint8_t mac[6];
  if(WiFiGenericClass::getMode() == WIFI_MODE_NULL){
      esp_read_mac(mac, ESP_MAC_WIFI_STA);
  }



  while(true) {
    if (connected_wifi == false){
      WiFi.begin(networkName,networkPswd);

      while(WiFi.status() != WL_CONNECTED) {
        vTaskDelay(500 / portTICK_PERIOD_MS);
      }
      connected_wifi = true;

      Serial.println("");
      Serial.print("Connected to ");
      Serial.println(networkName);
      Serial.print("IP address: ");
      for (int i = 0; i <4 ; i++) {
        Serial.println(WiFi.localIP()[i]);
      }
      while (!connected_server) {

        ip = WiFi.localIP();
        IPAddress buff_ip = IPAddress();
        buff_ip[0] = ip[0];
        buff_ip[1] = ip[1];
        buff_ip[2] = ip[2];
        uint8_t buffer[50] = "tlick";
        uint8_t inbuff[50] = "      ";
        uint8_t check[6] = "tlock";

        for (uint8_t i = 0; i < 256; i++) {
          buff_ip[3] = i;
          if (buff_ip[3] != ip[3]) {
            udp.beginPacket(buff_ip, udpPort);
            udp.write(buffer, 5);
            udp.endPacket();
            udp.parsePacket();
            Serial.printf("\n sending to ip: ");
            Serial.print(buff_ip);
            vTaskDelay(10);
            int checksum = 0;
            if(udp.read(inbuff,50) > 0){
              Serial.println("\n got here");
              for (int i = 0; i < 5; i++) {
                checksum += (check[i] == inbuff[i]);
              }
              if (checksum>=5) {
                serv_ip = udp.remoteIP();
                Serial.printf("\nconnected to serv_ip ");
                Serial.println(serv_ip);
                connected_server = true;
                break;
              }    
            }
          }
        }
        Serial.printf("connected_server:%d\n",connected_server);
      }
  
      
    

    vTaskDelay(5000/portTICK_PERIOD_MS);
    }






  }



}

void setup()
{
    Serial.begin(230400);
  while (!Serial); 
  Serial.flush();
  Serial.println("Setup started");
  pinMode(RESET_MC, OUTPUT);
  pinMode(SW1, INPUT);
  pinMode(SW2, INPUT);
  pinMode(SW3, INPUT);
  digitalWrite(SW1, LOW);
  digitalWrite(SW2, LOW);
  digitalWrite(SW3, LOW);
  digitalWrite(RESET_MC, HIGH);
  Serial.println("\nI2C Scanner");
  uint16_t a13 = analogRead(A13);
  Serial.printf("\n reading: %d \n", a13);
  //dataqueue = xQueueCreate(5, 24 * sizeof(float));

  //void testCom();
  //delay(5000);

  //testSys();


    if (0)
    xTaskCreatePinnedToCore(
    testCom
    ,  "TaskCommunication"   // A name just for humans
    ,  10240  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);

        xTaskCreatePinnedToCore(
    testSys
    ,  "TaskSystem"   // A name just for humans
    ,  8192  // This stack size can be checked & adjusted by reading the Stack Highwater
    ,  NULL
    ,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
    ,  NULL 
    ,  ARDUINO_RUNNING_CORE);


    
}






int oldmillis = 0;
void loop()
{
  /*
    int t0 = millis();
    int sw1_val;
    int sw2_val;
    int sw3_val;

    sw1_val = digitalRead(SW1);
    sw2_val = digitalRead(SW2);
    sw3_val = digitalRead(SW3);
  */
  vTaskDelay(1000 / portTICK_PERIOD_MS);
}




void TaskBlink(void *pvParameters)  // This is a task.
{
  (void) pvParameters;

/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.
    
  If you want to know what pin the on-board LED is connected to on your ESP32 model, check
  the Technical Specs of your board.
*/

  // initialize digital LED_BUILTIN on pin 13 as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  for (;;) // A Task shall never return or exit.
  {
    digitalWrite(LED_BUILTIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
    digitalWrite(LED_BUILTIN, LOW);    // turn the LED off by making the voltage LOW
    vTaskDelay(100);  // one tick delay (15ms) in between reads for stability
  }
}
