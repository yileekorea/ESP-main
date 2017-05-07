# 1 "c:\\users\\yj\\appdata\\local\\temp\\tmpyejbjx"
#include <Arduino.h>
# 1 "F:/GitHub/ESP-main/src/ESP-main.ino"
#include <Arduino.h>
# 55 "F:/GitHub/ESP-main/src/ESP-main.ino"
#include "emonesp.h"

#include "config.h"

#include "wifi.h"

#include "web_server.h"

#include "ota.h"

#include "input.h"

#include "output.h"

#include "emoncms.h"

#include "mqtt.h"







unsigned long tempTry = 1;

int numSensor = 0;
# 93 "F:/GitHub/ESP-main/src/ESP-main.ino"
void setup();
void loop();
#line 93 "F:/GitHub/ESP-main/src/ESP-main.ino"
void setup() {

  LED_setup(0.2);



  delay(1000);



  Serial.begin(115200);

#ifdef DEBUG_SERIAL1

  Serial1.begin(115200);

#endif



  DEBUG.println();

  DEBUG.print("io2LIFE ");

  DEBUG.println(ESP.getChipId());

  DEBUG.println("Firmware: "+ currentfirmware);





  config_load_settings();







  wifi_setup();





  web_server_setup();





  ota_setup();



  if (wifi_mode==WIFI_MODE_STA){

    verifyFingerprint();

  }



  mcp_GPIO_setup();



  delay(100);





  DEBUG.println("Loop start");

}
# 179 "F:/GitHub/ESP-main/src/ESP-main.ino"
void loop()

{

    ota_loop();

    web_server_loop();

    wifi_loop();







  if (wifi_mode==WIFI_MODE_STA || wifi_mode==WIFI_MODE_AP_AND_STA)

  {

    if((mqtt_server != 0) && (WiFi.status() == WL_CONNECTED))

    {

    mqtt_loop();



    if (tempTry == 0 || ((millis() - tempTry) > 15000UL))

    {

     if(readFromOneWire()) {

      sendTempData();

      relayControl();

          DEBUG.println("Firmware: "+ currentfirmware);

     }

     tempTry = millis();

    }

   }

  }

}