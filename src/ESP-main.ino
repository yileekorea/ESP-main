#include <Arduino.h>

/*
 * -------------------------------------------------------------------
 * ESP-main Serial to ESP-main-2 gateway
 * -------------------------------------------------------------------
 * Adaptation of Chris Howells OpenEVSE ESP Wifi
 * by Trystan Lea, Glyn Hudson, OpenEnergyMonitor
 * All adaptation GNU General Public License as below.
 *
 * -------------------------------------------------------------------
 *
 * This file is part of ESP-main-web project.
 * ESP-main is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3, or (at your option)
 * any later version.
 * ESP-main is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * You should have received a copy of the GNU General Public License
 * along with ESP-main; see the file COPYING.  If not, write to the
 * Free Software Foundation, Inc., 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 */

#include "io2better.h"
#include "config.h"
#include "wifi.h"
#include "web_server.h"
#include "ota.h"
#include "input.h"
#include "output.h"
//#include "ESP-main-2.h"
#include "mqtt.h"



unsigned long tempTry = 1;
int numSensor = 0;


// -------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------
void setup() {
  pinMode(5, OUTPUT);
  digitalWrite(5, HIGH);

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

  // Read saved settings from the config
  config_load_settings();
  //config_save_wifi("iptime_multi_2.4G", "1234");

  // Initialise the WiFi
  wifi_setup();

  // Bring up the web server
  web_server_setup();

  // Start the OTA update systems
  ota_setup();

  if (wifi_mode==WIFI_MODE_STA){
    DEBUG.println("Start finger print verify : ");
    verifyFingerprint();
  }

  mcp_GPIO_setup();

  delay(100);


  DEBUG.println("Loop start");
} // end setup

// -------------------------------------------------------------------
// LOOP
// -------------------------------------------------------------------
void loop()
{
    ota_loop();
    web_server_loop();
    wifi_loop();

  //String input = "test";
  //boolean gotInput = input_get(input);
  if (wifi_mode==WIFI_MODE_STA || wifi_mode==WIFI_MODE_AP_AND_STA)
  {
    if((mqtt_server != 0) && (WiFi.status() == WL_CONNECTED))
    {
  		mqtt_loop();

  		if (tempTry == 0 || ((millis() - tempTry) > 25000UL))  // 25sec
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
} // end loop
