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

#include "NTPClient.h"
#include <WiFiUdp.h>

#include "io2better.h"
#include "config.h"
#include "wifi.h"
#include "web_server.h"
#include "ota.h"
#include "input.h"
#include "output.h"
#include "mqtt.h"

WiFiUDP ntpUDP;
NTPClient timeClient(ntpUDP, "pool.ntp.org", 3600, 60000);

unsigned long tempTry = 0;
int numSensor = 0;
byte s_loop = 0;

int heating_system_status = 1; //ON state is default

unsigned long epochTime;
String formattedTime="";


// -------------------------------------------------------------------
// SETUP
// -------------------------------------------------------------------
void setup() {
  byte i = 0;

  pinMode(ESP_RESET_CTL, OUTPUT);
  digitalWrite(ESP_RESET_CTL, HIGH);
  LED_setup(0.2);
  delay(1000);

  Serial.begin(115200);
#ifdef DEBUG_SERIAL1
  Serial1.begin(115200);
#endif

  Serial.println();
  Serial.print("io2Better ");
  //Serial.println(ESP.getChipId());
  Serial.println(WiFi.macAddress());
  BuildVersion();
  Serial.println("Firmware: "+ currentfirmware);
  Serial.println("buildVersion: "+ buildVersion);

  Serial.println( "Build date: " __DATE__ " " __TIME__  );
  Serial.print( "Framework full version: " );
  Serial.println( ESP.getFullVersion() );

  showChipInfo();

  // Read saved settings from the config
  config_load_settings();
  //config_save_wifi("iptime_multi_2.4G", "1234");

  delay(500);                // longer but it is stable ;-)
  // Initialise the WiFi
  wifi_setup();

  // Bring up the web server
  web_server_setup();

  // Start the OTA update systems
  ota_setup();

  if (wifi_mode==WIFI_MODE_STA){
    Serial.println("Start finger print verify : ");
    verifyFingerprint();

    timeClient.begin();
    // GMT +1 = 3600
    timeClient.setTimeOffset(3600*9);
    timeClient.update();
/*
    configTime( NTP_TIME_SHIFT, 0, NTP_SERVER_NAME );
    // wait for ntp time
    Serial.print( "Wait for NTP sync " );

    while(( now = time(nullptr)) < 1550922262 )
    {
      Serial.print(".");
      delay(100);
    }
      Serial.println( " done." );
      Serial.println(ctime(&now));
      now += KST_time;
      Serial.println(ctime(&now));
*/
  }

  SPIFFS2accHistory();

  SPIFFS2systemSTATUS();

  readOneWireAddr();

  SPIFFS2L_Temp();

  SPIFFS_Timer_1();
  //delay(500);

  INTsetup();

  if (wifi_mode == WIFI_MODE_STA){
      Serial.println("Loop start...");
      LED_clear();
  }
  else{
	   Serial.println("WIFI_MODE is not STA...");
     LED_setup(0.2);
  }

    mcp_GPIO_setup(); //SPI GPIO

    ESP.wdtDisable();
    ESP.wdtEnable(WDTO_8S);


//    delay(500);                // longer but it is stable ;-)

    timeClient.update();
    formattedTime = timeClient.getFormattedTime();
    Serial.print("Formatted Time: ");
    Serial.println(formattedTime);
    epochTime = timeClient.getEpochTime();
    Serial.print("Epoch Time: ");
    Serial.println(epochTime);

    SPIFFS_Timer_1();

} // end setup

// -------------------------------------------------------------------
// LOOP
// -------------------------------------------------------------------
void loop()
{
  ESP.wdtFeed();

  timeClient.update();
  formattedTime = timeClient.getFormattedTime();
//  Serial.print("Formatted Time: ");
//  Serial.println(formattedTime);

  epochTime = timeClient.getEpochTime();
//  Serial.print("Epoch Time: ");
//  Serial.println(epochTime);

  static unsigned long last = millis();
  if (millis() - last > 5000) {
	   last = millis();
	    Serial.printf("[MAIN] Free heap: %d bytes\n", ESP.getFreeHeap());
  }

    ota_loop();

    web_server_loop();
    wifi_loop();
    //valve_relayControl();

  //String input = "test";
  //boolean gotInput = input_get(input);
  if (wifi_mode==WIFI_MODE_STA || wifi_mode==WIFI_MODE_AP_AND_STA)
  {
    if((mqtt_server != 0) && (WiFi.status() == WL_CONNECTED))
    {
  		mqtt_loop();
      //if ((tempTry == 0 || ((millis() - tempTry) > 6000UL))  && mqtt_connected())  // 6sec
      if ((tempTry == 0 || ((millis() - tempTry) > 3000UL))  && 1)  // 3sec
  		//if ((tempTry == 0 || ((millis() - tempTry) > 6000UL))  && 1)  // 6sec
  		{
    			if(INTstateHistory){
    			  INTstateHistory = 0;
    			  INTsetup();
    			  accHistory2SPIFFS();
    			}

    			Serial.println();
    			Serial.println("Firmware: "+ currentfirmware);
    			if ((userTempset == 1)){
            Serial.println("send all sensor temp data");
    				//readFromOneWire();
    				//sendTempData(); //send all sensor temp data
    				userTempset = 0;
    				//readOneWireAddr();
    			}
    			else {
    				//readOneWireAddr();
            //Serial.println("send single sensor temp data");
    				measureTemperature(s_loop);
    				readoutTemperature(s_loop);
    					if (initSending > 0) {
    					  //sendTempData(); //send all sensor temp data
    					  initSending > 0 ? initSending-- : initSending = 0;
    					}
    					else {
    					  send_a_TempData(s_loop);
    					}
              s_loop == (numSensor-1) ? s_loop=0 : s_loop++;
    			}
    			valve_relayControl();
    			tempTry = millis();

  		} //if ((tempTry == 0 ||...
    } //if((mqtt_server != 0) ...
  } //if (wifi_mode==WIFI_MODE_STA ...
} // end loop

void showChipInfo()
{
  Serial.println("-- CHIPINFO --");

  Serial.printf("Chip Id = %08X\n", ESP.getChipId() );
  Serial.printf("CPU Frequency = %dMHz\n", ESP.getCpuFreqMHz() );

  uint32_t realSize = ESP.getFlashChipRealSize();
  uint32_t ideSize = ESP.getFlashChipSize();
  FlashMode_t ideMode = ESP.getFlashChipMode();

  Serial.printf("\nFlash real id:   %08X\n", ESP.getFlashChipId());
  Serial.printf("Flash real size: %u\n", realSize);
  Serial.printf("Flash ide  size: %u\n", ideSize);
  Serial.printf("Flash chip speed: %u\n", ESP.getFlashChipSpeed());
  Serial.printf("Flash ide mode:  %s\n", (ideMode == FM_QIO ?
                                          "QIO" : ideMode == FM_QOUT ?
                                          "QOUT" : ideMode == FM_DIO ?
                                          "DIO" : ideMode == FM_DOUT ?
                                          "DOUT" : "UNKNOWN"));
  if (ideSize != realSize)
  {
    Serial.println("Flash Chip configuration wrong!\n");
  }
  else
  {
    Serial.println("Flash Chip configuration ok.\n");
  }
}
