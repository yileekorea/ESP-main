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

#include <time.h>

#include "io2better.h"
#include "config.h"
#include "wifi.h"
#include "web_server.h"
#include "ota.h"
#include "input.h"
#include "output.h"
//#include "ESP-main-2.h"
#include "mqtt.h"
#include "fauxmoESP.h"

fauxmoESP fauxmo;


unsigned long tempTry = 0;
int numSensor = 0;
byte s_loop = 0;

int heating_system_status = 1; //ON state is default

time_t now = 0l;
time_t lastTimestamp = 0l;
//const unsigned long KST_time = 32400UL;
const unsigned long KST_time = 28800UL;

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

// -------------------------------------------------------------------
// fauxmo_callback
// -------------------------------------------------------------------
// fauxmoESP 2.0.0 has changed the callback signature to add the device_id,
// this way it's easier to match devices to action without having to compare strings.
//fauxmo.onSetState([](unsigned char device_id, const char * device_name, bool state, unsigned char value) {
//Serial.printf("[MAIN] Device #%d (%s) state: %s value: %d\n", device_id, device_name, state ? "ON" : "OFF", value);
//digitalWrite(LED, !state);
//});

void fauxmo_callback(uint8_t device_id, const char * device_name, bool state) {
  Serial.print("Device "); Serial.print(device_name); Serial.print(device_id);
  Serial.print(" state is: ");


  if (state) {
    Serial.println("ON");
  } else {
    Serial.println("OFF");
  }

  if (device_id == 0){
    if (state) {
			Serial.println("heating is ON");
			heating_system_status = 1; //"ON"
      userTempset = 1;

		} else {
			Serial.println("heating is OFF");
			heating_system_status = 0; //"OFF"
      userTempset = 1;

		}
  }
  else if (device_id == 1){
    if (state) {
			Serial.println("relay is ON");
			heating_system_status = 1; //"ON"
		} else {
			Serial.println("relay is OFF");
			heating_system_status = 0; //"OFF"
		}
  }

  systemSTATUS2SPIFFS();
}
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

  DEBUG.println();
  DEBUG.print("io2Better ");
  //DEBUG.println(ESP.getChipId());
  DEBUG.println(WiFi.macAddress());
  BuildVersion();
  DEBUG.println("Firmware: "+ currentfirmware);
  DEBUG.println("buildVersion: "+ buildVersion);

  Serial.println( "Build date: " __DATE__ " " __TIME__  );
  Serial.print( "Framework full version: " );
  Serial.println( ESP.getFullVersion() );

  showChipInfo();

  // Read saved settings from the config
  config_load_settings();
  //config_save_wifi("iptime_multi_2.4G", "1234");

  delay(500);                // longer but it is stable ;-)

  configTime( NTP_TIME_SHIFT, 0, NTP_SERVER_NAME );

  // Initialise the WiFi
  wifi_setup();

  // Bring up the web server
  web_server_setup();

  // Start the OTA update systems
  ota_setup();

  if (wifi_mode==WIFI_MODE_STA){
    DEBUG.println("Start finger print verify : ");
    verifyFingerprint();

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
  }

  SPIFFS2accHistory();

  SPIFFS2systemSTATUS();

  // wireSetup(); //I2C setup

  readOneWireAddr();

  SPIFFS2L_Temp();

  //delay(500);

  INTsetup();

  if (wifi_mode == WIFI_MODE_STA){
      DEBUG.println("Loop start...");
      LED_clear();
  }
  else{
	   DEBUG.println("WIFI_MODE is not STA...");
     LED_setup(0.2);
  }

    mcp_GPIO_setup(); //SPI GPIO

    // Fauxmo
    // You have to call enable(true) once you have a WiFi connection
    // You can enable or disable the library at any moment
    // Disabling it will prevent the devices from being discovered and switched
    fauxmo.enable(true);
    fauxmo.enable(false);
    fauxmo.enable(true);

    fauxmo.addDevice("heating");
    //fauxmo.addDevice("relay");
    fauxmo.onMessage(fauxmo_callback);
    fauxmo.onSetState(fauxmo_callback);

    ESP.wdtDisable();
    ESP.wdtEnable(WDTO_8S);

    for(i=0;i<(numSensor-1);i++){
      Timer_1[i] = millis()-interOFF_Timer;
   }

} // end setup

// -------------------------------------------------------------------
// LOOP
// -------------------------------------------------------------------
void loop()
{
  ESP.wdtFeed();

  //fauxmo.handle();

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
  		//if ((tempTry == 0 || ((millis() - tempTry) > 5000UL))  && 1)  // 3sec
  		{
    			if(INTstateHistory){
    			  INTstateHistory = 0;
    			  INTsetup();
    			  accHistory2SPIFFS();
    			}

    			DEBUG.println();
    			DEBUG.println("Firmware: "+ currentfirmware);
    			if ((userTempset == 1)){
            DEBUG.println("send all sensor temp data");
    				//readFromOneWire();
    				//sendTempData(); //send all sensor temp data
    				userTempset = 0;
    				//readOneWireAddr();
    			}
    			else {
    				//readOneWireAddr();
            //DEBUG.println("send single sensor temp data");
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
