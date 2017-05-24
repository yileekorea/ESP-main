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
#include "input.h"
#include "output.h"
#include "gpio_MCP23S17.h"   // import library

#define R1_BUILTIN 4

//const uint8_t sclk = 14;
//const uint8_t mosi =13; //Master Output Slave Input ESP8266=Master,MCP23S08=slave
const uint8_t MCP_CS = 15;
gpio_MCP23S17 mcp(MCP_CS,0x20);//instance

Ticker ticker;
/*
unsigned long Timer_1[] = {0,0,0,0,0,0,0,0};
unsigned long Timer_2[] = {0,0,0,0,0,0,0,0};
byte isOFF[] = {0,0,0,0,0,0,0,0};
*/


/*
 * Temperature measurement
 */
void relayControl() {
    byte i;
	if(L_Temp[0] <= celsius[0]){
		digitalWrite(R1_BUILTIN,LOW);
	} else {
		digitalWrite(R1_BUILTIN,HIGH);
	}
  for ( i = 1; i < (numSensor-1) ; i++) {
    rStatus[i] == 0 ? mcp.gpioDigitalWrite(i+3,LOW) : mcp.gpioDigitalWrite(i+3,HIGH);
  }
/*
	for ( i = 1; i < (numSensor-1) ; i++) {
		if((L_Temp[i] <= celsius[i]) && ((millis() - Timer_2[i]) > 60000UL) && (isOFF[i] == 0)) { // 1min

			mcp.gpioDigitalWrite(i+3,LOW); //if current celsius Greater than setting --> off
			Timer_1[i] = millis();
			isOFF[i] = 1;
			DEBUG.print(" isOFF: ");
			DEBUG.print(i);
			DEBUG.println(isOFF[i]);
		}

		if(L_Temp[i] > celsius[i]) {
		  DEBUG.println();
		  DEBUG.print("L_Temp-");
		  DEBUG.print(i);
		  DEBUG.print(" : ");
		  DEBUG.print(L_Temp[i]);
		  DEBUG.print("    celsius-");
		  DEBUG.print(i);
		  DEBUG.print(" : ");
		  DEBUG.println(celsius[i]);
		  DEBUG.println();

		  mcp.gpioDigitalWrite(i+3,HIGH);
		  isOFF[i] = 0;
			}
		else if((millis() - Timer_1[i]) > 180000UL) { //3min
		  DEBUG.println();
		  DEBUG.print(i);
		  DEBUG.print(" : millis-");
		  DEBUG.print(millis());
		  DEBUG.print("  -   vControlTimer-");
		  DEBUG.print(Timer_1[i]);
		  DEBUG.print("  =  ");
		  DEBUG.println((millis() - Timer_1[i]));

		  Timer_1[i] = millis();
		  Timer_2[i] = millis();
		  mcp.gpioDigitalWrite(i+3,HIGH);
		  isOFF[i] = 0;

		}

	} // for
*/
}

void mcp_GPIO_setup() {
  Serial.print("Attempting SPI mcp.begin()...");
  Serial.println();
  mcp.begin(0);//x.begin(1) will override automatic SPI initialization

  pinMode(R1_BUILTIN, OUTPUT);
  mcp.gpioPinMode(0x0F);

  digitalWrite(R1_BUILTIN, HIGH);
  mcp.gpioPort(0xFF);
  delay(1000);
  mcp.gpioPort(0x00);
  digitalWrite(R1_BUILTIN, LOW);

}

void tick(){
  //toggle state
  int state = digitalRead(ESP_LED);  // get the current state of GPIO1 pin
  digitalWrite(ESP_LED, !state);     // set pin to the opposite state
}

void LED_setup(float t) {
  //set led pin as output
  pinMode(ESP_LED, OUTPUT);
  // start ticker with 0.5 because we start in AP mode and try to connect
  ticker.attach(t, tick);
 }

void LED_clear() {
 	ticker.detach();
	//keep LED off
	digitalWrite(ESP_LED, HIGH);
}
