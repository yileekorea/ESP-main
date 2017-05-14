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

#ifndef _EMONESP_INPUT_H
#define _EMONESP_INPUT_H

#include <Arduino.h>

// -------------------------------------------------------------------
// Support for reading input
// -------------------------------------------------------------------

extern String last_datastr;
extern String input_string;

extern String sName[]; 
extern float old_celsius[];
extern float celsius[];
extern float rStatus[];
extern float L_Temp[];

// -------------------------------------------------------------------
// Read input sent via the web_server or serial.
//
// data: if true is returned data will be updated with the new line of
//       input
// -------------------------------------------------------------------
extern boolean input_get(String& data);

extern String readFromOneWire();

#endif // _EMONESP_INPUT_H
