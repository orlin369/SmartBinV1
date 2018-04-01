/*

Copyright (c) [2018] [Orlin Dimitrov]

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

*/

#ifndef _APPLICATIONCONFIGURATION_h
#define _APPLICATIONCONFIGURATION_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "Arduino.h"
#else
	#include "WProgram.h"
#endif

#pragma region Definitions

/** @brief Sensor pin that correspond to lid open/close sensor. */
#define PIN_BIN_LID A2

/** @brief Sensor pin that correspond to trash value sensor. */
#define PIN_BIN_TRASH_SENSOR A0

/** @brief Sensor pin that correspond to lid lock sensor. */
#define PIN_RECYCLE_SENSOR 4

/** @brief Sensor pin that correspond to trash incoming sensor for trigger pin. */
#define PIN_US_TRIG A4

/** @brief Sensor pin that correspond to trash incoming sensor for echo pin. */
#define PIN_US_ECHO A5

/** @brief Actuator pin that correspond to lid open/close servo. */
#define PIN_SERVO_LID 5

/** @brief LED pin that correspond to status LED. */
#define PIN_STATUS_LED 13

/** @brief Digital sensors debonnce time [ms]. */
#define BTN_DEBOUNCE_TIME 200

/** @brief Lid open position [deg]. */
#define LID_OPEN_POSITION 140

/** @brief Lid close position [deg]. */
#define LID_CLOSE_POSITION 50

/** @brief Trash incoming distance [cm]. */
#define TRASH_INCOMMING_DISTANCE 25

/** @brief Open debounce time [ms]. */
#define OPEN_DEBOUNCE_TIME 200

/** @brief Close lid time [ms]. */
#define CLOSE_LID_TIME 10000

/** @brief Close step lid time [ms]. */
#define CLOSE_LID_STEP_TIME 2

/** @brief Application run time [ms]. */
#define APP_RUN_TIME 1

/** @brief Update messages run time [ms]. */
#define UPDATE_MESSAGE_TIME 1000

/** @brief LoRa interface integration. */
#define LORA_INTERFACE

#ifdef LORA_INTERFACE

/** @brief Schedule TX every this many seconds (might become longer due to duty cycle limitations). */
#define TX_INTERVAL 60UL

#endif // LORA_INTERFACE

#pragma endregion

#endif

