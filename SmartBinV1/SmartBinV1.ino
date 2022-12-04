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

// http://www.instructables.com/id/How-to-setup-a-Pololu-Carrier-with-Sharp-GP2Y0A60S/

#pragma region Headers

/** @brief Application configuration. */
#include "ApplicationConfiguration.h"

/** @brief Servo control library. */
#include <Servo.h>

/** @brief Button denounce library. */
#include <Button.h>

/** @brief HCSR-04 ultrasonic sensor. */
#include <Ultrasonic.h>

#ifdef  LORA_INTERFACE

/** @brief IBM TTN library. */
#include <lmic.h>

/** @brief HopeRF HAL drivers library. */
#include <hal/hal.h>

/** @brief SPI drivers library. */
#include <SPI.h>

#endif //  LORA_INTERFACE

#pragma endregion

#pragma region Constants

#ifdef  LORA_INTERFACE

/** @brief Pin mapping.
 *  @see lmic_pinmap
 */
const lmic_pinmap lmic_pins = 
{
	.nss = 10,
	.rxtx = LMIC_UNUSED_PIN,
	.rst = 9,
	.dio = { 2, 6, 7 },
};

/** @brief LoRaWAN NwkSKey, network session key This is the default Semtech key, which is used by the prototype TTN network initially. */
static const PROGMEM u1_t NWKSKEY[16] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/** @brief LoRaWAN AppSKey, application session key This is the default Semtech key, which is used by the prototype TTN network initially. */
static const PROGMEM u1_t APPSKEY[16] = { 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };

/** @brief LoRaWAN end-device address (DevAddr)
 *  @see http://thethingsnetwork.org/wiki/AddressSpace
 */
static const u4_t DEVADDR = 0xFFFFFFFF;

#endif

#pragma endregion

#pragma region Prototypes

#ifdef  LORA_INTERFACE

/** @brief On OS event handler.
 *  @param ev, event data.
 *  @return Void.
 */
void onEvent(ev_t ev);

/** @brief Do send job handler.
 *  @param j, job data.
 *  @return Void.
 */
void do_send(osjob_t* j);

/** @brief OS get art EUI.
 *  @param buf, data buffer.
 *  @return Void.
 */
void os_getArtEui(u1_t* buf);

/** @brief OS get device EUI.
 *  @param buf, data buffer.
 *  @return Void.
 */
void os_getDevEui(u1_t* buf);

/** @brief OS get device key.
 *  @param buf, data buffer.
 *  @return Void.
 */
void os_getDevKey(u1_t* buf);

#endif

/** @brief Run trough application logic.
 *  @return Void.
 */
void run_app();

/** @brief Unlock the bin lid.
 *  @return Void.
 */
void open_lid();

/** @brief Lock the bin lid.
 *  @return Void.
 */
void close_lid();

/** @brief Read trash value.
 *  @return double, Percentage trash value.
 */
double read_trash_value();

/** @brief Lid recycle callback.
 *  @param binStateMessage, Bin state.
 *  @return Void.
 */
void set_message();

#pragma endregion

#pragma region Variables

#ifdef  LORA_INTERFACE

/** @brief LoRa payload data. */
static uint8_t BinStateMessage_g[7] = {1, 2, 3, 4, 5, 6, 0};

/** @brief OS Send job handle. */
static osjob_t SendJob_g;

#endif
uint8_t IncomingTrashDebone_g = 0;

/** @brief Bin locked/unlocked flag. */
uint8_t RecycleSensorFlag_g = false;

/** @brief Bin trash value in percentage. */
uint8_t  PercentageBatteryLevel_g = 0;

/** @brief Bin trash value in percentage. */
uint8_t  PercentageTrashLevel_g = 0;

/** @brief Lid opening trash counter. */
uint16_t LidOpeningCount_g = 0;

/** @brief Incoming trash distance in [cm]. */
unsigned int IncomingTrashDistance_g = 0;

/** @brief Lid open flag. true = open, false = close. */
bool LidOpenFlag_g = false;

/** @brief Lid open flag. true = open, false = close. */
bool LidSensorFlag_g = false;

/** @brief Lid servo. */
Servo ServoLid_g;

/** @brief Lid sensor. */
Button LidSensor_g(PIN_BIN_LID, BTN_DEBOUNCE_TIME);

/** @brief Lid recycle sensor. */
Button LidRecycleSensor_g(PIN_RECYCLE_SENSOR, BTN_DEBOUNCE_TIME);

/** @brief Incoming trash sensor. */
Ultrasonic IncomingTrashSensor_g(PIN_US_TRIG, PIN_US_ECHO);

#pragma endregion

/** @brief The setup function runs once when you press reset or power the board.
 *  @return Void.
 */
void setup()
{	
	// Debug console.
	Serial.begin(115200);
	Serial.println("Initializing ...");

	// Setup the servo lid.
	ServoLid_g.attach(PIN_SERVO_LID);
	close_lid();

	// Setup the status LED.
	//pinMode(PIN_STATUS_LED, OUTPUT);
	//digitalWrite(PIN_STATUS_LED, HIGH);

	// Set default battery level.
	PercentageBatteryLevel_g = 100;
	
	// Set first message.
	set_message();

#ifdef  LORA_INTERFACE
	
#ifdef VCC_ENABLE
	// For Pinoccio Scout boards
	pinMode(VCC_ENABLE, OUTPUT);
	digitalWrite(VCC_ENABLE, HIGH);
	delay(1000);
#endif

	// LMIC init
	os_init();
	// Reset the MAC state. Session and pending data transfers will be discarded.
	LMIC_reset();

	// Set static session parameters. Instead of dynamically establishing a session
	// by joining the network, precomputed session parameters are be provided.
#ifdef PROGMEM
	// On AVR, these values are stored in flash and only copied to RAM
	// once. Copy them to a temporary buffer here, LMIC_setSession will
	// copy them into a buffer of its own again.
	uint8_t appskey[sizeof(APPSKEY)];
	uint8_t nwkskey[sizeof(NWKSKEY)];
	memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
	memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
	LMIC_setSession(0x1, DEVADDR, nwkskey, appskey);
#else
	// If not running an AVR with PROGMEM, just use the arrays directly
	LMIC_setSession(0x1, DEVADDR, NWKSKEY, APPSKEY);
#endif

#if defined(CFG_eu868)
	// Set up the channels used by the Things Network, which corresponds
	// to the defaults of most gateways. Without this, only three base
	// channels from the LoRaWAN specification are used, which certainly
	// works, so it is good for debugging, but can overload those
	// frequencies, so be sure to configure the full frequency range of
	// your network here (unless your network autoconfigures them).
	// Setting up channels should happen after LMIC_setSession, as that
	// configures the minimal channel set.
	// NA-US channels 0-71 are configured automatically
	LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	//LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);     // g-band
	//LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	//LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	//LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	//LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	//LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	//LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7), BAND_CENTI);      // g-band
	//LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK, DR_FSK), BAND_MILLI);       // g2-band

	// .
	LMIC_disableChannel(1);
	LMIC_disableChannel(2);

	// TTN defines an additional channel at 869.525Mhz using SF9 for class B
	// devices' ping slots. LMIC does not have an easy way to define set this
	// frequency and support for class B is spotty and untested, so this
	// frequency is not configured here.

#elif defined(CFG_us915)
	// NA-US channels 0-71 are configured automatically
	// but only one group of 8 should (a subband) should be active
	// TTN recommends the second sub band, 1 in a zero based count.
	// https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
	LMIC_selectSubBand(1);
#endif

	// Disable link check validation
	LMIC_setLinkCheckMode(0);

	// TTN uses SF9 for its RX2 window.
	LMIC.dn2Dr = DR_SF9;

	// Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
	LMIC_setDrTxpow(DR_SF7, 14);

	// Start job
	do_send(&SendJob_g);

#endif

}

/** @brief The loop function runs over and over again until power down or reset.
 *  @return Void.
 */
void loop()
{
	static unsigned long CurrMillisL = 0, AppPrevMillisL = 0, UpdatePrevMillisL = 0;

	// Update time.
	CurrMillisL = millis();

	// Check for 10 seconds time.
	if (CurrMillisL - AppPrevMillisL >= APP_RUN_TIME)
	{
		// save the last time.
		AppPrevMillisL = CurrMillisL;

		// Run the application.
		run_app();
	}

	// Check for 1 seconds time to update the message content.
	if (CurrMillisL - UpdatePrevMillisL >= UPDATE_MESSAGE_TIME)
	{
		// save the last time.
		UpdatePrevMillisL = CurrMillisL;

		// Set message.
		set_message();
	}
	
#ifdef LORA_INTERFACE

	os_runloop_once();

#endif // LORA_INTERFACE

}

#pragma region Functions

#ifdef  LORA_INTERFACE

/** @brief On OS event handler.
 *  @param ev, event data.
 *  @return Void.
 */
void onEvent(ev_t ev)
{
	Serial.print(os_getTime());
	Serial.print(": ");
	switch (ev) {
	case EV_SCAN_TIMEOUT:
		Serial.println(F("EV_SCAN_TIMEOUT"));
		break;
	case EV_BEACON_FOUND:
		Serial.println(F("EV_BEACON_FOUND"));
		break;
	case EV_BEACON_MISSED:
		Serial.println(F("EV_BEACON_MISSED"));
		break;
	case EV_BEACON_TRACKED:
		Serial.println(F("EV_BEACON_TRACKED"));
		break;
	case EV_JOINING:
		Serial.println(F("EV_JOINING"));
		break;
	case EV_JOINED:
		Serial.println(F("EV_JOINED"));
		break;
	case EV_RFU1:
		Serial.println(F("EV_RFU1"));
		break;
	case EV_JOIN_FAILED:
		Serial.println(F("EV_JOIN_FAILED"));
		break;
	case EV_REJOIN_FAILED:
		Serial.println(F("EV_REJOIN_FAILED"));
		break;
	case EV_TXCOMPLETE:
		Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));

		// Reset lid opening counter.
		LidOpeningCount_g = 0;

		if (LMIC.txrxFlags & TXRX_ACK)
			Serial.println(F("Received ack"));
		if (LMIC.dataLen) {
			Serial.println(F("Received "));
			Serial.println(LMIC.dataLen);
			Serial.println(F(" bytes of payload"));
		}
		// Schedule next transmission
		os_setTimedCallback(&SendJob_g, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
		break;
	case EV_LOST_TSYNC:
		Serial.println(F("EV_LOST_TSYNC"));
		break;
	case EV_RESET:
		Serial.println(F("EV_RESET"));
		break;
	case EV_RXCOMPLETE:
		// data received in ping slot
		Serial.println(F("EV_RXCOMPLETE"));
		break;
	case EV_LINK_DEAD:
		Serial.println(F("EV_LINK_DEAD"));
		break;
	case EV_LINK_ALIVE:
		Serial.println(F("EV_LINK_ALIVE"));
		break;
	default:
		Serial.println(F("Unknown event"));
		break;
	}
}

/** @brief Do send job handler.
 *  @param j, job data.
 *  @return Void.
 */
void do_send(osjob_t* j)
{
	// Check if there is not a current TX/RX job running
	if (LMIC.opmode & OP_TXRXPEND) {
		Serial.println(F("OP_TXRXPEND, not sending"));
	}
	else
	{
		// Prepare upstream data transmission at the next possible time.
		LMIC_setTxData2(1, BinStateMessage_g, sizeof(BinStateMessage_g) - 1, 0);

		// Debug message.
		Serial.println(F("Packet queued"));
	}
	// Next TX is scheduled after TX_COMPLETE event.
}

/** @brief OS get art EUI.
 *  @param buf, data buffer.
 *  @return Void.
 */
void os_getArtEui(u1_t* buf) { }

/** @brief OS get device EUI.
 *  @param buf, data buffer.
 *  @return Void.
 */
void os_getDevEui(u1_t* buf) { }

/** @brief OS get device key.
 *  @param buf, data buffer.
 *  @return Void.
 */
void os_getDevKey(u1_t* buf) { }

#endif

/** @brief Run trough application logic.
 *  @return Void.
 */
void run_app()
{
	// Local variables.
	static long microsec = 0;
	static unsigned long CurrentMillisL = 0, PreviousMillisL = 0;
	
	// Update recycle sensor.
	RecycleSensorFlag_g = (LidRecycleSensor_g.pressed() == HIGH);

	// Update lid sensor states.
	LidSensorFlag_g = (LidSensor_g.pressed() == LOW);

	// Update sensor only if the lid is closed.
	if (LidOpenFlag_g == false)
	{
		// Measure incoming trash value.
		IncomingTrashDistance_g = IncomingTrashSensor_g.read();
	}

	// Update trash value only if lid is closed.
	if (LidOpenFlag_g == false)
	{
		// Measure the trash value.
		PercentageTrashLevel_g = read_trash_value();
	}

	// If the coup is closed then run the logic.
	if (RecycleSensorFlag_g == false)
	{
		// If the trash value is less then 100% then run the logic.
		if (PercentageTrashLevel_g < 100)
		{
			// Wait for incoming trash.
			if (IncomingTrashDistance_g <= TRASH_INCOMING_DISTANCE)
			{
				IncomingTrashDebone_g++;

				if (IncomingTrashDebone_g > OPEN_DEBOUNCE_TIME)
				{
					if (LidOpenFlag_g == false)
					{
						// Open the lid.
						open_lid();

						// Increment lid opening counter.
						LidOpeningCount_g++;
					}
				}
			}
			else
			{
				// Clear debounce time.
				IncomingTrashDebone_g = 0;
			}
		}

		// If the lid is open. Wait 10 seconds to close it.
		if (LidOpenFlag_g == true)
		{
			// Update time.
			CurrentMillisL = millis();

			// Check for 10 seconds time.
			if (CurrentMillisL - PreviousMillisL >= CLOSE_LID_TIME)
			{
				// save the last time you blinked the LED
				PreviousMillisL = CurrentMillisL;

				// Close the lid.
				close_lid();
			}
		}
	}
}

/** @brief Unlock the bin lid.
 *  @return Void.
 */
void open_lid()
{
	for (uint8_t position = LID_CLOSE_POSITION; position < LID_OPEN_POSITION; position++)
	{
		ServoLid_g.write(position);
		delay(CLOSE_LID_STEP_TIME);
	}

	LidOpenFlag_g = true;
}

/** @brief Lock the bin lid.
 *  @return Void.
 */
void close_lid()
{
	for (uint8_t position = LID_OPEN_POSITION; position >= LID_CLOSE_POSITION; position--)
	{
		ServoLid_g.write(position);
		delay(CLOSE_LID_STEP_TIME);
	}

	LidOpenFlag_g = false;
}

/** @brief Read trash value.
 *  @return double, Percentage trash value.
 */
double read_trash_value()
{
	// Read ADC.
	uint16_t SensorValueL = analogRead(PIN_BIN_TRASH_SENSOR);
	
	//Serial.print("Sensor ");
	//Serial.println(SensorValueL);

	// Constraint sensor value.
	SensorValueL = (uint16_t)constrain(SensorValueL, 103, 311);

	// Scale to percentage.
	SensorValueL = (uint16_t)map(SensorValueL, 103, 311, 0, 100);

	// Constraint percentage value.
	double PercentageTrashValueL = constrain(SensorValueL, 0, 100);

	// Return the value.
	return PercentageTrashValueL;
}

/** @brief Lid recycle callback.
 *  @param binStateMessage, Bin state.
 *  @return Void.
 */
void set_message()
{

#ifdef LORA_INTERFACE

	// Set battery level.
	BinStateMessage_g[0] = PercentageBatteryLevel_g;

	// Set trash level.
	BinStateMessage_g[1] = PercentageTrashLevel_g;

	// Set lid count.
	BinStateMessage_g[2] = (uint8_t)(LidOpeningCount_g >> 8);
	BinStateMessage_g[3] = (uint8_t)LidOpeningCount_g;

	// Set check sum.
	BinStateMessage_g[4] = BinStateMessage_g[0] ^ BinStateMessage_g[2];
	BinStateMessage_g[5] = BinStateMessage_g[1] ^ BinStateMessage_g[3];

	Serial.print("PercentageBatteryLevel: ");
	Serial.println(BinStateMessage_g[0]);
	Serial.print("PercentageTrashLevel: ");
	Serial.println(BinStateMessage_g[1]);
	Serial.print("LidOpeningCount: ");
	Serial.println((BinStateMessage_g[2] * 256) + BinStateMessage_g[3]);
	Serial.println("");

#endif //  LORA_INTERFACE

}

#pragma endregion
